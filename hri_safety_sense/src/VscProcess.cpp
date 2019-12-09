/*
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * ROS Includes
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int32.hpp"

/**
 * System Includes
 */
#include <chrono>
#include <errno.h>
#include <exception>
#include <functional>
#include <string.h>
#include <math.h>
#include <memory>
#include <system_error>
#include <sys/time.h>
#include <sys/resource.h>

/**
 * Includes
 */
#include "VscProcess.hpp"
#include "JoystickHandler.hpp"
#include "hri_c_driver/VehicleInterface.h"
#include "hri_c_driver/VehicleMessages.h"

namespace hri_safety_sense {

#define THROTTLE(clock, duration, thing) \
{ \
  static rclcpp::Time _last_output_time(0, 0, clock->get_clock_type()); \
  auto _now = clock->now(); \
  if (_now - _last_output_time > duration) { \
    _last_output_time = _now; \
    thing; \
  } \
}

VscProcess::VscProcess(const rclcpp::NodeOptions &node_options) :
  rclcpp::Node("VscProcess", node_options), myEStopState_(0)
{
  std::string serialPort = this->declare_parameter<std::string>("serial",
    "/dev/ttyACM0");
  RCLCPP_INFO(this->get_logger(), "Serial Port set to:  %s", serialPort.c_str());

  int serialSpeed = this->declare_parameter<int>("serial_speed", 115200);
  RCLCPP_INFO(this->get_logger(), "Serial Port Speed set to:  %i", serialSpeed);

  std::string frameId = this->declare_parameter<std::string>("frame_id",
    "/srcs");
  RCLCPP_INFO(this->get_logger(), "Frame ID set to:  %s", frameId.c_str());

  /* Open VSC Interface */
  vscInterface_ = vsc_initialize(serialPort.c_str(), serialSpeed);
  if (vscInterface_ == NULL) {
    RCLCPP_FATAL(this->get_logger(), "Cannot open serial port! (%s, %i)", serialPort.c_str(), serialSpeed);
    throw std::system_error(std::make_error_code(std::errc::io_error), "Cannot open serial port");
  } else {
    RCLCPP_INFO(this->get_logger(), "Connected to VSC on %s : %i", serialPort.c_str(), serialSpeed);
  }

  // Attempt to Set priority
  bool setPriority = this->declare_parameter<bool>("set_priority", false);
  RCLCPP_INFO(this->get_logger(), "Set priority:  %i", setPriority);

  if (setPriority) {
    if (setpriority(PRIO_PROCESS, 0, -19) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Unable to set priority of process! (%i, %s)", errno, strerror(errno));
      throw std::system_error(errno, std::system_category(), strerror(errno));
    }
  }

  // Create Message Handlers
  joystickHandler_ = new JoystickHandler(this->get_node_topics_interface(),
    this->get_node_logging_interface(), this->get_node_clock_interface(),
    frameId);

  // EStop callback
  estopServ_ = this->create_service<hri_safety_sense_srvs::srv::EmergencyStop>(
    "safety/service/send_emergency_stop", std::bind(&VscProcess::EmergencyStop,
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // KeyValue callbacks
  keyValueServ_ = this->create_service<hri_safety_sense_srvs::srv::KeyValue>(
    "safety/service/key_value", std::bind(&VscProcess::KeyValue, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  keyStringServ_ = this->create_service<hri_safety_sense_srvs::srv::KeyString>(
    "safety/service/key_string", std::bind(&VscProcess::KeyString, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Publish Emergency Stop Status
  estopPub_ = this->create_publisher<std_msgs::msg::UInt32>(
    "safety/emergency_stop", 10);

  // Main Loop Timer Callback
  mainLoopTimer_ = this->create_wall_timer(
    rclcpp::Duration(1.0 / VSC_INTERFACE_RATE * 10e9).to_chrono<std::chrono::nanoseconds>(),
    std::bind(&VscProcess::processOneLoop, this));

  // Init last time to now
  lastDataRx_ = this->now();

  // Clear all error counters
  memset(&errorCounts_, 0, sizeof(errorCounts_));
}

VscProcess::~VscProcess()
{
  // Destroy vscInterface
  vsc_cleanup(vscInterface_);

  if(joystickHandler_) delete joystickHandler_;
}

bool VscProcess::EmergencyStop(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<hri_safety_sense_srvs::srv::EmergencyStop::Request> req,
  const std::shared_ptr<hri_safety_sense_srvs::srv::EmergencyStop::Response> /*res*/)
{
  myEStopState_ = static_cast<uint32_t>(req->emergency_stop);

  RCLCPP_WARN(this->get_logger(), "VscProcess::EmergencyStop: to 0x%x", myEStopState_);

  return true;
}

bool VscProcess::KeyValue(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<hri_safety_sense_srvs::srv::KeyValue::Request> req,
  const std::shared_ptr<hri_safety_sense_srvs::srv::KeyValue::Response> /*res*/)
{
  // Send heartbeat message to vehicle in every state
  vsc_send_user_feedback(vscInterface_, req->key, req->value);

  RCLCPP_INFO(this->get_logger(), "VscProcess::KeyValue: 0x%x, 0x%x", req->key, req->value);

  return true;
}

bool VscProcess::KeyString(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<hri_safety_sense_srvs::srv::KeyString::Request> req,
  const std::shared_ptr<hri_safety_sense_srvs::srv::KeyString::Response> /*res*/)
{
  // Send heartbeat message to vehicle in every state
  vsc_send_user_feedback_string(vscInterface_, req->key, req->value.c_str());

  RCLCPP_INFO(this->get_logger(), "VscProcess::KeyString: 0x%x, %s", req->key, req->value.c_str());

  return true;
}

void VscProcess::processOneLoop()
{
  // Send heartbeat message to vehicle in every state
  vsc_send_heartbeat(vscInterface_, myEStopState_);

  // Check for new data from vehicle in every state
  readFromVehicle();
}

int VscProcess::handleHeartbeatMsg(VscMsgType& recvMsg)
{
  int retVal = 0;

  if(recvMsg.msg.meta.length == sizeof(HeartbeatMsgType)) {
    RCLCPP_DEBUG(this->get_logger(), "Received Heartbeat from VSC");

    HeartbeatMsgType *msgPtr = (HeartbeatMsgType*)recvMsg.msg.meta.data;

    // Publish E-STOP Values
    std_msgs::msg::UInt32 estopValue;
    estopValue.data = msgPtr->EStopStatus;
    estopPub_->publish(estopValue);

    if(msgPtr->EStopStatus > 0) {
      RCLCPP_WARN(this->get_logger(), "Received ESTOP from the vehicle!!! 0x%x", msgPtr->EStopStatus);
    }

  } else {
    RCLCPP_WARN(this->get_logger(), "RECEIVED HEARTBEAT WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
      static_cast<unsigned int>(sizeof(HeartbeatMsgType)),
      recvMsg.msg.meta.length);
    retVal = 1;
  }

  return retVal;
}

void VscProcess::readFromVehicle()
{
  VscMsgType recvMsg;

  /* Read all messages */
  while (vsc_read_next_msg(vscInterface_, &recvMsg) > 0) {
    /* Read next Vsc Message */
    switch (recvMsg.msg.meta.msgType) {
    case MSG_VSC_HEARTBEAT:
      if(handleHeartbeatMsg(recvMsg) == 0) {
        lastDataRx_ = this->now();
      }

      break;
    case MSG_VSC_JOYSTICK:
      if(joystickHandler_->handleNewMsg(recvMsg) == 0) {
        lastDataRx_ = this->now();
      }

      break;

    case MSG_VSC_NMEA_STRING:
//      handleGpsMsg(&recvMsg);

      break;
    case MSG_USER_FEEDBACK:
//      handleFeedbackMsg(&recvMsg);

      break;
    default:
      errorCounts_.invalidRxMsgCount++;
      RCLCPP_ERROR(this->get_logger(), "Receive Error.  Invalid MsgType (0x%02X)", recvMsg.msg.meta.msgType);
      break;
    }
  }

  // Log warning when no data is received
  rclcpp::Duration noDataDuration = this->now() - lastDataRx_;
  if(noDataDuration > rclcpp::Duration(0, 250000000)) {
    // TODO(ros2/rclcpp#879) RCLCPP_THROTTLE_WARN() when released
    THROTTLE(this->get_clock(), std::chrono::nanoseconds(500000000),
      RCLCPP_WARN(this->get_logger(), "No Data Received in %g.%09i seconds",
      noDataDuration.seconds(), noDataDuration.nanoseconds()));
  }

}
}

