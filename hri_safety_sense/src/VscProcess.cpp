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

/**
 * System Includes
 */
#include <errno.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <sys/resource.h>

/**
 * Includes
 */
#include "VscProcess.h"
#include "JoystickHandler.h"
#include "hri_c_driver/VehicleInterface.h"
#include "hri_c_driver/VehicleMessages.h"

using namespace std::placeholders;
using namespace hri_safety_sense;

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

VscProcess::VscProcess(rclcpp::NodeOptions &node_options) :
  rclcpp::Node("VscProcess", node_options), myEStopState(0)
{
  std::string serialPort = "/dev/ttyACM0";
  this->declare_parameter<std::string>("serial", serialPort);
  if (this->get_parameter_or<std::string>("serial", serialPort, serialPort))
    RCLCPP_INFO(this->get_logger(), "Serial Port updated to:  %s", serialPort.c_str());

  int serialSpeed = 115200;
  this->declare_parameter<int>("serial_speed", serialSpeed);
  if (this->get_parameter_or<int>("serial_speed", serialSpeed, serialSpeed))
    RCLCPP_INFO(this->get_logger(), "Serial Port Speed updated to:  %i", serialSpeed);

  std::string frameId = "/srcs";
  this->declare_parameter<std::string>("frame_id", frameId);
  if (this->get_parameter_or<std::string>("frame_id", frameId, frameId))
    RCLCPP_INFO(this->get_logger(), "Frame ID updated to:  %s", frameId.c_str());

  /* Open VSC Interface */
  vscInterface = vsc_initialize(serialPort.c_str(), serialSpeed);
  if (vscInterface == NULL) {
    RCLCPP_FATAL(this->get_logger(), "Cannot open serial port! (%s, %i)", serialPort.c_str(), serialSpeed);
  } else {
    RCLCPP_INFO(this->get_logger(), "Connected to VSC on %s : %i", serialPort.c_str(), serialSpeed);
  }

  // Attempt to Set priority
  bool setPriority = false;
  this->declare_parameter<bool>("set_priority", setPriority);
  if (this->get_parameter_or<bool>("set_priority", setPriority, setPriority))
    RCLCPP_INFO(this->get_logger(), "Set priority updated to:  %i", setPriority);

  if(setPriority) {
    if(setpriority(PRIO_PROCESS, 0, -19) == -1) {
      RCLCPP_ERROR(this->get_logger(), "UNABLE TO SET PRIORITY OF PROCESS! (%i, %s)", errno, strerror(errno));
    }
  }

  // Create Message Handlers
  joystickHandler = new JoystickHandler(this->get_node_topics_interface(),
    this->get_node_logging_interface(), this->get_node_clock_interface(),
    frameId);

  // EStop callback
  estopServ = this->create_service<hri_safety_sense_srvs::srv::EmergencyStop>(
    "safety/service/send_emergency_stop", std::bind(&VscProcess::EmergencyStop,
    this, _1, _2, _3));

  // KeyValue callbacks
  keyValueServ = this->create_service<hri_safety_sense_srvs::srv::KeyValue>(
    "safety/service/key_value", std::bind(&VscProcess::KeyValue, this, _1, _2, _3));
  keyStringServ = this->create_service<hri_safety_sense_srvs::srv::KeyString>(
    "safety/service/key_string", std::bind(&VscProcess::KeyString, this, _1, _2, _3));

  // Publish Emergency Stop Status
  estopPub = this->create_publisher<std_msgs::msg::UInt32>(
    "safety/emergency_stop", 10);

  // Main Loop Timer Callback
  mainLoopTimer = this->create_wall_timer(
    rclcpp::Duration(1.0 / VSC_INTERFACE_RATE * 10e9).to_chrono<std::chrono::nanoseconds>(),
    std::bind(&VscProcess::processOneLoop, this));

  // Init last time to now
  lastDataRx = this->now();

  // Clear all error counters
  memset(&errorCounts, 0, sizeof(errorCounts));
}

VscProcess::~VscProcess()
{
  // Destroy vscInterface
  vsc_cleanup(vscInterface);

  if(joystickHandler) delete joystickHandler;
}

bool VscProcess::EmergencyStop(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<hri_safety_sense_srvs::srv::EmergencyStop::Request> req,
  const std::shared_ptr<hri_safety_sense_srvs::srv::EmergencyStop::Response> /*res*/)
{
  myEStopState = (uint32_t)req->emergency_stop;

  RCLCPP_WARN(this->get_logger(), "VscProcess::EmergencyStop: to 0x%x", myEStopState);

  return true;
}

bool VscProcess::KeyValue(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<hri_safety_sense_srvs::srv::KeyValue::Request> req,
  const std::shared_ptr<hri_safety_sense_srvs::srv::KeyValue::Response> /*res*/)
{
  // Send heartbeat message to vehicle in every state
  vsc_send_user_feedback(vscInterface, req->key, req->value);

  RCLCPP_INFO(this->get_logger(), "VscProcess::KeyValue: 0x%x, 0x%x", req->key, req->value);

  return true;
}

bool VscProcess::KeyString(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<hri_safety_sense_srvs::srv::KeyString::Request> req,
  const std::shared_ptr<hri_safety_sense_srvs::srv::KeyString::Response> /*res*/)
{
  // Send heartbeat message to vehicle in every state
  vsc_send_user_feedback_string(vscInterface, req->key, req->value.c_str());

  RCLCPP_INFO(this->get_logger(), "VscProcess::KeyString: 0x%x, %s", req->key, req->value.c_str());

  return true;
}

void VscProcess::processOneLoop()
{
  // Send heartbeat message to vehicle in every state
  vsc_send_heartbeat(vscInterface, myEStopState);

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
    estopPub->publish(estopValue);

    if(msgPtr->EStopStatus > 0) {
      RCLCPP_WARN(this->get_logger(), "Received ESTOP from the vehicle!!! 0x%x", msgPtr->EStopStatus);
    }

  } else {
    RCLCPP_WARN(this->get_logger(), "RECEIVED HEARTBEAT WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
        (unsigned int)sizeof(HeartbeatMsgType), recvMsg.msg.meta.length);
    retVal = 1;
  }

  return retVal;
}

void VscProcess::readFromVehicle()
{
  VscMsgType recvMsg;

  /* Read all messages */
  while (vsc_read_next_msg(vscInterface, &recvMsg) > 0) {
    /* Read next Vsc Message */
    switch (recvMsg.msg.meta.msgType) {
    case MSG_VSC_HEARTBEAT:
      if(handleHeartbeatMsg(recvMsg) == 0) {
        lastDataRx = this->now();
      }

      break;
    case MSG_VSC_JOYSTICK:
      if(joystickHandler->handleNewMsg(recvMsg) == 0) {
        lastDataRx = this->now();
      }

      break;

    case MSG_VSC_NMEA_STRING:
//      handleGpsMsg(&recvMsg);

      break;
    case MSG_USER_FEEDBACK:
//      handleFeedbackMsg(&recvMsg);

      break;
    default:
      errorCounts.invalidRxMsgCount++;
      RCLCPP_ERROR(this->get_logger(), "Receive Error.  Invalid MsgType (0x%02X)", recvMsg.msg.meta.msgType);
      break;
    }
  }

  // Log warning when no data is received
  rclcpp::Duration noDataDuration = this->now() - lastDataRx;
  if(noDataDuration > rclcpp::Duration(0, 250000000)) {
    // TODO(ros2/rclcpp#879) RCLCPP_THROTTLE_WARN() when released
    THROTTLE(this->get_clock(), std::chrono::nanoseconds(500000000),
      RCLCPP_WARN(this->get_logger(), "No Data Received in %g.%09i seconds",
      noDataDuration.seconds(), noDataDuration.nanoseconds()));
  }

}
}

