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
#include "rclcpp/rclcpp.h"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/uint32.hpp"

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
using namespace hri_safety_sense_srvs;

VscProcess::VscProcess() :
	Node("VscProcess"), myEStopState(0)
{
	std::string serialPort = "/dev/ttyACM0";
  serialPort = this->declare_parameter<std::string>("serial");
  if(this->get_parameter<std::string>("serial", serialPort)) {
		RCLCPP_INFO("Serial Port updated to:  %s",serialPort.c_str());
	}

	int  serialSpeed = 115200;
  serialSpeed = this->declare_parameter<int>("serial_speed");
	if(this->get_parameter<int>("serial_speed", serialSpeed)) {
		RCLCPP_INFO("Serial Port Speed updated to:  %i",serialSpeed);
	}

	std::string frameId = "/srcs";
	if(nh.getParam("frame_id", frameId)) {
		ROS_INFO("Frame ID updated to:  %s",frameId.c_str());
	}

	/* Open VSC Interface */
	vscInterface = vsc_initialize(serialPort.c_str(),serialSpeed);
	if (vscInterface == NULL) {
		RCLCPP_FATAL("Cannot open serial port! (%s, %i)",serialPort.c_str(),serialSpeed);
	} else {
		RCLCPP_INFO("Connected to VSC on %s : %i",serialPort.c_str(),serialSpeed);
	}

	// Attempt to Set priority
	bool  set_priority = false;
  set_priority = this->declare_parameter<bool>("set_priority");
	if(this->get_parameter<bool>("set_priority", set_priority)) {
		RCLCPP_INFO("Set priority updated to:  %i",set_priority);
	}

	if(set_priority) {
		if(setpriority(PRIO_PROCESS, 0, -19) == -1) {
			RCLCPP_ERROR("UNABLE TO SET PRIORITY OF PROCESS! (%i, %s)",errno,strerror(errno));
		}
	}

	// Create Message Handlers
	joystickHandler = new JoystickHandler(frameId);

	// EStop callback
	estopServ = this->create_service<hri_safety_sense::EmergencyStop>(
		"safety/service/send_emergency_stop", std::bind(&VscProcess::EmergencyStop,
		this, _1, _2));

	// KeyValue callbacks
	keyValueServ = this->create_service<hri_safety_sense::KeyValue>(
		"safety/service/key_value", std::bind(&VscProcess::KeyValue, this, _1, _2));
	keyStringServ = this->create_service<hri_safety_sense::KeyValue>(
		"safety/service/key_string", std::bind(&VscProcess::KeyString, this, _1,
		_2));

	// Publish Emergency Stop Status
	estopPub = this->create_publisher<std_msgs::msg::UInt32>(
    "safety/emergency_stop", 10);

	// Main Loop Timer Callback
	mainLoopTimer = this->create_wall_timer(
    rclcpp::Duration(1.0/VSC_INTERFACE_RATE),
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

bool VscProcess::EmergencyStop(EmergencyStop::Request  &req, EmergencyStop::Response &res )
{
	myEStopState = (uint32_t)req.EmergencyStop;

	RCLCPP_WARN("VscProcess::EmergencyStop: to 0x%x", myEStopState);

	return true;
}

bool VscProcess::KeyValue(KeyValue::Request  &req, KeyValue::Response &res )
{
	// Send heartbeat message to vehicle in every state
	vsc_send_user_feedback(vscInterface, req.Key, req.Value);

	RCLCPP_INFO("VscProcess::KeyValue: 0x%x, 0x%x", req.Key, req.Value);

	return true;
}

bool VscProcess::KeyString(KeyString::Request  &req, KeyString::Response &res )
{
	// Send heartbeat message to vehicle in every state
	vsc_send_user_feedback_string(vscInterface, req.Key, req.Value.c_str());

	RCLCPP_INFO("VscProcess::KeyValue: 0x%x, %s", req.Key, req.Value.c_str());

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

	if(recvMsg.msg.length == sizeof(HeartbeatMsgType)) {
		RCLCPP_DEBUG("Received Heartbeat from VSC");

		HeartbeatMsgType *msgPtr = (HeartbeatMsgType*)recvMsg.msg.data;

		// Publish E-STOP Values
		std_msgs::msg::UInt32 estopValue;
		estopValue.data = msgPtr->EStopStatus;
		estopPub.publish(estopValue);

		if(msgPtr->EStopStatus > 0) {
			RCLCPP_WARN("Received ESTOP from the vehicle!!! 0x%x",msgPtr->EStopStatus);
		}

	} else {
		RCLCPP_WARN("RECEIVED HEARTBEAT WITH INVALID MESSAGE SIZE! Expected: 0x%x, Actual: 0x%x",
				(unsigned int)sizeof(HeartbeatMsgType), recvMsg.msg.length);
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
		switch (recvMsg.msg.msgType) {
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
//			handleGpsMsg(&recvMsg);

			break;
		case MSG_USER_FEEDBACK:
//			handleFeedbackMsg(&recvMsg);

			break;
		default:
			errorCounts.invalidRxMsgCount++;
			RCLCPP_ERROR("Receive Error.  Invalid MsgType (0x%02X)",recvMsg.msg.msgType);
			break;
		}
	}

	// Log warning when no data is received
	rclcpp::Duration noDataDuration = this->now() - lastDataRx;
	if(noDataDuration > rclcpp::Duration(.25)) {
		RCLCPP_WARN_THROTTLE(.5, "No Data Received in %i.%09i seconds", noDataDuration.sec, noDataDuration.nsec );
	}

}

