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

#ifndef __VSC_PROCESS_INCLUDED__
#define __VSC_PROCESS_INCLUDED__

/**
 * ROS Includes
 */
#include "hri_safety_sense_msgs/srv/emergency_stop.hpp"
#include "hri_safety_sense_msgs/srv/key_value.hpp"
#include "hri_safety_sense_msgs/srv/key_string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

/**
 * HRI_COMMON Includes
 */
#include "MsgHandler.h"
#include "hri_c_driver/VehicleMessages.h"
#include "hri_c_driver/VehicleInterface.h"

using namespace hri_safety_sense_srvs;

namespace hri_safety_sense {

	// Diagnostics
	struct ErrorCounterType {
		uint32_t sendErrorCount;
		uint32_t invalidRxMsgCount;
	};

	/**
	 * Local Definitions
	 */
	const unsigned int VSC_INTERFACE_RATE = 50; /* 50 Hz */
	const unsigned int VSC_HEARTBEAT_RATE = 20; /* 20 Hz */

	class VscProcess : public rclcpp::Node {
	   public:
		  VscProcess();
		  ~VscProcess();

		  // Main loop
		  void processOneLoop();

		  // ROS Callback's
		  bool EmergencyStop(hri_safety_sense_msgs::srv::EmergencyStop::Request &req, hri_safety_sense_msgs::srv::EmergencyStop::Response &res);
		  bool KeyValue(hri_safety_sense_msgs::srv::KeyValue::Request &req, hri_safety_sense_msgs::srv::KeyValue::Response &res);
		  bool KeyString(hri_safety_sense_msgs::srv::KeyString::Request &req, hri_safety_sense_msgs::srv::KeyString::Response &res);

	   private:

		  void readFromVehicle();
		  int handleHeartbeatMsg(VscMsgType& recvMsg);

		  // Local State
		  uint32_t 				myEStopState;
		  ErrorCounterType 		errorCounts;

		  // ROS
		  rclcpp::TimerBase::SharedPtr		mainLoopTimer;
		  rclcpp::Service<hri_safety_sense_msgs::srv::EmergencyStop>::SharedPtr    estopServ;
      rclcpp::Service<hri_safety_sense_msgs::srv::KeyValue>::SharedPtr		keyValueServ, keyStringServ;
		  rclcpp::Publisher<std_msgs::msg::UInt32>		estopPub;
		  rclcpp::Time 			lastDataRx, lastTxTime;

		  // Message Handlers
		  MsgHandler			*joystickHandler;

		  /* File descriptor for VSC Interface */
		  VscInterfaceType		*vscInterface;

	};

} // namespace


#endif
