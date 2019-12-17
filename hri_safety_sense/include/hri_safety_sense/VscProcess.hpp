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

#include <memory>

/**
 * ROS Includes
 */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

/**
 * HRI_COMMON Includes
 */
#include "hri_c_driver/VehicleMessages.h"
#include "hri_c_driver/VehicleInterface.h"

#include "hri_safety_sense_srvs/srv/emergency_stop.hpp"
#include "hri_safety_sense_srvs/srv/key_value.hpp"
#include "hri_safety_sense_srvs/srv/key_string.hpp"

#include "hri_safety_sense/MsgHandler.hpp"

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

  class VscProcess final : public rclcpp::Node {
    public:

      explicit VscProcess(const rclcpp::NodeOptions &node_options);
      VscProcess(VscProcess &&c) = delete;
      VscProcess &operator=(VscProcess &&c) = delete;
      VscProcess(const VscProcess &c) = delete;
      VscProcess &operator=(const VscProcess &c) = delete;

      ~VscProcess() override;

    private:

      // Main loop
      void processOneLoop();

      // ROS Callbacks
      bool EmergencyStop(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hri_safety_sense_srvs::srv::EmergencyStop::Request> req,
        const std::shared_ptr<hri_safety_sense_srvs::srv::EmergencyStop::Response> res);
      bool KeyValue(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hri_safety_sense_srvs::srv::KeyValue::Request> req,
        const std::shared_ptr<hri_safety_sense_srvs::srv::KeyValue::Response> res);
      bool KeyString(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<hri_safety_sense_srvs::srv::KeyString::Request> req,
        const std::shared_ptr<hri_safety_sense_srvs::srv::KeyString::Response> res);

      void readFromVehicle();
      int handleHeartbeatMsg(const VscMsgType& recvMsg);

      // Local State
      uint32_t myEStopState_;
      ErrorCounterType errorCounts_{};

      // ROS
      rclcpp::TimerBase::SharedPtr mainLoopTimer_;
      rclcpp::Service<hri_safety_sense_srvs::srv::EmergencyStop>::SharedPtr estopServ_;
      rclcpp::Service<hri_safety_sense_srvs::srv::KeyValue>::SharedPtr keyValueServ_;
      rclcpp::Service<hri_safety_sense_srvs::srv::KeyString>::SharedPtr keyStringServ_;
      rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr estopPub_;
      int prevEStop_{-1};
      rclcpp::Time lastDataRx_;

      // Message Handlers
      std::unique_ptr<MsgHandler> joystickHandler_;

      /* File descriptor for VSC Interface */
      VscInterfaceType *vscInterface_;
  };

}  // namespace hri_safety_sense

#endif
