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

#ifndef __JOYSTICK_HANDLER_INCLUDED__
#define __JOYSTICK_HANDLER_INCLUDED__

/**
 * Includes
 */
#include "rclcpp/rclcpp.hpp"

#include "hri_c_driver/VehicleMessages.h"
#include "MsgHandler.h"

namespace hri_safety_sense {

  /**
   *
   */
  class JoystickHandler : public MsgHandler {
     public:
      JoystickHandler(
        rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr nodeTopics,
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr nodeLogger,
        rclcpp::node_interfaces::NodeClockInterface::SharedPtr nodeClock);
      ~JoystickHandler();

      uint32_t handleNewMsg(const VscMsgType &incomingMsg);

     private:

      float getStickValue(JoystickType joystick);
      int32_t getButtonValue(uint8_t button);

      rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr nodeLogger;
      rclcpp::node_interfaces::NodeClockInterface::SharedPtr nodeClock;

      rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr rawLeftPub, rawRightPub;
  };

}

#endif
