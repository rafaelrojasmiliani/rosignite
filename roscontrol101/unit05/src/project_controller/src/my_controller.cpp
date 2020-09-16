/*
 * =====================================================================================
 *
 *       Filename:  my_controller.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  16/09/2020 08:23:35
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *   Organization:
 *
 * =====================================================================================
 */

#include <controller_interface/controller.h> // controller_interface::Controller (template class), controller_interface::ControllerBase
#include <hardware_interface/joint_command_interface.h> // hardware_interface::PositionJointInterface
#include <pluginlib/class_list_macros.h> // PLUGINLIB_EXPORT_CLASS
#include <stdlib.h>

namespace controller_ns {

class PositionController : public controller_interface::Controller<
                               hardware_interface::PositionJointInterface> {
public:
  PositionController()
      : controller_interface::Controller<
            hardware_interface::PositionJointInterface>(),
        gain_(2.25), setpoint_(-2.0) {}
  /** \brief The init function is called to initialize the controller from a
   * non-realtime thread with a pointer to the hardware interface, itself,
   * instead of a pointer to a RobotHW.
   *
   * \param _hardware_interface The specific hardware interface used by this
   * controller.
   *
   * \param _controller_node_handle A NodeHandle in the namespace from which the
   * controller
   * should read its configuration, and where it should set up its ROS
   * interface.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(hardware_interface::PositionJointInterface *_hardware_interface,
            ros::NodeHandle &_controller_node_handle) {
    std::string my_joint;
    bool the_paramenter_exists;
    // Get a string value from the parameter server.
    the_paramenter_exists = _controller_node_handle.getParam("joint", my_joint);
    if (!the_paramenter_exists) {
      ROS_ERROR("Could not find joint name");
      return false;
    }

    joint_ = _hardware_interface->getHandle(my_joint); // throws on failure
    return true;
  }

  /** \brief This is called periodically by the realtime thread when the
   * controller is running.
   * In this case our control is
   * u = error*gain
   * This is a virtual method from controller_interface::ControllerBase
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref update
   */
  void update(const ros::Time & /*_time*/, const ros::Duration & /*_period */) {
    double error = this->setpoint_ - this->joint_.getPosition();
    this->joint_.setCommand(error * this->gain_);
  }

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   * This is a virtual method from controller_interface::ControllerBase
   *
   * \param time The current time
   */
  //  void starting(const ros::Time) {}
  //  void stopping(const ros::Time) {}

private:
  hardware_interface::JointHandle joint_;
  double gain_;
  double setpoint_;
};
PLUGINLIB_EXPORT_CLASS(controller_ns::PositionController,
                       controller_interface::ControllerBase);
}
