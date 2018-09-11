#ifndef JAGUAR_BASE_JAGUAR_HARDWARE_HPP
#define JAGUAR_BASE_JAGUAR_HARDWARE_HPP

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "jaguar_msgs/JaguarStatus.h"
#include "sensor_msgs/NavSatFix.h"
#include <string>
#include <jaguar_base/drrobot_wrapper.hpp>

namespace jaguar_base
{

  /**
  * Class representing Jaguar hardware, allows for ros_control to modify internal state via joint interfaces
  */
class JaguarHardware : public hardware_interface::RobotHW{

  public:

    JaguarHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);

    void updateJointsFromHardware();

    void writeCommandsToHardware();

    void updateSensors(bool imu, bool gps, bool status);

    void publishSensors(bool imu, bool gps, bool status);

    void releaseEBrake();


  private:

    void resetTravelOffset();

    void registerControlInterfaces();

    double linearToAngular(const double &travel) const;

    double angularToLinear(const double &angle) const;

    void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

    double getMotorsTravel(const int motor_index);

    double getLinearVelocity(const int motor_index);

    ros::NodeHandle nh_, private_nh_;

    drrobot_wrapper::DrRobotWrapper wrapper;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    // Diagnostics
    ros::Publisher gps_publisher_;
    ros::Publisher status_publisher_;
    sensor_msgs::NavSatFix gps_status;
    jaguar_msgs::JaguarStatus jaguar_status_msg_;

    // ROS Parameters
    std::string robotType_;
    std::string robotID_;
    std::string robotIP_;
    std::string robotCommMethod_;
    std::string robotSerialPort_;
    std::string base_frame_;

    double wheel_diameter_;
    double maxSpeed_;

    int commPortNum_;
    int encoderOneCircleCnt_;
    int message_queue_;
    int battery_packs_;

    bool use_IMU_;
    bool use_GPS_;
    bool use_motors_;
    bool use_status_;

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint{
      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() :
        position(0), velocity(0), effort(0), velocity_command(0){ }
    } joints_[4];
  };

}  // namespace jaguar_base
#endif  // JAGUAR_BASE_JAGUAR_HARDWARE_H
