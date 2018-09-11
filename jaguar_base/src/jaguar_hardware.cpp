#include "jaguar_hardware.hpp"
#include <boost/assign/list_of.hpp>
#define pi 3.14159265358979323846

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
};

using namespace std;

namespace jaguar_base{

  /**
  * Initialize Jaguar hardware
  */
  JaguarHardware::JaguarHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
  :
  nh_(nh), 
  private_nh_(private_nh){

    private_nh_.param<string>("RobotID", robotID_, "Roboskel_Roula");
    private_nh_.param<string>("RobotType", robotType_, "Jaguar");
    private_nh_.param<string>("RobotCommMethod", robotCommMethod_, "Network");
    private_nh_.param<string>("RobotBaseIP", robotIP_, "192.168.0.60");
    private_nh_.param<string>("RobotSerialPort",robotSerialPort_, "dev/ttyS0");
    private_nh_.param<string>("base_frame",base_frame_, "base_link");

    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.26);
    private_nh_.param<double>("MaxSpeed", maxSpeed_, 2.0);

    private_nh_.param<int>("battery_packs", battery_packs_, 3);
    private_nh_.param<int>("EncoderCircleCnt", encoderOneCircleCnt_, 512);
    private_nh_.param<int>("RobotPortNum", commPortNum_, 10001);
    private_nh_.param<int>("max_message_queue", message_queue_, 10);

    private_nh_.param<bool>("use_IMU", use_IMU_, true);
    private_nh_.param<bool>("use_GPS", use_GPS_, true);
    private_nh_.param<bool>("use_motors", use_motors_, true);
    private_nh_.param<bool>("use_status", use_status_, true);

    wrapper = drrobot_wrapper::DrRobotWrapper(robotID_, robotType_, robotCommMethod_, robotIP_, commPortNum_, robotSerialPort_,
                                                encoderOneCircleCnt_, message_queue_, use_IMU_, use_GPS_, use_motors_, use_status_, battery_packs_);

    gps_publisher_ = nh.advertise<sensor_msgs::NavSatFix>("gps_status", 10);
    status_publisher_ = nh_.advertise<jaguar_msgs::JaguarStatus>("jaguar_status", 10);

    resetTravelOffset();
    registerControlInterfaces();

    wrapper.updateStatus();
  }

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  */
  void JaguarHardware::resetTravelOffset(){
    wrapper.updateMotors();
    if (wrapper.motor_data_queue_.size() > 0){
      for (int i = 0; i < 4; i++){
        double travel = getMotorsTravel(i);
        joints_[i].position_offset = travel;//linearToAngular(travel);
      }
    }
    else{
      ROS_ERROR("Could not get encoder data to calibrate travel offset");
    }
  }


  /**
  * Register interfaces with the RobotHW interface manager, allowing ros_control operation
  */
  void JaguarHardware::registerControlInterfaces(){
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel")
      ("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
    for (unsigned int i = 0; i < joint_names.size(); i++){
      hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                  &joints_[i].position, &joints_[i].velocity,
                                  &joints_[i].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      hardware_interface::JointHandle joint_handle(
        joint_state_handle, &joints_[i].velocity_command);
      velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);

    sleep(3);

    releaseEBrake();
  }

  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void JaguarHardware::updateJointsFromHardware(){

    wrapper.updateMotors();
    if (wrapper.motor_data_queue_.size() > 0){
      for (int i = 0; i < 4; i++){
        double delta = 0;
        if(i % 2 == 0){//Jaguar's motors are reversed on one side!!
          delta = getMotorsTravel(i) - joints_[i].position - joints_[i].position_offset;
          joints_[i].velocity = getLinearVelocity(i);
        }
        else{
          delta = - getMotorsTravel(i) - joints_[i].position - joints_[i].position_offset;
          joints_[i].velocity = -getLinearVelocity(i);
        }

        // detect suspiciously large readings, possibly from encoder rollover
        if (std::abs(delta) < 10.0){
          joints_[i].position += delta;
        }
        else{
          // suspicious! drop this measurement and update the offset for subsequent readings
          joints_[i].position_offset += delta;
          ROS_DEBUG("Dropping overflow measurement from encoder");
        }
      }
    }
  }

  /**
  * Get latest velocity commands from ros_control via joint structure, and send to MCU
  */
  void JaguarHardware::writeCommandsToHardware(){
    double diff_speed_left = angularToLinear(joints_[LEFT].velocity_command);
    double diff_speed_right = angularToLinear(joints_[RIGHT].velocity_command);

    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    wrapper.sendMotorCommand(diff_speed_left, diff_speed_right);
  }

  /**
  * Scale left and right speed outputs to maintain ros_control's desired trajectory without saturating the outputs
  */
  void JaguarHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right){
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > maxSpeed_){
      diff_speed_left *= maxSpeed_ / large_speed;
      diff_speed_right *= maxSpeed_ / large_speed;
    }
  }

  void JaguarHardware::updateSensors(bool imu, bool gps, bool status){
    if(imu){
      wrapper.updateIMU();
    }
    if(gps){
      wrapper.updateGPS();
    }
    if(status){
      wrapper.updateStatus();
    }
  }

  void JaguarHardware::publishSensors(bool imu, bool gps, bool status){
      if(imu){
        //TODO
      }
      if(gps){
        long unsigned int sz = wrapper.gps_data_queue_.size();
        if(sz > 0){
          gps_status = sensor_msgs::NavSatFix();
          gps_status.header.stamp = ros::Time::now();
          gps_status.header.frame_id = string("base_link");
          gps_status.status.status = wrapper.gps_data_queue_[sz-1].gpsStatus;
          gps_status.latitude = wrapper.gps_data_queue_[sz-1].latitude;
          gps_status.longitude = wrapper.gps_data_queue_[sz-1].longitude;
          gps_publisher_.publish(gps_status);
        }
      }
      if(status){
        long unsigned int sz = wrapper.status_data_queue_.size();
        if(sz > 0){
          jaguar_status_msg_ = jaguar_msgs::JaguarStatus();
          jaguar_status_msg_.header.stamp = ros::Time::now();
          jaguar_status_msg_.header.frame_id = base_frame_;
          jaguar_status_msg_.status.insert(jaguar_status_msg_.status.end(), wrapper.status_data_queue_[sz-1].status, std::end(wrapper.status_data_queue_[sz-1].status));
          jaguar_status_msg_.internal_chip_temp.insert(jaguar_status_msg_.internal_chip_temp.end(), wrapper.status_data_queue_[sz-1].temp1, std::end(wrapper.status_data_queue_[sz-1].temp1));
          jaguar_status_msg_.driver1_temp.insert(jaguar_status_msg_.driver1_temp.end(), wrapper.status_data_queue_[sz-1].temp2, std::end(wrapper.status_data_queue_[sz-1].temp2));
          jaguar_status_msg_.driver2_temp.insert(jaguar_status_msg_.driver2_temp.end(), wrapper.status_data_queue_[sz-1].temp3, std::end(wrapper.status_data_queue_[sz-1].temp3));
          jaguar_status_msg_.battery_voltage.insert(jaguar_status_msg_.battery_voltage.end(), wrapper.status_data_queue_[sz-1].volMain, std::end(wrapper.status_data_queue_[sz-1].volMain));
          jaguar_status_msg_.vol12V.insert(jaguar_status_msg_.vol12V.end(), wrapper.status_data_queue_[sz-1].vol12V, std::end(wrapper.status_data_queue_[sz-1].vol12V));
          jaguar_status_msg_.vol5V.insert(jaguar_status_msg_.vol5V.end(), wrapper.status_data_queue_[sz-1].vol5V, std::end(wrapper.status_data_queue_[sz-1].vol5V));
          jaguar_status_msg_.dinput.insert(jaguar_status_msg_.dinput.end(), wrapper.status_data_queue_[sz-1].dinput, std::end(wrapper.status_data_queue_[sz-1].dinput));
          jaguar_status_msg_.doutput.insert(jaguar_status_msg_.doutput.end(), wrapper.status_data_queue_[sz-1].doutput, std::end(wrapper.status_data_queue_[sz-1].doutput));
          jaguar_status_msg_.ack.insert(jaguar_status_msg_.ack.end(), wrapper.status_data_queue_[sz-1].ack, std::end(wrapper.status_data_queue_[sz-1].ack));
          status_publisher_.publish(jaguar_status_msg_);
        }
      }
  }

  void JaguarHardware::releaseEBrake(){
    wrapper.eBrake(false);
  }

  /**
  * RobotHW provides velocity command in rad/s, Jaguar needs m/s,
  */
  double JaguarHardware::angularToLinear(const double &angle) const{
    return angle * wheel_diameter_ / 2;
  }

    //getMotorsTravel returns radians
    double JaguarHardware::getMotorsTravel(const int motor_index){
      return 2 * pi * wrapper.motor_data_queue_[wrapper.motor_data_queue_.size()-1].motorSensorEncoderPos[motor_index] / encoderOneCircleCnt_;
    }

    double JaguarHardware::getLinearVelocity(const int motor_index){
      return maxSpeed_ * wrapper.motor_data_queue_[wrapper.motor_data_queue_.size()-1].motorSensorEncoderVel[motor_index] / 1000;
    }


}  // namespace jaguar_base
