#ifndef DRROBOT_WRAPPER_HPP
#define DRROBOT_WRAPPER_HPP
#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sstream>
#include <jaguar_base/DrRobotMotionSensorDriver.hpp>

#define MOTOR_NUM 4

using namespace std;
using namespace DrRobot_MotionSensorDriver;

namespace drrobot_wrapper{
class DrRobotWrapper{
public:

    ros::NodeHandle node_;

    vector<IMUSensorData> imu_data_queue_;
    vector<GPSSensorData> gps_data_queue_;
    vector<MotorSensorData> motor_data_queue_;
    vector<MotorBoardData> status_data_queue_;

    DrRobotWrapper(string robotID, string robotType, string robotCommMethod, string robotIP, int commPortNum, string robotSerialPort,
                    int encoderOneCircleCnt, int message_queue, bool use_IMU, bool use_GPS, bool use_motors, bool use_status, int battery_packs);

    DrRobotWrapper(){}

    ~DrRobotWrapper(){}

    int start();

    int stop();

    void sendMotorCommand(const double diff_speed_left, const double diff_speed_right);

    void eBrake(const bool enable);

    void updateIMU();

    void updateGPS();

    void updateMotors();

    void updateStatus();

    void updateAll();

private:

    DrRobotMotionSensorDriver* drrobotMotionDriver_;

    struct DrRobotMotionConfig robotConfig1_;
    struct MotorSensorData motorSensorData_;
    struct MotorBoardData motorBoardData_;
    struct IMUSensorData imuSensorData_;
    struct GPSSensorData gpsSensorData_;

    std::string robotType_;
    std::string robotID_;
    std::string robotIP_;
    std::string robotCommMethod_;
    std::string robotSerialPort_;

    double maxSpeed_;

    int commPortNum_;
    int encoderOneCircleCnt_;
    int message_queue_;
    int battery_packs_;

    bool use_IMU_;
    bool use_GPS_;
    bool use_motors_;
    bool use_status_;

};
}
#endif