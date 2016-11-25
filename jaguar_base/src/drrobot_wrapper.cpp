#include <jaguar_base/drrobot_wrapper.hpp>
#include <chrono>
#include <thread>

using namespace std;
using namespace DrRobot_MotionSensorDriver;

namespace drrobot_wrapper{
    DrRobotWrapper::DrRobotWrapper(string robotID, string robotType, string robotCommMethod, string robotIP, int commPortNum, string robotSerialPort,
                    int encoderOneCircleCnt, int message_queue, bool use_IMU, bool use_GPS, bool use_motors, bool use_status, int battery_packs){

        robotID_ = robotID;
        robotType_ = robotType;
        robotCommMethod_ = robotCommMethod;
        robotIP_ = robotIP;
        commPortNum_ = commPortNum;
        robotSerialPort_ = robotSerialPort;
        encoderOneCircleCnt_ = encoderOneCircleCnt;
        message_queue_ = message_queue;
        use_IMU_ = use_IMU;
        use_GPS_ = use_GPS;
        use_motors_ = use_motors;
        use_status_ = use_status;
        battery_packs_ = battery_packs;

        if (robotCommMethod_ == "Network"){
            robotConfig1_.commMethod = Network;
        }
        else{
            robotConfig1_.commMethod = Serial;
        }

        if (robotType_ == "Jaguar"){
            robotConfig1_.robotType = Jaguar;
        }
        robotConfig1_.portNum = commPortNum_;

        strcpy(robotConfig1_.robotIP,robotIP_.c_str());

        strcpy(robotConfig1_.serialPortName,robotSerialPort_.c_str());

        drrobotMotionDriver_ = new DrRobotMotionSensorDriver();
        if(robotType_ == "Jaguar"){
            drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
        }
        start();
    }

    int DrRobotWrapper::start(){
        int res = -1;
        if(robotType_ == "Jaguar"){
            res = drrobotMotionDriver_->openNetwork(robotConfig1_.robotIP,robotConfig1_.portNum);
            if (res == 0){
                ROS_INFO("Opened port number at: [%d]", robotConfig1_.portNum);
            }
            else{
                ROS_ERROR("Could not open network connection to [%s,%d]",  robotConfig1_.robotIP,robotConfig1_.portNum);
            }
        }
        return(0);
    }

    int DrRobotWrapper::stop(){
        int status = 0;
        drrobotMotionDriver_->close();

        usleep(1000000);
        return(status);
    }

    void DrRobotWrapper::sendMotorCommand(const double diff_speed_left, const double diff_speed_right){
        std::string command = "MMW !M ";
        command += to_string(static_cast<int>((diff_speed_left)*1000)) + " " + to_string(static_cast<int>(-(diff_speed_right)*1000));
        //ROS_WARN("%s",command.c_str());
        drrobotMotionDriver_->sendCommand(command.c_str(), strlen(command.c_str()));
    }

    void DrRobotWrapper::eBrake(const bool enable){
        updateStatus();
        while(enable && status_data_queue_[status_data_queue_.size()-1].status[0] != 16){
            drrobotMotionDriver_ -> sendCommand("MMW !EX", 7);
            ROS_INFO("Trying to enable e-brake");
            updateStatus();
        }
        while(!enable && status_data_queue_[status_data_queue_.size()-1].status[0] != 0){
            drrobotMotionDriver_->sendCommand("MMW !MG", 7);
            ROS_INFO("Trying to disable e-brake");
            sleep(1);
            updateStatus();
        }
    }

    void DrRobotWrapper::updateIMU(){
        if(use_IMU_){
            drrobotMotionDriver_->readIMUSensorData(&imuSensorData_);
            imu_data_queue_.push_back(imuSensorData_);
            if(imu_data_queue_.size() > message_queue_){
                imu_data_queue_.erase(imu_data_queue_.begin());
            }
            drrobotMotionDriver_->sendCommand("PING",4);
        }
    }

    void DrRobotWrapper::updateGPS(){
        if(use_GPS_){
            drrobotMotionDriver_->readGPSSensorData(&gpsSensorData_);
            gps_data_queue_.push_back(gpsSensorData_);
            if(gps_data_queue_.size() > message_queue_){
                gps_data_queue_.erase(gps_data_queue_.begin());
            }
            drrobotMotionDriver_->sendCommand("PING",4);
        }
    }

    void DrRobotWrapper::updateStatus(){
        if(use_status_){
            drrobotMotionDriver_->readMotorBoardData(&motorBoardData_);
            status_data_queue_.push_back(motorBoardData_);
            if(status_data_queue_.size() > message_queue_){
                status_data_queue_.erase(status_data_queue_.begin());
            }
            drrobotMotionDriver_->sendCommand("PING",4);
        }
    }

    void DrRobotWrapper::updateMotors(){
        if(use_motors_){
            drrobotMotionDriver_->readMotorSensorData(&motorSensorData_);
            motor_data_queue_.push_back(motorSensorData_);
            if(motor_data_queue_.size() > message_queue_){
                motor_data_queue_.erase(motor_data_queue_.begin());
            }
            drrobotMotionDriver_->sendCommand("PING",4);
        }
    }

    void DrRobotWrapper::updateAll(){
        updateIMU();
        updateGPS();
        updateMotors();
        updateStatus();
    }
}
