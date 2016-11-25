#ifndef DATA_STRUCTS_H_
#define DATA_STRUCTS_H_

namespace DrRobot_MotionSensorDriver {

  struct DrRobotMotionConfig {
    char robotID[CHAR_BUF_LEN];        //!< robotID, you could specify your own name for your robot
    char robotIP[CHAR_BUF_LEN];        //!< robot main WiFi module IP address, you could get it by manual
    int portNum;                       //!< robot main WiFi module port number, default is power system on 10001 port, motion system on 10002
    CommMethod commMethod;             //!< communication method enum CommMethod
    char serialPortName[CHAR_BUF_LEN]; //!< serial port name if you use serial communication
    RobotType robotType;               //!< specify the control system on the robot, enum BoardType
  };

  struct MotorSensorData {
    int motorSensorEncoderPos[MOTORSENSOR_NUM];         //!< encoder count reading
    int motorSensorEncoderVel[MOTORSENSOR_NUM];         //!< encoder velocity reading
    int motorSensorCurrent[MOTORSENSOR_NUM];            //!< motor current AD value reading,
    int motorSensorTemperature[MOTORSENSOR_NUM];        //!< motor temperature sensor reading
    int motorSensorPWM[MOTORSENSOR_NUM];                //!< motor driver board output PWM value,
    int motorSensorEncoderPosDiff[MOTORSENSOR_NUM];     //!< encoder count reading difference related with last reading
  };

  struct MotorBoardData {
    int status[battery_packs];                         //!< motor board status, read back from query "FF"
    int temp1[battery_packs];                           //!< motor board internal temperature 1
    int temp2[battery_packs];                           //!< motor board internal temperature 2
    int temp3[battery_packs];                           //!< motor board internal temperature 3
    double volMain[battery_packs];                     //!< motor board main power voltage, default is battery voltage
    double vol12V[battery_packs];                      //!< motor board 12V power voltage
    double vol5V[battery_packs];                       //!< motor board 5V power voltage
    int dinput[battery_packs];                             //!< digital input, not used now
    int doutput[battery_packs];                             //!< digital output, not used now
    int ack[battery_packs];                            //!< 0- right command received("+"), -1 wrong command("-")
  };

  struct GPSSensorData {
    long timeStamp;             //!< GPS Message time stamp, format: hhmmss
    long dateStamp;             //!< GPS date stamp, format:ddmmyy
    int gpsStatus;              //!< GPS status, 0-fixed,1-differential, -1 -- invalid
    double latitude;            //!< GPS latitude, - ==south, + == north
    double longitude;           //!< GPS longitude, - ==west, + == east
    double vog;                 //!< GPS velocity over ground   m/s
    double cog;                 //!< GPS course over ground,,radian
  };

  struct IMUSensorData {
    int seq;                    //!< IMU sensor package sequence number, 0~ 255
    double yaw;                 //!< yaw estimate from robot, unit:radian
    double pitch;               //!< pitch estimate from robot, unit:radian, not used now
    double roll;                //!< roll estimate from robot, unit:radian, not used now

    int gyro_x;                 //!< raw gyro x axis data
    int gyro_y;                 //!< raw gyro y axis data
    int gyro_z;                 //!< raw gyro z axis data

    int accel_x;                 //!< raw accel x axis data
    int accel_y;                 //!< raw accel y axis data
    int accel_z;                 //!< raw accel z axis data

    int comp_x;                 //!< raw magnetic sensor x axis data
    int comp_y;                 //!< raw magnetic sensor y axis data
    int comp_z;                 //!< raw magnetic sensor z axis data
  };
}
#endif