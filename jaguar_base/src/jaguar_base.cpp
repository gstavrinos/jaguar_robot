#include "jaguar_hardware.hpp"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

/**
* Control loop for Jaguar, not realtime safe
*/
void controlLoop(jaguar_base::JaguarHardware &jaguar,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{

    // Calculate monotonic time difference
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    // Process control loop
    jaguar.updateJointsFromHardware();
    cm.update(ros::Time::now(), elapsed);
    jaguar.writeCommandsToHardware();
}

/**
* Status loop for Jaguar
*/
void statusLoop(jaguar_base::JaguarHardware &jaguar)
{
    //Update and publish Status
    jaguar.updateSensors(false, true, true);
    jaguar.publishSensors(false, true, true);
}

int main(int argc, char *argv[])
{ 
    ros::init(argc, argv, "jaguar_base");
    ros::NodeHandle nh, private_nh("~");

    double control_frequency, status_frequency;
    private_nh.param<double>("control_frequency", control_frequency, 10.0);
    private_nh.param<double>("status_frequency", status_frequency, 1.0);

    ROS_WARN("We are still testing stuff!");

    // Initialize robot hardware and link to controller manager
    jaguar_base::JaguarHardware jaguar(nh, private_nh);
    controller_manager::ControllerManager cm(&jaguar, nh);

    // Setup separate queue and single-threaded spinner to process timer callbacks
    // that interface with Jaguar hardware
    ros::CallbackQueue jaguar_queue;
    ros::AsyncSpinner jaguar_spinner(1, &jaguar_queue);

    time_source::time_point last_time = time_source::now();
    ros::TimerOptions control_timer(
    ros::Duration(1 / control_frequency),
    boost::bind(controlLoop, boost::ref(jaguar), boost::ref(cm), boost::ref(last_time)),
    &jaguar_queue);
    ros::Timer control_loop = nh.createTimer(control_timer);

    ros::TimerOptions status_timer(
    ros::Duration(1 / status_frequency),
    boost::bind(statusLoop, boost::ref(jaguar)),
    &jaguar_queue);
    ros::Timer status_timer_ = nh.createTimer(status_timer);

    jaguar_spinner.start();

    // Process remainder of ROS callbacks separately, mainly ControlManager related
    ros::spin();

    return 0;
}
