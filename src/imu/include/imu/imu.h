#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"
#include <sstream>
#include <ros/console.h>
#include <sstream>
#include <thread>
#include <atomic>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
typedef int16_t speed_MMpS_t;


class imu
{
  private:
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    ros::NodeHandle nh_;
    ros::Publisher pub_imu;
    std::string serial_port_;//="/dev/ttySAC2";
    int baud_rate_;//=115200;
    std::string result;
    size_t bytes_wrote;

    serial::Serial my_serial;

    //serial::Serial my_serial;
    //my_serial(serial_port_, 115200, serial::Timeout::simpleTimeout(1000));
    std::string readline();
    void processValues();
    std::thread mThread;

    std::atomic<bool> mIsRunning {true};
    // std::atomic<float> mYaw;
    // std::atomic<float> mRoll;
    // std::atomic<float> mPitch;
    // std::atomic<float> mArx;
    // std::atomic<float> mAry;
    // std::atomic<float> mArz;

    sensor_msgs::Imu imu_msg = sensor_msgs::Imu();

  public:


    imu(ros::NodeHandle nh);
    ~imu();
    void init();
    void my_sleep(unsigned long milliseconds);
};
