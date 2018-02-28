#include <imu/imu.h>
#include <thread>
#include <atomic>

imu::imu(ros::NodeHandle nh): nh_(nh), priv_nh_("~"), my_serial("/dev/ttyUSB3",115200, serial::Timeout::simpleTimeout(1000))
{
  result="";
  pub_imu = nh_.advertise<sensor_msgs::Imu>(nh_.resolveName("/imu"), 1);
  priv_nh_.param<std::string>("arduino_serial_port", serial_port_, "/dev/ttyUSB3");
  priv_nh_.param("arduino_baud_rate", baud_rate_,115200);
  my_serial.close();
  my_serial.setPort(serial_port_);
  my_serial.setBaudrate(baud_rate_);
  my_serial.open();
  //my_serial.setTimeout(1000);
  //my_serial.Timeout.simpleTimeout(1000)
  //  init();

  mThread = std::thread([this]() { while(mIsRunning) {
    my_sleep(1);
    processValues();
  }});

}

    //! Empty stub
imu::~imu() {
  mIsRunning = false;
  mThread.join();
}

void imu::processValues() {

  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = "imu";
  imu_msg.orientation.w = 0.0;
  //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  auto result = readline();
  if(result == "ypr")
  {
    auto value = readline();
    std::stringstream ss(value);
    ss >> imu_msg.orientation.z;
    //imu_msg.orientation.z = std::stod(value);
    //ROS_INFO("yaw\n");
  } else if(result == "roll")
  {
      auto value = readline();
      std::stringstream ss(value);
      ss >> imu_msg.orientation.y;
      //imu_msg.orientation.y = std::stod(value);
      //ROS_INFO("roll\n");
  } else if(result == "pitch")
  {
      auto value = readline();
      std::stringstream ss(value);
      ss >> imu_msg.orientation.x;
      //imu_msg.orientation.x = std::stod(value);
      //ROS_INFO("pitch\n");

  } else if(result == "arx")
  {
      auto value = readline();
      std::stringstream ss(value);
      ss >> imu_msg.orientation.x;
      //imu_msg.linear_acceleration.x = std::stof(value);
      //ROS_INFO("arx\n");

  } else if(result == "ary")
  {
      auto value = readline();
      std::stringstream ss(value);
      ss >> imu_msg.linear_acceleration.y;
      //imu_msg.linear_acceleration.y =  std::stof(value);
      //ROS_INFO("ary\n");

  } else if(result == "arz")
  {
      auto value = readline();
      std::stringstream ss(value);
      ss >> imu_msg.linear_acceleration.z;
      //imu_msg.linear_acceleration.z = std::stof(value);
      //ROS_INFO("arz\n");
  } else if(result == "gx")
  {
      auto value = readline();
      std::stringstream ss(value);
      ss >> imu_msg.angular_velocity.x;
      //imu_msg.angular_velocity.x  = std::stof(value);
      //ROS_INFO("gx: \n");
  } else if(result == "gy")
  {
      auto value = readline();
      std::stringstream ss(value);
      ss >> imu_msg.angular_velocity.y;
      //imu_msg.angular_velocity.y  = std::stof(value);
      //ROS_INFO("gy: \n");
  } else if(result == "gz")
  {
      auto value = readline();
      std::stringstream ss(value);
      ss >> imu_msg.angular_velocity.z;
      //imu_msg.angular_velocity.z  = std::stof(value);
      //ROS_INFO("gz: \n");
  }

  pub_imu.publish(imu_msg);


}

void imu::my_sleep(unsigned long milliseconds) {
    usleep(milliseconds*1000); // 100 ms
}

void imu::init()
{
  try
    {
      ROS_INFO("imu::Is the serial port %s open?",serial_port_.c_str());
      //cout << my_serial.getBaudrate() << endl;
	  if(my_serial.isOpen())
	    ROS_INFO("imu::Yes.");
	  else
	    ROS_ERROR("imu::No.");
    }
    catch(const std::exception& e)
    {
      	ROS_ERROR("imu::could not find serial port");
    }
}

std::string imu::readline()
{
  std::string result = "";
  uint8_t c = '\0';
  // read until new line terminates result string
  while (c  != '\n')
  {
      my_serial.read(&c, 1);
      if(c != '\n' and c != '\r')
      {
      result += char(c);
      }
  }
    return result;
}
