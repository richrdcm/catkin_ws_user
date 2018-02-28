#include <imu/imu.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle nh;
  ros::Rate loop(100);
  imu node(nh);
  ros::spinOnce();
  node.init();
  while(ros::ok())
  {
    //node.run(1000);
    ros::spinOnce();
    //loop.sleep();
  }
  return 0;
}
