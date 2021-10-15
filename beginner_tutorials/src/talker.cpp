#include "ros/ros.h"
#include "std_msgs/String.h"
#include"beginner_tutorials/Num.h"
#include<string.h>
#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<beginner_tutorials::Num>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  std::string name("Allison");
  int age = 12;
  int sex = 1;

  while (ros::ok())
  {
    
    beginner_tutorials::Num msg;
    msg.header.seq = count;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "Num";
    msg.sex = sex;
    msg.age = age;
    msg.name = name;


    ROS_INFO("Publish/\n");
    ROS_INFO("Header/\n");
    ROS_INFO("	seq [%d]\n", msg.header.seq);
    ROS_INFO("	frame_id [%s]\n", msg.header.frame_id.c_str() );
    ROS_INFO("	name [%s]\n", msg.name.c_str() );
    ROS_INFO("	sex [%d]\n", msg.sex);
    ROS_INFO("	age [%d]\n", msg.age);


    chatter_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    count=count+1;
  }


  return 0;
}
