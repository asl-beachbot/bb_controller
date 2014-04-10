#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv)
{
ros::init(argc, argv, "publisherforcontrollertestFitti");
ros::NodeHandle n;

ros::Publisher holy_pub = n.advertise<geometry_msgs::PoseStamped>("/localization/bot_pose",1000);

ros::Rate loop_rate(25);
int count = 0;

while (ros::ok())
{
geometry_msgs::PoseStamped bot_pose_instance;


bot_pose_instance.pose.position.x = 2;
bot_pose_instance.pose.position.y = 1;
bot_pose_instance.pose.position.z = -1.002;

holy_pub.publish(bot_pose_instance);
ros::spinOnce();

loop_rate.sleep();
count += 1;

}
return 0;
}
