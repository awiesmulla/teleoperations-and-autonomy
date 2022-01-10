#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/Odometry.h"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Function prototype
int send_nav_goal(float x, float y);
void odomcallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    float x_pos = msg->pose.pose.position.x;
    float y_pos = msg->pose.pose.position.y;
    // send_nav_goal(x_pos + 1.0,y_pos + 0.0);
}

//Sends the Navigation goal
int send_nav_goal(float x, float y)
{
    move_base_msgs::MoveBaseGoal goal;
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    goal.target_pose.header.frame_id = "map";//////////////////////////////////
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");

}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n("~");
  ros::Subscriber sub = n.subscribe("/odometry/filtered_map",100,odomcallback);////////topic and time///////
  send_nav_goal(2.0,0.0);
  return 0;
}
