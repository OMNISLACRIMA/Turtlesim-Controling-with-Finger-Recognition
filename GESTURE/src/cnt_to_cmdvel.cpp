#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <turtlesim/Pose.h>
#include "GESTURERECOGNITIONSMY/GR.h"
#include <std_srvs/Empty.h>
#define PI 3.141592
ros::Subscriber sub_cnt;
ros::Publisher pub_vel;
geometry_msgs::Twist cmd_vel;
turtlesim::PoseConstPtr cur_pose;
turtlesim::Pose g_pose;
std_srvs::Empty empty;
ros::ServiceClient reset;

float w=0.5;//触壁反弹角速度大小
enum Edge
{no,top,bottom,left,right,};
enum State
{standard,edges,};
State state=standard;
Edge edge=no;

int isedge(const turtlesim::PoseConstPtr& cur_pose);
void setedgegoal(const turtlesim::PoseConstPtr& cur_pose);
void Angulartwist(ros::Publisher pub_vel);
void setangularvs();
void messageCallback(const GESTURERECOGNITIONSMY::GR::ConstPtr& msg)
{
  if (state==standard)
  {
    if(isedge(cur_pose)>0)
    {
      setedgegoal(cur_pose);
      setangularvs();
    }
    switch(msg->A)
    {
      case 5:   cmd_vel.linear.x =1;cmd_vel.angular.x =0;
                cmd_vel.linear.y =0;cmd_vel.angular.y =0;
                cmd_vel.linear.z =0;cmd_vel.angular.z =0;
                break;

      case 4:   cmd_vel.linear.x =0;cmd_vel.angular.x =0;
                cmd_vel.linear.y =0;cmd_vel.angular.y =0;
                cmd_vel.linear.z =0;cmd_vel.angular.z =1;
                break;

      case 3:   cmd_vel.linear.x =2;cmd_vel.angular.x =0;
                cmd_vel.linear.y =0;cmd_vel.angular.y =0;
                cmd_vel.linear.z =0;cmd_vel.angular.z =-0.5;
                break;

      case 2:   reset.call(empty);
                cmd_vel.linear.x =0;cmd_vel.angular.x =0;
                cmd_vel.linear.y =0;cmd_vel.angular.y =0;
                cmd_vel.linear.z =0;cmd_vel.angular.z =0;
                break;

      case 1:   cmd_vel.linear.x =rand()%11-5;cmd_vel.angular.x =0;
                cmd_vel.linear.y =0;cmd_vel.angular.y =0;
                cmd_vel.linear.z =0;cmd_vel.angular.z =rand()%11-5;
                break;

      default:  cmd_vel.linear.x =0;cmd_vel.angular.x =0;
                cmd_vel.linear.y =0;cmd_vel.angular.y =0;
                cmd_vel.linear.z =0;cmd_vel.angular.z =0;
                break;
    }
    pub_vel.publish(cmd_vel);
  }
  else if(state==edges)
  {
    Angulartwist(pub_vel);
  }
}

void poseMessageReceived(const turtlesim::PoseConstPtr& pose)
{
  cur_pose=pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cnt_to_vel");
  ros::NodeHandle n;
  sub_cnt = n.subscribe("/cnttt", 1000, messageCallback);
  pub_vel = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  ros::Subscriber pose = n.subscribe("turtle1/pose",1000,&poseMessageReceived); 
  reset = n.serviceClient<std_srvs::Empty>("reset");
  while(ros::ok)
  {ros::spinOnce();}
  return 0;
}

int isedge(const turtlesim::PoseConstPtr& cur_pose)
{
  if(cur_pose->x>=11.088889){edge=right;state=edges;}
  else if(cur_pose->y>=11.088889){edge=top;state=edges;}
  else if(cur_pose->x==0){edge=left;state=edges;}
  else if(cur_pose->y==0){edge=bottom;state=edges;}
  else{edge=no;}
  return edge;
}

void setedgegoal(const turtlesim::PoseConstPtr& cur_pose)
{
  if(edge==right)
  {
    if(cur_pose->theta>=0){g_pose.theta=PI-cur_pose->theta;}
    else{g_pose.theta=-PI-cur_pose->theta;}
  }
  if(edge==top)
  {
    if(cur_pose->theta!=PI){g_pose.theta=-cur_pose->theta;}
    else{g_pose.theta=PI;}
  }
  if(edge==bottom)
  {
    if(cur_pose->theta!=PI){g_pose.theta=-cur_pose->theta;}
    else{g_pose.theta=PI;}
  }  
  if(edge==left)
  {
    if(cur_pose->theta>=0){g_pose.theta=PI-cur_pose->theta;}
    else{g_pose.theta=-PI-cur_pose->theta;}
  }
}

void Angulartwist(ros::Publisher pub_vel)
{
  if(abs(g_pose.theta-cur_pose->theta)>0.01)
  {
  ROS_INFO("Now pose.theta is %f",cur_pose->theta);
  cmd_vel.linear.x =0;cmd_vel.angular.x =0;
  cmd_vel.linear.y =0;cmd_vel.angular.y =0;
  cmd_vel.linear.z =0;cmd_vel.angular.z =w;
  pub_vel.publish(cmd_vel);
  }
  else
  {
  ROS_INFO("Have reached the angular goal");
  cmd_vel.linear.x =1;cmd_vel.angular.x =0;
  cmd_vel.linear.y =0;cmd_vel.angular.y =0;
  cmd_vel.linear.z =0;cmd_vel.angular.z =0;
  pub_vel.publish(cmd_vel);
  ros::Duration(0.1).sleep();
  state=standard;
  edge=no;
  }
}

void setangularvs()
{
  if(cur_pose->theta>0)
  {
    if(g_pose.theta>cur_pose->theta||g_pose.theta<cur_pose->theta-PI){w=abs(w);}
    else{w=-abs(w);}
  }
  else
  {
    if(g_pose.theta>cur_pose->theta&&g_pose.theta<cur_pose->theta+PI){w=abs(w);}
    else{w=-abs(w);}
  }
}