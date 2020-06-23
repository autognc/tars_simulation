#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <tf/tf.h>
#include <math.h>
#include <std_msgs/Int16MultiArray.h>

#include "open_base/Movement.h"

#define pi 3.14159265

//Aspects of desired circular path
float radius = 1, T = 10;
float thetadot = 2*pi/T;

// Suscriber/Publisher
ros::Subscriber vicon_sub;
ros::Publisher vel_pub,ref_pub, command_pub;

open_base::Movement omni_command;


double x,y;
double Xc,Yc,thetaC,wheel1,wheel2,wheel3;
float rWheel = 0.02;    // radius of wheel, m
float D = 0.04;         // distance from CoM to wheel, m

geometry_msgs::Pose2D reftraj;
ros::Time last_received;
std_msgs::Int16MultiArray twist;

double Xt(double time)
{
    return radius*cos(time*thetadot);
}

double Yt(double time)
{
    return radius*sin(time*thetadot);
}

double dXt(double time)
{
    return -radius*thetadot*sin(time*thetadot);
}

double dYt(double time)
{
    return radius*thetadot*cos(time*thetadot);
}


void viconCallback(geometry_msgs::Pose2D rover)
{
    x = rover.x;
    y = rover.y;
    last_received = ros::Time::now();
    /*
    x = rover.pose.position.x - 1;
    y = rover.pose.position.y;
    last_received = ros::Time::now();
    tf::Quaternion q(rover.pose.orientation.x,rover.pose.orientation.y,rover.pose.orientation.z,rover.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    yaw = yaw - pi/2;
    if (yaw>pi) yaw -= 2*pi;
    if (yaw<-pi) yaw += 2*pi;
    */
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_turtle");
    ros::NodeHandle nh;
    
    int freq = 10;
    ros::Rate loop_rate(freq);
    last_received = ros::Time::now();
    ros::Duration d;
    
    
    vel_pub = nh.advertise<std_msgs::Int16MultiArray>("/cmd",10);
    ref_pub = nh.advertise<geometry_msgs::Pose2D>("/ref_traj",10);

    vicon_sub = nh.subscribe("/open_base/pose/world", 100, viconCallback);

    command_pub = nh.advertise<open_base::Movement>("/open_base/command", 10);

    double time;


 while(ros::ok()){
    d = ros::Time::now() - last_received;
    time = last_received.toSec();


    //Publish a reference trajectory to monitor on rviz
    reftraj.x = Xt(time);
    reftraj.y = Yt(time);
    reftraj.theta = time*thetadot;
    //reftraj.pose.position.x = Xt(time);
    //reftraj.pose.position.y = Yt(time);
    //reftraj.pose.position.z = 0;
    //reftraj.pose.orientation.w = cos(time*thetadot/2.0 - pi/4);
    //reftraj.pose.orientation.z = sin(time*thetadot/2.0 - pi/4);


    omni_command.movement = 1;
    omni_command.generic.frame = 3;
    omni_command.generic.target.x = reftraj.x;
    omni_command.generic.target.y = reftraj.y;
    omni_command.generic.target.theta = reftraj.theta;

    //Publish necessary topics
    //vel_pub.publish(twist);
    command_pub.publish(omni_command);
    ref_pub.publish(reftraj);
    ros::spinOnce();
    loop_rate.sleep();
    
 }
 

}