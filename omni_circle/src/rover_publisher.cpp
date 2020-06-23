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
float radius = 2, T = 30;
float thetadot = 2*pi/T;

// Suscriber/Publisher
ros::Subscriber vicon_sub;
ros::Publisher vel_pub,ref_pub, command_pub;

open_base::Movement omni_command;


double x,y, theta;
double dXc,dYc,dthetaC,wheel1,wheel2,wheel3;
float rWheel = 0.02;    // radius of wheel, m
float D = 0.0307807;         // distance from CoM to wheel, m

geometry_msgs::Pose2D reftraj;
ros::Time last_received;
std_msgs::Int16MultiArray twist;

ros::Time beginTime;
ros::Duration runPeriod = ros::Duration(T);
ros::Time loopTime = beginTime + runPeriod;

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

// calculate total vehicle velocity
double dVt(double time)
{
    return radius*thetadot*(cos(time*thetadot)*cos(time*thetadot) + sin(time*thetadot)*sin(time*thetadot));
}


void viconCallback(geometry_msgs::Pose2D rover)
{
    
    x = rover.x;
    y = rover.y;
    theta = rover.theta + pi/6;
    last_received = ros::Time::now();
    
    
    //if (theta>=pi) theta -= 2*pi + pi/6;
    if (theta<=0) theta += 2*pi;
    /*
    if ((last_received >= beginTime) & (theta<=0)) {
        theta += 2*pi;
    }
    */
    if (last_received > loopTime) theta += 2*pi;
    

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

    double time, ref_time;
    double ref_theta;
 
    double J11,J12,J13,J21,J22,J23,J31,J32,J33;
    //reftraj.header.frame_id = "slam";
    double Pxerr=0.0,Pyerr=0.0,Ptheta_err=0.0,Iyerr=0.0,Ixerr=0.0,Itheta_err=0.0,Dxerr=0.0,Dyerr=0.0,Dtheta_err=0.0,OldErrX=0.0,OldErrY=0.0,OldErrTheta=0.0;

    //Parameters defining wheel location on body
    double alpha1, alpha2, alpha3, nu;
    //alpha is angle relative to local frame, nu is offset from x,y axis
    alpha1 = 0;
    alpha2 = 2*pi / 3;
    alpha3 = 4*pi / 3;
    nu = 0;
    

//while(ros::Time::now() <= loopTime){
while(ros::ok()) {
    d = ros::Time::now() - last_received;
    time = d.toSec();

    ref_time = last_received.toSec();

    //Publish a reference trajectory to monitor on rviz
    reftraj.x = Xt(ref_time);
    reftraj.y = Yt(ref_time);

    ref_theta = ref_time*thetadot - pi/6;
    //if (ref_theta > pi) ref_theta -= 2*pi;
    //if (ref_theta > 2*pi) ref_theta -= 2*pi;
    reftraj.theta = ref_theta;


    //PID controller
    // Proportional control:
    Pxerr = (Xt(ref_time) - x);
    Pyerr = (Yt(ref_time) - y);
    Ptheta_err = (ref_theta - theta);

    // Integral control:
    Ixerr += (Xt(ref_time) - x)/freq;
    Iyerr += (Yt(ref_time) - y)/freq;
    Itheta_err += (ref_theta - theta)/freq;
    if (Ixerr>0.5) {
        Ixerr = 0.5;
    }
    else if (Ixerr<-0.5) { 
        Ixerr = -0.5; 
    }
    if (Iyerr>0.5) {
        Iyerr = 0.5;
    }
    else if (Iyerr<-0.5) {
        Iyerr = -0.5;
    }
    if (Itheta_err>0.5) {
        Itheta_err = 0.5;
    }
    else if (Itheta_err<-0.5) {
        Itheta_err = -0.5;
    }

    // Derivative control:
    Dxerr = (Xt(ref_time) - x) - OldErrX;
    Dxerr = (Yt(ref_time) - y) - OldErrY;
    Dtheta_err = (ref_theta - theta) - OldErrTheta;

    OldErrX = Xt(ref_time) - x;
    OldErrY = Yt(ref_time) - y;
    OldErrTheta = ref_theta - theta;

    // Set P, I, D values
    double Kp, Ki, Kd;
    Kp = 0.05;
    Ki = 0.006;
    Kd = 3; //2;

    //Calculate robot speed with PID feedback
    dXc = dXt(time) + Kp*Pxerr + Ki*Ixerr + Kd*Dxerr;
    dYc = dYt(time) + Kp*Pyerr + Ki*Iyerr + Kd*Dyerr;
    //dXc = 0;
    //dYc = sqrt(dXt(time)*dXt(time) + dYt(time)*dYt(time));
    //dYc = sqrt(dXc*dXc + dYc*dYc);
    //dXc = 0;
    dthetaC = thetadot + Kp*Ptheta_err + Ki*Itheta_err + Kd*Dtheta_err;
    //dthetaC = 0 + Kp*Ptheta_err + Ki*Itheta_err + Kd*Dtheta_err;

    //std::cout<<"Trying to get to "<<Xc<<" "<<Yc<<std::endl;

    //Calculate wheel speeds with vanHaendel model
    // dXc is the xdot, dYc is the ydot, dthetaC is the thetadot

    //Make the Jinv matrix
    J11 = -sin(nu);
    J12 = cos(nu);
    J13 = D;
    J21 = -sin(nu+alpha2);
    J22 = cos(nu+alpha2);
    J23 = D;
    J31 = -sin(nu+alpha3);
    J32 = cos(nu+alpha3);
    J33 = D;

    wheel1 = J11*dXc + J12*dYc + J13*dthetaC;       //v_right (v3)
    wheel2 = J21*dXc + J22*dYc + J23*dthetaC;       //v_left (v1)
    wheel3 = J31*dXc + J32*dYc + J33*dthetaC;       //v_back (v2)

    if (wheel1 > 0.6) wheel1 = 0.6;
    if (wheel1 < -0.6) wheel1 = -0.6;
    if (wheel2 > 0.6) wheel2 = 0.6;
    if (wheel2 < -0.6) wheel2 = -0.6;
    if (wheel3 > 0.6) wheel3 = 0.6;
    if (wheel3 < -0.6) wheel3 = -0.6;

    //Saturate wheel speeds and push into topic type
    /*
    if (wheel1 < 1) wheel1 = 1;
    if (wheel1 > 127) wheel1 = 127;

    if (wheel2 < 128) wheel2 = 128;
    if (wheel2 > 255) wheel2 = 255;

    if (wheel3 < 1) wheel3 = 1;
    if (wheel3 > 127) wheel3 = 127;
    */

    /*
    twist.data.clear();
    twist.data.push_back(wheel1);
    twist.data.push_back(wheel2);
    twist.data.push_back(wheel3);
    twist.data.push_back(0);
    */

    omni_command.movement = 3;
    omni_command.wheel.v_left = wheel2;     // v1
    omni_command.wheel.v_back = wheel3;     // v2
    omni_command.wheel.v_right = wheel1;    // v3

    /*
    if (ros::Time::now() >= endTime)
    {
        omni_command.movement = 3;

        omni_command.wheel.v_left = 0;
        omni_command.wheel.v_back = 0;
        omni_command.wheel.v_right = 0;
    }
    */

    //Publish necessary topics
    //vel_pub.publish(twist);
    command_pub.publish(omni_command);
    ref_pub.publish(reftraj);
    ros::spinOnce();
    loop_rate.sleep();

    
 }



/*
while (ros::ok())
{



    omni_command.movement = 3;
    omni_command.wheel.v_left = -0.09827304;
    omni_command.wheel.v_back = 0.21588622;
    omni_command.wheel.v_right = -0.09827304;

    command_pub.publish(omni_command);

    ros::spinOnce();
    loop_rate.sleep();
}
*/
}