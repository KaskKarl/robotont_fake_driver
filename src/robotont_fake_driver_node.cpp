#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

double VX = 0.0;
double VY = 0.0;
double VTH = 0.0;

double X = 0.0;
double Y = 0.0;
double TH = 0.0;

ros::Time RECEIVE_TIME;

void receiveCmd(geometry_msgs::Twist  /*input_velocity*/)
{
  receive_time = ros::Time::now();
  vx = input_velocity.linear.x;
  vy = input_velocity.linear.y;
  vth = input_velocity.angular.z;
}

// service callback function
bool servCB(std_srvs::Empty::Request&  /*request*/, std_srvs::Empty::Response&  /*response*/)
{
  X = 0.0;
  Y = 0.0;
  TH = 0.0;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotont_fake_driver_node");
  ros::NodeHandle n;
  ros::Subscriber vel_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, receiveCmd);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  std::string odom_frame;
  std::string base_frame;
  n.param<std::string>("robotont_fake_driver_node/odom_frame", odom_frame, "odom");
  n.param<std::string>("robotont_fake_driver_node/base_frame", base_frame, "base_footprint");

  // service
  ros::ServiceServer service = n.advertiseService("odom_reset", servCB);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate r(100);
  while (n.ok())
  {
    ros::spinOnce();  // Check for incoming messages
    current_time = ros::Time::now();
    if (current_time - receive_time > ros::Duration(0.5))
    {
      vx = 0.0;
      vy = 0.0;
      vth = 0.0;
    }
    // Compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    // Since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    // first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame;
    odom_trans.child_frame_id = base_frame;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // Send the transform
    odom_broadcaster.sendTransform(odom_trans);
    r.sleep();

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame;

    // Set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // Set the velocity
    odom.child_frame_id = base_frame;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    // DEBUG "nav_msgs::Odometry odom"
    ROS_DEBUG("Publishing odometry: px [%f]; py [%f] | qx [%f]; qy [%f]; qz [%f]; qw [%f] | to [%s] \n",
              odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.orientation.x,
              odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w,
              odom.header.frame_id.c_str());

    // Publish the message
    odom_pub.publish(odom);
    last_time = current_time;
  }
}
