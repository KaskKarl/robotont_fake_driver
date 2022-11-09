#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <gtest/gtest.h>


double lx = 0.2;
double px = 0.0;
bool state = true;

ros::Time start_time, current_time;

TEST(TestSuits, odom_reset_test){
	EXPECT_EQ(px, 0.0);

}

void callBack(nav_msgs::Odometry data){
	px = data.pose.pose.position.x;

}



int main(int argc, char** argv) {

	ros::init(argc, argv, "odom_reset");
	ros::NodeHandle nh;
	
	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
	ros::Subscriber sub=nh.subscribe<nav_msgs::Odometry>("/odom", 1000, callBack);
	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("odom_reset");
	
	geometry_msgs::Twist vel_msg;
	std_srvs::Empty srv;
	
	ros::Rate r(100);

	
	start_time, current_time = ros::Time::now();
	while (ros::ok() && current_time - start_time < ros::Duration(3)){
		
		vel_msg.linear.x = lx;
		
		pub.publish(vel_msg);
		
		current_time = ros::Time::now();
		ros::spinOnce();
		
		r.sleep();

	}
	
	while (ros::ok()){
		
		
		if (px == 0.0){
			break;
		}
		else
		{
			if (client.call(srv))
    			{
      				ROS_INFO("Sent reset message");
    			}
    			else
     			{
      				ROS_ERROR("Failed to call service odom_reset");
      				return 1;
     			}
		}
		ros::spinOnce();
		r.sleep();
	
	}
	
	
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
