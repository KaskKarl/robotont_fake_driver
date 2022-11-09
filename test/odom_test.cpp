#include<gtest/gtest.h>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>

double px = 0.0;
double lx = 0.2;
int counter = 0;

ros::Time start_time, current_time;

TEST(TestSuits, odom_test){
	EXPECT_NEAR(lx * (current_time.toSec() - start_time.toSec()), px, 0.1);
}

void callBack(nav_msgs::Odometry data){
	px = data.pose.pose.position.x;
}


int main(int argc, char** argv) {
	
	
	ros::init(argc, argv, "odom_test_node");
	ros::NodeHandle nh;
	
	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	geometry_msgs::Twist vel_msg;
	
	ros::Subscriber sub=nh.subscribe<nav_msgs::Odometry>("/odom", 1000, callBack);

	ros::Rate rate(100);
	sleep(2);
	
	 
	current_time, start_time = ros::Time::now();
	while (ros::ok() &&  current_time - start_time < ros::Duration(5)){
		vel_msg.linear.x = lx;
		pub.publish(vel_msg);
		
		current_time = ros::Time::now();
		rate.sleep();
		ros::spinOnce();
		counter++;
	}
	ros::spinOnce();
	
	
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();


}
