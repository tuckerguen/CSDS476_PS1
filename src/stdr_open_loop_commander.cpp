#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


ros::Publisher twist_commander;
double sample_dt = 0.01; //specify a sample period of 10ms

void run_command(double lin_x, double ang_z, double duration, ros::Rate loop_timer){
	double timer=0.0;
	geometry_msgs::Twist twist_cmd;
	twist_cmd.linear.x = lin_x;
	twist_cmd.angular.z = ang_z;
	while(timer<duration) {
		twist_commander.publish(twist_cmd);
		timer+=sample_dt;
		loop_timer.sleep();
	}
}

void warmup_comms(ros::Rate loop_timer){
	geometry_msgs::Twist twist_cmd;
	//start sending some zero-velocity commands, just to warm up communications with STDR
	for (int i=0;i<10;i++) {
		twist_commander.publish(twist_cmd);
		loop_timer.sleep();
	}
}

void stop_motion(ros::Rate loop_timer){
	//halt the motion
	geometry_msgs::Twist twist_cmd;
	twist_cmd.angular.z=0.0;
	twist_cmd.linear.x=0.0;
	for (int i=0;i<10;i++) {
		twist_commander.publish(twist_cmd);
		loop_timer.sleep();
	}
}

//node to send Twist commands to the Simple 2-Dimensional Robot Simulator via cmd_vel
int main(int argc, char **argv) {
	ros::init(argc, argv, "stdr_commander");

	//some "magic numbers"
	double speed = 1.0; // 1m/s speed command
	double yaw_rate = 0.52; //0.5 rad/sec yaw rate command
	double time_3_sec = 3.0; // should move 3 meters or 1.5 rad in 3 seconds

	ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS

	twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
	geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR

	ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate

	// start with all zeros in the command message; should be the case by default, but just to be safe..
	twist_cmd.linear.x=0.0;
	twist_cmd.linear.y=0.0;
	twist_cmd.linear.z=0.0;
	twist_cmd.angular.x=0.0;
	twist_cmd.angular.y=0.0;
	twist_cmd.angular.z=0.0;

	warmup_comms(loop_timer);

	run_command(speed, 0, time_3_sec,loop_timer);
	run_command(0, yaw_rate, time_3_sec,loop_timer);
	run_command(speed, 0, time_3_sec,loop_timer);
	run_command(0, -yaw_rate, time_3_sec,loop_timer);

	run_command(speed, 0, 4.0,loop_timer);
	run_command(0, yaw_rate, time_3_sec,loop_timer);
	run_command(speed, 0, 2,loop_timer);
	run_command(0, yaw_rate, time_3_sec,loop_timer);
	run_command(speed, 0, 5.5,loop_timer);

	run_command(0, -yaw_rate, 2, loop_timer);
	run_command(speed, 0, 2.8,loop_timer);
	run_command(0, -yaw_rate, 0.95, loop_timer);
	run_command(speed, 0, 5, loop_timer);

	stop_motion(loop_timer);
	//done commanding the robot; node runs to completion
}

