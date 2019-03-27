

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "dynamixel_workbench_msgs/JointCommand.h"

#define MAX_TURN 360
#define MAX_F 700
#define MIN_F 290
#define TURN_INDEX 3
#define F_INDEX 2
#define JOINT_ID 1

ros::Publisher pub_turn;
ros::ServiceClient client;
ros::Publisher pub_f;
std_msgs::Float32 angle_cmd;
sensor_msgs::JointState f_cmd_srv;


void Callback(const sensor_msgs::Joy cmd) {
    int f_gap = MAX_F - MIN_F;
    float turn_rate;
    float f_rate;
    
    turn_rate = cmd.axes[TURN_INDEX];
    angle_cmd.data = -turn_rate * MAX_TURN;
    pub_turn.publish(angle_cmd);

    
    f_rate = cmd.axes[F_INDEX];
    ROS_INFO("Msg Recieved");
    f_cmd_srv.position = {MIN_F + (-f_rate+1)/2 * f_gap};
    pub_f.publish(f_cmd_srv);

    // if (client.call(f_cmd_srv)) {
    ROS_INFO("Current_Position=%f",f_cmd_srv.position.at(0));
    // };
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "joy_ctrl");
	ros::NodeHandle nh;
	pub_turn = nh.advertise<std_msgs::Float32>("ros_talon/steering_angle", 0);
    // client = nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");
    pub_f = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position", 0);
	ros::Subscriber sub = nh.subscribe("joy",0,Callback);
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

	// ros::spin();
}
