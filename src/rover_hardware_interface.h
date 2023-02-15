#ifndef ROVER_HARDWARE_INTERFACE_H_
#define ROVER_HARDWARE_INTERFACE_H_

#include <boost/assign/list_of.hpp>
#include <sstream>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <ros/console.h>

class RoverHardwareInterface : public hardware_interface::RobotHW {
public:
	RoverHardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node, double target_max_wheel_angular_speed);

	void updateJointsFromHardware(const ros::Duration& period);
	void writeCommandsToHardware();

private:
	ros::NodeHandle _node;
	ros::NodeHandle _private_node;

	hardware_interface::JointStateInterface _joint_state_interface;
	hardware_interface::VelocityJointInterface _velocity_joint_interface;

	ros::Subscriber _front_left_wheel_angle_sub;
	ros::Subscriber _front_right_wheel_angle_sub;
	ros::Subscriber _rear_left_wheel_angle_sub;
	ros::Subscriber _rear_right_wheel_angle_sub;

	ros::Publisher _front_left_wheel_vel_pub;
	ros::Publisher _front_right_wheel_vel_pub;
	ros::Publisher _rear_left_wheel_vel_pub;
	ros::Publisher _rear_right_wheel_vel_pub;

	struct Joint {
		double position;
		double position_offset;
		double velocity;
		double effort;
		double velocity_command;

		Joint()
			: position(0)
			, velocity(0)
			, effort(0)
			, velocity_command(0) {}
	} _joints[4];

	double _front_left_wheel_angle;
	double _front_right_wheel_angle;
	double _rear_left_wheel_angle;
	double _rear_right_wheel_angle;

	double _max_wheel_angular_speed;

	void registerControlInterfaces();
	void frontLeftWheelAngleCallback(const std_msgs::Float64& msg);
	void frontRightWheelAngleCallback(const std_msgs::Float64& msg);
	void rearLeftWheelAngleCallback(const std_msgs::Float64& msg);
	void rearRightWheelAngleCallback(const std_msgs::Float64& msg);
	void limitMaximumSpeed(double& max_speed_front_left_wheel, double& max_speed_front_right_wheel, 
	double& max_speed_rear_left_wheel, double& max_speed_rear_right_wheel);
};

RoverHardwareInterface::RoverHardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node, double target_max_wheel_angular_speed)
	: _node(node)
	, _private_node(private_node)
	, _max_wheel_angular_speed(target_max_wheel_angular_speed) {
	registerControlInterfaces();

	_front_left_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/rover/front_left_wheel/target_velocity", 1);
	_front_right_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/rover/front_right_wheel/target_velocity", 1);
	_rear_left_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/rover/rear_left_wheel/target_velocity", 1);
	_rear_right_wheel_vel_pub = _node.advertise<std_msgs::Float64>("/rover/rear_right_wheel/target_velocity", 1);
	_front_left_wheel_angle_sub = _node.subscribe("rover/front_left_wheel/angle", 1, &RoverHardwareInterface::frontLeftWheelAngleCallback, this);
	_front_right_wheel_angle_sub = _node.subscribe("rover/front_right_wheel/angle", 1, &RoverHardwareInterface::frontRightWheelAngleCallback, this);
	_rear_left_wheel_angle_sub = _node.subscribe("rover/rear_left_wheel/angle", 1, &RoverHardwareInterface::rearLeftWheelAngleCallback, this);
	_rear_right_wheel_angle_sub = _node.subscribe("rover/rear_right_wheel/angle", 1, &RoverHardwareInterface::rearRightWheelAngleCallback, this);
}

void RoverHardwareInterface::writeCommandsToHardware() {
	double instant_angle_speed_front_left = _joints[0].velocity_command;
	double instant_angle_speed_front_right = _joints[1].velocity_command;
	double instant_angle_speed_rear_left = _joints[2].velocity_command;
	double instant_angle_speed_rear_right = _joints[3].velocity_command;
	
	limitMaximumSpeed(instant_angle_speed_front_left, instant_angle_speed_front_right, instant_angle_speed_rear_left, instant_angle_speed_rear_right);

	std_msgs::Float64 front_left_wheel_vel_msg;
	std_msgs::Float64 front_right_wheel_vel_msg;
	std_msgs::Float64 rear_left_wheel_vel_msg;
	std_msgs::Float64 rear_right_wheel_vel_msg;

	front_left_wheel_vel_msg.data = instant_angle_speed_front_left;
	front_right_wheel_vel_msg.data = instant_angle_speed_front_right;
	rear_left_wheel_vel_msg.data = instant_angle_speed_rear_left;
	rear_right_wheel_vel_msg.data = instant_angle_speed_rear_right;

	_front_left_wheel_vel_pub.publish(front_left_wheel_vel_msg);
	_front_right_wheel_vel_pub.publish(front_right_wheel_vel_msg);
	_rear_left_wheel_vel_pub.publish(rear_left_wheel_vel_msg);
	_rear_right_wheel_vel_pub.publish(rear_right_wheel_vel_msg);
}

void RoverHardwareInterface::updateJointsFromHardware(const ros::Duration& period) {
	double delta_front_left_wheel = _front_left_wheel_angle - _joints[0].position - _joints[0].position_offset;
	double delta_front_right_wheel = _front_right_wheel_angle - _joints[1].position - _joints[1].position_offset;
	double delta_rear_left_wheel = _rear_left_wheel_angle - _joints[2].position - _joints[2].position_offset;
	double delta_rear_right_wheel = _rear_right_wheel_angle - _joints[3].position - _joints[3].position_offset;
	
	if (std::abs(delta_front_left_wheel) < 1) {
		_joints[0].position += delta_front_left_wheel;
		_joints[0].velocity = delta_front_left_wheel / period.toSec();
	} else {
		_joints[0].position_offset += delta_front_left_wheel;
	}

	if (std::abs(delta_front_right_wheel) < 1) {
		_joints[1].position += delta_front_right_wheel;
		_joints[1].velocity = delta_front_right_wheel / period.toSec();
	} else {
		_joints[1].position_offset += delta_front_right_wheel;
	}

	if (std::abs(delta_rear_left_wheel) < 1) {
		_joints[2].position += delta_rear_left_wheel;
		_joints[2].velocity = delta_rear_left_wheel / period.toSec();
	} else {
		_joints[2].position_offset += delta_rear_left_wheel;
	}

	if (std::abs(delta_rear_right_wheel) < 1) {
		_joints[3].position += delta_rear_right_wheel;
		_joints[3].velocity = delta_rear_right_wheel / period.toSec();
	} else {
		_joints[3].position_offset += delta_rear_right_wheel;
	}
}

void RoverHardwareInterface::registerControlInterfaces() {
	ros::V_string joint_names = boost::assign::list_of("front_left_wheel_to_base")("front_right_wheel_to_base")("rear_left_wheel_to_base")("rear_right_wheel_to_base");

	for (unsigned int i = 0; i < joint_names.size(); i++) {
		hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &_joints[i].position, &_joints[i].velocity, &_joints[i].effort);
		_joint_state_interface.registerHandle(joint_state_handle);

		hardware_interface::JointHandle joint_handle(joint_state_handle, &_joints[i].velocity_command);
		_velocity_joint_interface.registerHandle(joint_handle);
	}
	registerInterface(&_joint_state_interface);
	registerInterface(&_velocity_joint_interface);
}

void RoverHardwareInterface::frontLeftWheelAngleCallback(const std_msgs::Float64& msg) {
	_front_left_wheel_angle = msg.data;
}

void RoverHardwareInterface::frontRightWheelAngleCallback(const std_msgs::Float64& msg) {
	_front_right_wheel_angle = msg.data;
}
void RoverHardwareInterface::rearLeftWheelAngleCallback(const std_msgs::Float64& msg) {
	_rear_left_wheel_angle = msg.data;
}

void RoverHardwareInterface::rearRightWheelAngleCallback(const std_msgs::Float64& msg) {
	_rear_right_wheel_angle = msg.data;
}

void RoverHardwareInterface::limitMaximumSpeed(double& max_speed_front_left_wheel, double& max_speed_front_right_wheel, 
	double& max_speed_rear_left_wheel, double& max_speed_rear_right_wheel) {
	if (max_speed_front_left_wheel  >  _max_wheel_angular_speed) {
		max_speed_front_left_wheel =  _max_wheel_angular_speed;
	}	

	if (max_speed_front_right_wheel  >  _max_wheel_angular_speed) {
		max_speed_front_right_wheel =  _max_wheel_angular_speed;
	}	

	if (max_speed_rear_left_wheel  >  _max_wheel_angular_speed) {
		max_speed_rear_left_wheel =  _max_wheel_angular_speed;
	}	

	if (max_speed_rear_right_wheel  >  _max_wheel_angular_speed) {
		max_speed_rear_right_wheel =  _max_wheel_angular_speed;
	}		
}

#endif // ROVER_HARDWARE_INTERFACE_H_