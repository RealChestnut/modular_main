#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/String.h>
#include <vector>
#include <cmath>
#include <cstdio>
#include <chrono>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/Imu.h>
//#include "FAC_MAV/FAC_MAV_ctrler.h"

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>


#include "nav_msgs/Odometry.h"


ros::Publisher essential_data_log;
ros::Publisher test_data_log;
std_msgs::Float64MultiArray logging_data;
std_msgs::Float64 test_data;
geometry_msgs::Vector3 pos;
geometry_msgs::Vector3 pos_d;
geometry_msgs::Vector3 angle;
geometry_msgs::Vector3 angle_d;
geometry_msgs::Vector3 lin_vel;
geometry_msgs::Vector3 lin_vel_d;
std_msgs::Float32 delta_time;

float theta11=0, theta12=0, theta21=0, theta22=0;
float theta11_des=0, theta12_des=0, theta21_des=0, theta22_des=0;
float theta_main_arr[2];
float theta_sub_arr[2];
float desired_thrust_arr[8];
float thrust11=0,thrust12=0,thrust13=0,thrust14=0;
float thrust21=0,thrust22=0,thrust23=0,thrust24=0;
float F_xd=0,F_yd=0,F_zd=0;
float tau_r_d=0, tau_p_d=0, tau_y_d=0;

int16_t PWM_data_0=0;
int16_t PWM_data_1=0;
int16_t PWM_data_2=0;
int16_t PWM_data_3=0;

int16_t PWM_data_sub_0=0;
int16_t PWM_data_sub_1=0;
int16_t PWM_data_sub_2=0;
int16_t PWM_data_sub_3=0;


int16_t PWM_arr[8];
double angle_buffer = 0;

int32_t Sbus_arr[8];
int32_t sbus0=0,sbus1=0,sbus2=0,sbus3=0,sbus4=0,sbus5=0,sbus6=0,sbus7=0;

void sbus_data_callback(const std_msgs::Int32MultiArray::ConstPtr& msg);
void angle_callback(const geometry_msgs::Vector3& msg);
void desired_angle_callback(const geometry_msgs::Vector3& msg);
void pos_callback(const geometry_msgs::Vector3& msg);
void desired_pos_callback(const geometry_msgs::Vector3& msg);
void lin_vel_callback(const geometry_msgs::Vector3& msg);
void desired_lin_vel_callback(const geometry_msgs::Vector3& msg);

void Pwm_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);
void Pwm_sub_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);

void jointstateCallback(const sensor_msgs::JointState& msg);
void jointstate_sub_Callback(const sensor_msgs::JointState& msg);

void desired_theta_main_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void desired_theta_sub_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

void desired_thrust_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void desired_force_callback(const geometry_msgs::Vector3& msg);
void desired_torque_callback(const geometry_msgs::Vector3& msg);

void delta_time_callback(const std_msgs::Float32& msg);
void publisherSet();
int main(int argc, char **argv)
{
    ros::init(argc, argv, "logging_test_node");

    ros::NodeHandle nh;
    ros::Subscriber angle_log =nh.subscribe("/angle",1,angle_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber desired_angle_log =nh.subscribe("desired_angle",1,desired_angle_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber position_log =nh.subscribe("pos",1,pos_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber desired_position_log =nh.subscribe("pos_d",1,desired_pos_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber lin_vel_log = nh.subscribe("lin_vel",1,lin_vel_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber lin_vel_d_log = nh.subscribe("lin_vel_d",1,desired_lin_vel_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber PWM_data_log = nh.subscribe("PWMs",1,Pwm_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber PWM_data_sub_log = nh.subscribe("PWMs_sub",1,Pwm_sub_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber delta_time_log = nh.subscribe("delta_t",1,delta_time_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber servo_theta_main_log = nh.subscribe("joint_states",1,jointstateCallback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber servo_theta_sub_log = nh.subscribe("joint_states_sub",1,jointstate_sub_Callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber desired_theta_main_log = nh.subscribe("master_servo_cmd_pub",1,desired_theta_main_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber desired_theta_sub_log = nh.subscribe("master2slave_servo_pub",1,desired_theta_sub_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber desired_thrust_log = nh.subscribe("Force_comb",1,desired_thrust_callback,ros::TransportHints().tcpNoDelay()); 
    ros::Subscriber desired_force3_log = nh.subscribe("force_d",1,desired_force_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber desired_torque3_log = nh.subscribe("torque_d",1,desired_torque_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber sbus_data_plot = nh.subscribe("Sbus_data_plot",1,sbus_data_callback,ros::TransportHints().tcpNoDelay());


    essential_data_log=nh.advertise<std_msgs::Float64MultiArray>("simple_logging_data",10);
    ros::Timer timerPublish_log = nh.createTimer(ros::Duration(1.0/200.0),std::bind(publisherSet));
    ros::spin();
    return 0;

}

void publisherSet(){
     

    logging_data.data.resize(57);
    logging_data.data[0]=angle.x;
    logging_data.data[1]=angle.y;
    logging_data.data[2]=angle.z;
    logging_data.data[3]=angle_d.x;
    logging_data.data[4]=angle_d.y;
    logging_data.data[5]=angle_d.z;
    logging_data.data[6]=pos.x;
    logging_data.data[7]=pos.y;
    logging_data.data[8]=pos.z;
    logging_data.data[9]=pos_d.x;
    logging_data.data[10]=pos_d.y;
    logging_data.data[11]=pos_d.z;
    logging_data.data[12]=lin_vel.x;
    logging_data.data[13]=lin_vel.y;
    logging_data.data[14]=lin_vel.z;
    logging_data.data[15]=lin_vel_d.x;
    logging_data.data[16]=lin_vel_d.y;
    logging_data.data[17]=lin_vel_d.z;
    logging_data.data[18]=PWM_data_0;
    logging_data.data[19]=PWM_data_1;
    logging_data.data[20]=PWM_data_2;
    logging_data.data[21]=PWM_data_3;
    logging_data.data[22]=PWM_data_sub_0;
    logging_data.data[23]=PWM_data_sub_1;
    logging_data.data[24]=PWM_data_sub_2;
    logging_data.data[25]=PWM_data_sub_3;
    logging_data.data[26]=theta11;
    logging_data.data[27]=theta12;
    logging_data.data[28]=theta21;
    logging_data.data[29]=theta22;
    logging_data.data[30]=theta11_des;
    logging_data.data[31]=theta12_des;
    logging_data.data[32]=theta21_des;
    logging_data.data[33]=theta22_des;
    logging_data.data[34]=thrust11;
    logging_data.data[35]=thrust12;
    logging_data.data[36]=thrust13;
    logging_data.data[37]=thrust14;
    logging_data.data[38]=thrust21;
    logging_data.data[39]=thrust22;
    logging_data.data[40]=thrust23;
    logging_data.data[41]=thrust24;
    logging_data.data[42]=F_xd;
    logging_data.data[43]=F_yd;
    logging_data.data[44]=F_zd;
    logging_data.data[45]=tau_r_d;
    logging_data.data[46]=tau_p_d;
    logging_data.data[47]=tau_y_d;

    logging_data.data[48]=sbus0;
    logging_data.data[49]=sbus1;
    logging_data.data[50]=sbus2;
    logging_data.data[51]=sbus3;
    logging_data.data[52]=sbus4;
    logging_data.data[53]=sbus5;
    logging_data.data[54]=sbus6;
    logging_data.data[55]=sbus7;


    logging_data.data[56]=delta_time.data;
    essential_data_log.publish(logging_data);

}

void angle_callback(const geometry_msgs::Vector3& msg){
    angle.x=msg.x;//sugueng need!
    angle.y=msg.y;//data type should be equal
    angle.z=msg.z;

}

void desired_angle_callback(const geometry_msgs::Vector3& msg){
    angle_d.x=msg.x;
    angle_d.y=msg.y;
    angle_d.z=msg.z;

}
void pos_callback(const geometry_msgs::Vector3& msg){
    pos.x=msg.x;
    pos.y=msg.y;
    pos.z=msg.z;
}

void desired_pos_callback(const geometry_msgs::Vector3& msg){
    pos_d.x=msg.x;
    pos_d.y=msg.y;
    pos_d.z=msg.z;

}

void lin_vel_callback(const geometry_msgs::Vector3& msg){
 
    lin_vel.x = msg.x;
    lin_vel.y = msg.y;
    lin_vel.z = msg.z;

}

void desired_lin_vel_callback(const geometry_msgs::Vector3& msg){

    lin_vel_d.x = msg.x;
    lin_vel_d.y = msg.y;
    lin_vel_d.z = msg.z;
}

void Pwm_callback(const std_msgs::Int16MultiArray::ConstPtr& msg){

   for(int i=0;i<15;i++){
                PWM_arr[i]=msg->data[i];
        }
        PWM_data_0 = PWM_arr[0];
        PWM_data_1 = PWM_arr[1];
        PWM_data_2 = PWM_arr[2];
        PWM_data_3 = PWM_arr[3];
	
	PWM_data_sub_0 =PWM_arr[8];
	PWM_data_sub_1 =PWM_arr[9];
	PWM_data_sub_2 =PWM_arr[10];
	PWM_data_sub_3 =PWM_arr[11];
}

void Pwm_sub_callback(const std_msgs::Int16MultiArray::ConstPtr& msg){
/*
   for(int i=4;i<8;i++){
                PWM_arr[i]=msg->data[i];
        }
        PWM_data_sub_0 = PWM_arr[4];
        PWM_data_sub_1 = PWM_arr[5];
        PWM_data_sub_2 = PWM_arr[6];
        PWM_data_sub_3 = PWM_arr[7];*/
}

void sbus_data_callback(const std_msgs::Int32MultiArray::ConstPtr& msg){


	for(int i=0;i<8;i++){
                Sbus_arr[i]=msg->data[i];
        }
	sbus0=Sbus_arr[0];
	sbus1=Sbus_arr[1];
	sbus2=Sbus_arr[2];
	sbus3=Sbus_arr[3];
	sbus4=Sbus_arr[4];
	sbus5=Sbus_arr[5];
	sbus6=Sbus_arr[6];
	sbus7=Sbus_arr[7];



	

}



void delta_time_callback(const std_msgs::Float32& msg){

    delta_time.data=msg.data;

}

void jointstateCallback(const sensor_msgs::JointState& msg){

	theta11=msg.position[0];
	theta12=msg.position[1];
	theta21=msg.position[2];
	theta22=msg.position[3];
	

}
void jointstate_sub_Callback(const sensor_msgs::JointState& msg){
/*
	theta21=0;//msg.position[0];
	theta22=0;//msg.position[1]; */
}

void desired_theta_main_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	for(int i=0;i<2;i++){
        	        theta_main_arr[i]=msg->data[i];
        	}
        theta11_des = theta_main_arr[0];
        theta12_des = theta_main_arr[1];	

}
void desired_theta_sub_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	for(int i=0;i<2;i++){
                        theta_sub_arr[i]=msg->data[i];
                }
        theta21_des = theta_sub_arr[0];
        theta22_des = theta_sub_arr[1];

}
void desired_thrust_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
	for(int i=0;i<8;i++){
                        desired_thrust_arr[i]=msg->data[i];
                }
        thrust11 = desired_thrust_arr[0];
        thrust12 = desired_thrust_arr[1];
	thrust13 = desired_thrust_arr[2];
        thrust14 = desired_thrust_arr[3];
	thrust21 = desired_thrust_arr[4];
        thrust22 = desired_thrust_arr[5];
	thrust23 = desired_thrust_arr[6];
        thrust24 = desired_thrust_arr[7];

}
void desired_force_callback(const geometry_msgs::Vector3& msg){
	F_xd=msg.x;
	F_yd=msg.y;
	F_zd=msg.z;


}
void desired_torque_callback(const geometry_msgs::Vector3& msg){
	tau_r_d=msg.x;
        tau_p_d=msg.y;
        tau_y_d=msg.z;

}
