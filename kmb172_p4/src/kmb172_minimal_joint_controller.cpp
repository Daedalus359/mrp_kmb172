#include <ros/ros.h> //ALWAYS need to include this
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>

//a simple saturation function; provide saturation threshold, sat_val, and arg to be saturated, val
double sat(double val, double sat_val) {
    if (val>sat_val)
        return (sat_val);
    if (val< -sat_val)
        return (-sat_val);
    return val;
    
}

double g_pos_cmd_1=0.0; //position command input-- global var
double g_pos_cmd_2=0.0; //Same as above but for joint 2


void posCmd_1_CB(const std_msgs::Float64& pos_cmd_1_msg) 
{ 
  ROS_INFO("received value of pos_cmd_1 is: %f",pos_cmd_1_msg.data); 
  g_pos_cmd_1 = pos_cmd_1_msg.data;
} 

void posCmd_2_CB(const std_msgs::Float64& pos_cmd_2_msg) //a callback for the second joint
{ 
  ROS_INFO("received value of pos_cmd_2 is: %f",pos_cmd_2_msg.data); 
  g_pos_cmd_2 = pos_cmd_2_msg.data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_joint_controller");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);
    
    // make sure service is available before attempting to proceed, else node will crash
    bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
      ROS_INFO("waiting for apply_joint_effort service");
      half_sec.sleep();
    }
    ROS_INFO("apply_joint_effort service exists");

    //**!!** does this spot require duplication? I don't think so.
    ros::ServiceClient set_trq_client = 
       nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    
    service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/get_joint_properties",true);
      ROS_INFO("waiting for /gazebo/get_joint_properties service");
      half_sec.sleep();
    }
    ROS_INFO("/gazebo/get_joint_properties service exists");
    
    ros::ServiceClient get_jnt_state_client = 
       nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");

    gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg_1;
    gazebo_msgs::GetJointProperties get_joint_state_srv_msg_1;

    gazebo_msgs::ApplyJointEffort effort_cmd_srv_msg_2;
    gazebo_msgs::GetJointProperties get_joint_state_srv_msg_2;
    
    ros::Publisher trq_1_publisher = nh.advertise<std_msgs::Float64>("jnt_1_trq", 1); 
    ros::Publisher vel_1_publisher = nh.advertise<std_msgs::Float64>("jnt_1_vel", 1);     
    ros::Publisher pos_1_publisher = nh.advertise<std_msgs::Float64>("jnt_1_pos", 1);  

    //same as above, but for joint 2
    ros::Publisher trq_2_publisher = nh.advertise<std_msgs::Float64>("jnt_2_trq", 1); 
    ros::Publisher vel_2_publisher = nh.advertise<std_msgs::Float64>("jnt_2_vel", 1);     
    ros::Publisher pos_2_publisher = nh.advertise<std_msgs::Float64>("jnt_2_pos", 1);


    ros::Publisher joint_1_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_1_states", 1);
    ros::Publisher joint_2_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_2_states", 1); 

    ros::Subscriber pos_cmd_1_subscriber = nh.subscribe("pos_cmd_1",1,posCmd_1_CB); 
    ros::Subscriber pos_cmd_2_subscriber = nh.subscribe("pos_cmd_2",1,posCmd_2_CB); 
     
    std_msgs::Float64 trq_1_msg;
    std_msgs::Float64 trq_2_msg;

    //I don't know what these do yet
    std_msgs::Float64 q1_msg,q1dot_msg;
    std_msgs::Float64 q2_msg,q2dot_msg;

    sensor_msgs::JointState joint_1_state_msg;
    sensor_msgs::JointState joint_2_state_msg;

    double q1, q1dot;
    double q2, q2dot;

    double dt = 0.01;
    ros::Duration duration(dt);
    ros::Rate rate_timer(1/dt);
    
    effort_cmd_srv_msg_1.request.joint_name = "joint1";//does this require a corresponding change in another file?
    effort_cmd_srv_msg_1.request.effort = 0.0;
    effort_cmd_srv_msg_1.request.duration= duration;

    effort_cmd_srv_msg_2.request.joint_name = "joint2";
    effort_cmd_srv_msg_2.request.effort = 0.0;
    effort_cmd_srv_msg_2.request.duration= duration;


    get_joint_state_srv_msg_1.request.joint_name = "joint1";//does this require a corresponding change in another file?
    get_joint_state_srv_msg_2.request.joint_name = "joint2";

    //double q1_des = 1.0;
    double q1_err;
    double q2_err;

    double Kp = 10.0;
    double Kv = 3;
    double trq_cmd;

    // set up the joint_state_msg fields to define a single joint,
    // called joint1, and initial position and vel values of 0
	joint_1_state_msg.header.stamp = ros::Time::now();
	joint_1_state_msg.name.push_back("joint1");
        joint_1_state_msg.position.push_back(0.0);
        joint_1_state_msg.velocity.push_back(0.0);

// set up the joint_state_msg fields to define a single joint,
    // called joint2, and initial position and vel values of 0
	joint_2_state_msg.header.stamp = ros::Time::now();
	joint_2_state_msg.name.push_back("joint2");
        joint_2_state_msg.position.push_back(0.0);
        joint_2_state_msg.velocity.push_back(0.0);

    while(ros::ok()) {    
        get_jnt_state_client.call(get_joint_state_srv_msg_1);
        q1 = get_joint_state_srv_msg_1.response.position[0];
        q1_msg.data = q1;
        pos_1_publisher.publish(q1_msg);

        get_jnt_state_client.call(get_joint_state_srv_msg_2);
        q2 = get_joint_state_srv_msg_2.response.position[0];
        q2_msg.data = q2;
        pos_2_publisher.publish(q2_msg);
        
        q1dot = get_joint_state_srv_msg_1.response.rate[0];
        q1dot_msg.data = q1dot;
        vel_1_publisher.publish(q1dot_msg);

        q2dot = get_joint_state_srv_msg_2.response.rate[0];
        q2dot_msg.data = q2dot;
        vel_2_publisher.publish(q2dot_msg);

	joint_1_state_msg.header.stamp = ros::Time::now();
        joint_1_state_msg.position[0] = q1; 
        joint_1_state_msg.velocity[0] = q1dot;
	joint_1_state_publisher.publish(joint_1_state_msg);

        joint_2_state_msg.header.stamp = ros::Time::now();
        joint_2_state_msg.position[0] = q1; 
        joint_2_state_msg.velocity[0] = q1dot;
	joint_2_state_publisher.publish(joint_2_state_msg);
        
        //ROS_INFO("q1 = %f;  q1dot = %f",q1,q1dot);
        //watch for periodicity
        q1_err= g_pos_cmd_1-q1;
        if (q1_err>M_PI) {
            q1_err -= 2*M_PI;
        }
        if (q1_err< -M_PI) {
            q1_err += 2*M_PI;
        }  

        q2_err= g_pos_cmd_2-q2;
        if (q2_err>M_PI) {
            q2_err -= 2*M_PI;
        }
        if (q2_err< -M_PI) {
            q2_err += 2*M_PI;
        }       
            
        trq_cmd = Kp*(q1_err)-Kv*q1dot;
        //trq_cmd = sat(trq_cmd, 10.0); //saturate at 1 N-m
        trq_1_msg.data = trq_cmd;
        trq_1_publisher.publish(trq_1_msg);
        // send torque command to Gazebo
        effort_cmd_srv_msg_1.request.effort = trq_cmd;
        set_trq_client.call(effort_cmd_srv_msg_1);
        //make sure service call was successful
        bool result_1 = effort_cmd_srv_msg_1.response.success;
        if (!result_1)
            ROS_WARN("service call to apply_joint_effort for joint 1 failed!");
        ros::spinOnce();
	rate_timer.sleep();

        trq_cmd = Kp*(q2_err)-Kv*q2dot;
        //trq_cmd = sat(trq_cmd, 10.0); //saturate at 1 N-m
        trq_2_msg.data = trq_cmd;
        trq_2_publisher.publish(trq_2_msg);
        // send torque command to Gazebo
        effort_cmd_srv_msg_2.request.effort = trq_cmd;
        set_trq_client.call(effort_cmd_srv_msg_2);
        //make sure service call was successful
        bool result_2 = effort_cmd_srv_msg_2.response.success;
        if (!result_2)
            ROS_WARN("service call to apply_joint_effort for joint 2 failed!");
        ros::spinOnce();
	rate_timer.sleep();
  }
}
