#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <math.h>       /* sin */
#include <kmb172_p2/SinCommandMsg.h> /*message type I defined for this assignment*/
#include <iostream>
#include <string>

double amp_cmd
double freq_cmd

bool callback(example_ros_service::ExampleServiceMsgRequest& request, example_ros_service::ExampleServiceMsgResponse& response)
{
    ROS_INFO("callback activated");
    double reqd_amp_cmd(request.ampCommand);//store requested amplitude command
    double reqd_freq_cmd(request.freqCommand);//store requested frequency command

    amp_cmd = reqd_amp_cmd;
    freq_cmd = reqd_freq_cmd;
    
    response.success=true;//actual commands about to be issued

    ros::init(argc, argv, "sin_commanding_node");
    ros::NodeHandle p;

    ros::Publisher my_publisher_object = p.advertise<std_msgs::Float64>("vel_cmd", 1);
    //"vel_cmd" is the name of the topic to which we will publish
    
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sin_commander_service");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("sin_commander", callback);
    ROS_INFO("Ready to command a sine wave.");

    ros::spin();

    return 0;
}

int commanding_section(double amp_cmd, double freq_cmd) {//takes arguments from service parts now
    ros::init(argc, argv, "sin_commander"); // name of this node will be "sin_commander"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);
    //"vel_cmd" is the name of the topic to which we will publish
    
    std_msgs::Float64 vel_desired;// the data that we will modify and publish with

    double frequency = 0.0;//create the variable that represents desired frequency
    double amplitude = 0.0;//create the variable that represents desired amplitude

    //*** REFORMULATE AS A SERVICE ***
    //ask for and store desired frequency
    std::cout << "Input desired frequency: ";
    std::cin >> frequency;

    //ask for and store desired amplitude
    std::cout << "Input desired amplitude: ";
    std::cin >> amplitude;

    double pubRate = 1000.0;//the rate at which vel_cmd will be published to
   
    ros::Rate naptime(pubRate);//create a ros object from the ros “Rate” class; 
    //set the sleep timer for 5Hz repetition rate (arg is in units of Hz)

    double sin_arg = 0.0;//will be the input to the sin function
    double time = 0.0;
    double pi = 3.14159;
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
	time = fmod((time + (1/pubRate)), (1/frequency));//determines location in current wavelength
        vel_desired.data = amplitude * sin(2*pi*frequency*time);//calculates desired velocity
        my_publisher_object.publish(vel_desired); // publish the value--of type Float64-- 
        //to the topic "topic1"
	//the next line will cause the loop to sleep for the balance of the desired period 
        // to achieve the specified loop frequency 
	naptime.sleep();
    }
}
