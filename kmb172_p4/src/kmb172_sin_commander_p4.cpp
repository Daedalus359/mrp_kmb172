#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <math.h>       /* sin */

int main(int argc, char **argv) {
    ros::init(argc, argv, "sin_commander"); // name of this node will be "sin_commander"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("pos_cmd_1", 1);
    ros::Publisher my_publisher_object_2 = n.advertise<std_msgs::Float64>("pos_cmd_2", 1);
    //"vel_cmd" is the name of the topic to which we will publish
    
    // the data that we will modify and publish with
    std_msgs::Float64 pos_desired_1;
    std_msgs::Float64 pos_desired_2;

    double frequency_1 = 0.0;//create the variable that represents desired frequency
    double amplitude_1 = 0.0;//create the variable that represents desired amplitude
    double frequency_2 = 0.0;//create the variable that represents desired frequency
    double amplitude_2 = 0.0;//create the variable that represents desired amplitude

    //ask for and store desired frequency
    std::cout << "Input desired frequency for joint 1: ";
    std::cin >> frequency_1;

    //ask for and store desired frequency
    std::cout << "Input desired frequency for joint 2: ";
    std::cin >> frequency_2;

    //ask for and store desired amplitude
    std::cout << "Input desired amplitude for joint 1: ";
    std::cin >> amplitude_1;

    //ask for and store desired amplitude
    std::cout << "Input desired amplitude for joint 2: ";
    std::cin >> amplitude_2;

    double pubRate = 1000.0;//the rate at which vel_cmd will be published to
   
    ros::Rate naptime(pubRate);//create a ros object from the ros “Rate” class; 
    //set the sleep timer for 5Hz repetition rate (arg is in units of Hz)

    double sin_arg_1 = 0.0;//will be the input to the sin function for joint 1
    double time_1 = 0.0;

    double sin_arg_2 = 0.0;//will be the input to the sin function for joint 2
    double time_2 = 0.0;

    double pi = 3.14159;
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
	time_1 = fmod((time_1 + (1/pubRate)), (1/frequency_1));//determines location in current 
        pos_desired_1.data = amplitude_1 * sin(2*pi*frequency_1*time_1);//calculates desired position
        my_publisher_object.publish(pos_desired_1); // publish the value--of type Float64-

	ROS_INFO("pos_desired_1 updated");

	time_2 = fmod((time_2 + (1/pubRate)), (1/frequency_2));//determines location in current 
        pos_desired_2.data = amplitude_2 * sin(2*pi*frequency_2*time_2);//calculates desired position
        my_publisher_object_2.publish(pos_desired_2); // publish the value--of type Float64-- 
        //to the topic "topic1"


	//the next line will cause the loop to sleep for the balance of the desired period 
        // to achieve the specified loop frequency

	naptime.sleep();
    }
}
