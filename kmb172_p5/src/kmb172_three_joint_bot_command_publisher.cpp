#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "three_DOF_bot_publisher"); 
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS

    //advertise a topic for each joint
    ros::Publisher my_publisher_object_1 = n.advertise<std_msgs::Float64>("/three_DOF_robot/joint1_position_controller/command", 1);
    ros::Publisher my_publisher_object_2 = n.advertise<std_msgs::Float64>("/three_DOF_robot/joint2_position_controller/command", 1);
    ros::Publisher my_publisher_object_3 = n.advertise<std_msgs::Float64>("/three_DOF_robot/joint3_position_controller/command", 1);

    //"topic1" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    
    std_msgs::Float64 input_float_1; //create a variable of type "Float64", 

    std_msgs::Float64 input_float_2; //create a variable of type "Float64", 

    std_msgs::Float64 input_float_3; //create a variable of type "Float64", 
   
    ros::Rate naptime(0.3); //create a ros object from the ros “Rate” class; 

    input_float_1.data = 0.0;

    input_float_2.data = 0.0;

    input_float_3.data = 0.0;

    int poseNum = 0;//to be changed once per loop to switch between poses
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
	my_publisher_object_1.publish(input_float_1); // publish the value--of type Float64--
	my_publisher_object_2.publish(input_float_2); // publish the value--of type Float64--
	my_publisher_object_3.publish(input_float_3); // publish the value--of type Float64--

	if (poseNum == 0)
	{
	    poseNum = 1;
	    input_float_1.data = 0.5;
	    input_float_2.data = 0.5;
	    input_float_3.data = 0.5;
	}

	else
	{
	    poseNum = 0;
	    input_float_1.data = 0.0;
	    input_float_2.data = 0.0;
	    input_float_3.data = 0.0;
	}

	naptime.sleep(); 
    }
}
