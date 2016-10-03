#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <math.h>       /* sin */
#include <kmb172_p3/WavCycles.h> /*message type I defined for this assignment*/
#include <iostream>
#include <string>
#include <actionlib/server/simple_action_server.h>

class SinCommanderActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<kmb172_p3::WavCyclesAction> as_;
    
    // here are some message types to communicate with our client(s)
    kmb172_p3::WavCyclesGoal goal_; // goal message, received from client
    kmb172_p3::WavCyclesResult result_; // put results here, to be sent back to the client when done w/ goal
    kmb172_p3::WavCyclesFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client



public:
    SinCommanderActionServer(); //define the body of the constructor outside of class definition

    ~SinCommanderActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<kmb172_p3::WavCyclesAction>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, which is a member method
// of our class SinCommanderActionServer.  Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB takes one argument
// the final argument  "false" says don't start the server yet.  (We'll do this in the constructor)

SinCommanderActionServer::SinCommanderActionServer() :
   as_(nh_, "example_action", boost::bind(&SinCommanderActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of sinCommanderActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <kmb172_p3::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "kmb172_p3", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void SinCommanderActionServer::executeCB(const actionlib::SimpleActionServer<kmb172_p3::WavCyclesAction>::GoalConstPtr& goal) {
    //ROS_INFO("in executeCB");
    //ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes
    
    //....
    

    //KEVIN's NOTES
    //Need to know how to access request data fields
    //Need to implement a count of cycles completed 
	//use an if statement where I calculate the new time value and throw in a count ++ if needed

    //goal_ : float64 amplitude, float64 frequency, int32 numCycles
    //result_ : bool success
    //feedback_ : int32 numCyclesCompleted

    //grab the waveform parameters from the request (goal)
    double desiredAmplitude = goal_.amplitude;
    double desiredFrequency = goal_.frequency;
    int desiredNumCycles = goal_.numCycles;

    ros::Publisher my_publisher_object = nh_.advertise<std_msgs::Float64>("vel_cmd", 1);//add a publisher component
    ROS_INFO("Publisher Ready")
    
    //some code from P1
    double pubRate = 1000.0;//the rate at which vel_cmd will be published to
    std_msgs::Float64 vel_desired;// the data that I will modify and publish with
    double sin_arg = 0.0;//will be the input to the sin function
    double time = 0.0;
    double pi = 3.14159;
    ros::Rate naptime(pubRate);//create a ros object from the ros “Rate” class; 
    //set the sleep timer for 5Hz repetition rate (arg is in units of Hz)

    int cyclesCompleted = 0;//the number of periods of the sine wave that have passed

    while(ros::ok && cyclesCompleted < (desiredNumCycles)) {
        if ((time + (1/pubRate)) > (1/desiredFrequency)){
	    time = fmod((time + (1/pubRate)), (1/desiredFrequency));//determines location in wave
        }else{
	    time = fmod((time + (1/pubRate)), (1/desiredFrequency));//determines location in wave
	    cyclesCompleted = cyclesCompleted + 1;
	}
        vel_desired.data = desiredAmplitude * sin(2*pi*desiredFrequency*time);//calculates desired velocity
        my_publisher_object.publish(vel_desired); // publish the value--of type Float64-- 
        //to the topic "topic1"
	ROS_INFO("amp_cmd = %f", amp_cmd);
        ROS_INFO("freq_cmd = %f", freq_cmd);
	ROS_INFO("time = %f", time);
        //ros::spinOnce(); //no longer needed from P2?
        naptime.sleep();
    }

    // for illustration, populate the "result" message with two numbers:
    // the "input" is the message count, copied from goal->input (as sent by the client)
    // the "goal_stamp" is the server's count of how many goals it has serviced so far
    // if there is only one client, and if it is never restarted, then these two numbers SHOULD be identical...
    // unless some communication got dropped, indicating an error
    // send the result message back with the status of "success"

    g_count++; // keep track of total number of goals serviced since this server was started
    result_.output = g_count; // we'll use the member variable result_, defined in our class
    result_.goal_stamp = goal->input;
    
    // the class owns the action server, so we can use its member methods here
   
    // DEBUG: if client and server remain in sync, all is well--else whine and complain and quit
    // NOTE: this is NOT generically useful code; server should be happy to accept new clients at any time, and
    // no client should need to know how many goals the server has serviced to date
    if (g_count != goal->input) {
        ROS_WARN("hey--mismatch!");
        ROS_INFO("g_count = %d; goal_stamp = %d", g_count, result_.goal_stamp);
        g_count_failure = true; //set a flag to commit suicide
        ROS_WARN("informing client of aborted goal");
        as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
    }
    else {
         as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "WavCycles_action_server_node"); // name this node 

    ROS_INFO("instantiating the WavCycles action server: ");

    SinCommadnerActionServer as_object; // create an instance of the class "SinCommadnerActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    
    ros::spin();
//while (!g_count_failure) {
        //ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
   // }

    return 0;
}
