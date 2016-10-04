#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <kmb172_p3/WavCyclesAction.h> /*message type I defined for this assignment*/
#include <iostream>
#include <string>
using namespace std;

void doneCb(const actionlib::SimpleClientGoalState& state,
        const kmb172_p3::WavCyclesResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("cycles completed");
}

int main(int argc, char **argv) {

    //make a node
    ros::init(argc, argv, "command_client");
    ros::NodeHandle n;
    
    //make a goal object
    kmb172_p3::WavCyclesGoal goal;

    //make the action client
    actionlib::SimpleActionClient<kmb172_p3::WavCyclesAction> action_client("example_action", true);

    // attempt to connect to the server :
    ROS_INFO (" waiting for server : ") ;

    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever

        if (!server_exists) {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

    //

    double des_amp;//the desired amplitude
    double des_freq;//the desired frequency
    int des_num_cycles;//the desired number of cycles

    
    //get numbers via command line
    cout<<endl;
    cout << "enter desired amplitude: ";
    cin >> des_amp;
    cout<<endl;
    cout << "enter desired frequency: ";
    cin >> des_freq;
    cout<<endl;
    cout << "enter desired number of cycles: ";
    cin >> des_num_cycles;

    //populate goal data structure with our desired values
    goal.amplitude = des_amp;
    goal.frequency = des_freq;
    goal.numCycles = des_num_cycles;

    //send the goal to the server
    action_client.sendGoal(goal,&doneCb);

    return 0;
}
