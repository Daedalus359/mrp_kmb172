//example ROS client:
// first run: rosrun example_ROS_service example_ROS_service
// then start this node:  rosrun example_ROS_service example_ROS_client

#include <ros/ros.h>
#include <kmb172_p2/SinCommandMsg.h> // this message type is defined in the current package
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "command_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<example_ros_service::ExampleServiceMsg>("sin_commander");
    kmb172_p2::SinCommandMsg srv;
    bool successful_call = false;

    double des_amp;//the desired amplitude
    double des_freq;//the desired frequency

    cout<<endl;
    cout << "enter desired amplitude: ";
    cin >> des_amp;
    cout<<endl;
    cout << "enter desired frequency: ";
    cin >> des_freq

    srv.request.ampCommand = des_amp;//fill the relevent message fields with desired values
    srv.request.ampCommand = des_freq;

    if (client.call(srv)) {//call the service and check if the service ran correctly
        cout << "command received";
    }

    return 0;
}
