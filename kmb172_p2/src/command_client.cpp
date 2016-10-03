#include <ros/ros.h>
#include <kmb172_p2/SinCommandMsg.h> // this message type is defined in the current package
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char **argv) {

    //make a ServiceClient node
    ros::init(argc, argv, "command_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<kmb172_p2::SinCommandMsg>("sin_commander");

    //create a message data structure and populate it with a default return value
    kmb172_p2::SinCommandMsg srv;
    bool successful_call = false;

    double des_amp;//the desired amplitude
    double des_freq;//the desired frequency

    
    //get numbers via command line
    cout<<endl;
    cout << "enter desired amplitude: ";
    cin >> des_amp;
    cout<<endl;
    cout << "enter desired frequency: ";
    cin >> des_freq;

    //populate srv data structure with desired sine wave attributes
    srv.request.ampCommand = des_amp;//fill the relevent message fields with desired values
    srv.request.freqCommand = des_freq;

    //check for and report response from server
    if (client.call(srv)) {//call the service and check if the service ran correctly
        cout << "command processed successfully";
        cout<<endl;
    }

    return 0;
}
