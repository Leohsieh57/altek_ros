#include <stdlib.h>     
#include <ros/ros.h>
#include <altek_ros/start_publish.h>
#include <string>

using namespace std;
using namespace ros;

int main(int argc, char** argv){
    ros::init(argc, argv, "altek_ros_initiate_camera"); 
    NodeHandle n;

    //check camera fisrt
    bool open_app;
    n.getParam("altek/option/open_app", open_app);
    // check if we succeeded
    if(open_app){
       //open app
        string pw, bash_path;
        n.getParam("altek/password", pw);
        pw.erase(remove(pw.begin(), pw.end(), '*'), pw.end());
        n.getParam("altek/path/bash_cmd", bash_path);

        string cmd = "echo '" + pw + "' | sudo -S bash "+bash_path;
        system(cmd.c_str());
        
    }

    ROS_INFO("Initialting Camera.. ");
    ServiceClient client = n.serviceClient<altek_ros::start_publish>("altek_ros/start_publish");
    altek_ros::start_publish srv;
    client.call(srv);
    
    return 0;              
}