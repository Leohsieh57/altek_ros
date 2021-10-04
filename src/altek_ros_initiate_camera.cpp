#include <stdlib.h>     
#include <ros/ros.h>
#include <altek_ros/start_publish.h>
#include <string>

using namespace std;
using namespace ros;

int main(int argc, char** argv){
    ros::init(argc, argv, "altek_ros_initiate_camera"); 
    NodeHandle n;

    string pw, bash_path;
    n.getParam("altek/password", pw);
    pw.erase(remove(pw.begin(), pw.end(), '*'), pw.end());
    n.getParam("altek/path/bash_cmd", bash_path);

    string cmd = "echo '" + pw + "' | sudo -S bash "+bash_path;
    ROS_INFO("Initialting Camera.. ");
    system(cmd.c_str());

    ServiceClient client = n.serviceClient<altek_ros::start_publish>("altek_ros/start_publish");
    altek_ros::start_publish srv;
    client.call(srv);

    ROS_INFO("Start publish ROS topic! ");
    
    return 0;              
}