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
    
    ros::Rate rate(1);
    altek_ros::start_publish srv;
    for(uint nFail = 0; !client.call(srv); nFail++){
        if(nFail < 10)
            ROS_WARN("Waiting for response from server.. ");

        else{
            ROS_ERROR("Can't receive reponse from server, aborting.. "); 
            exit(-1);
        }
        rate.sleep();
    }
    
    return 0;              
}