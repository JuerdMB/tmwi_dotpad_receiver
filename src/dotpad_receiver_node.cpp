#include <ros/ros.h>
#include "DotPadManager.h"

CString DOTPAD_COMPORT = "5";
ros::Publisher pub_screenstatus;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DotPadController");
    ros::NodeHandle nodeHandle("~");

    DotPadManager manager(nodeHandle, DOTPAD_COMPORT);

    while (ros::ok())
    {
        // Publish trigger to TOPIC_SCREEN_READY

        // Wait until callback finished
        // Publish trigger for mapper to send new data

        ros::spinOnce();
    }

    return 0;
};