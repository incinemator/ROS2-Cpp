#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>
#include <string>
#include <list>
#include <iostream>
#include <map>

using namespace std;

class RosbotGetOut {
    public:
        RosbotGetOut(float speed);
        ~RosbotGetOut();
        void set_speed(float in_speed);
        void recenter();
        void navigate();
    private:
        RosbotClass rosbot;
};

int main(int argc, char **argv)
{
    ros::init(arc, argv, "rosbot_node");
    return 0;
}

void RosbotGetOut::navigate()
{
    rosbot.move_forward(speed);

    while(rosbot.get_laser(0) < 1.75) {

    }
}