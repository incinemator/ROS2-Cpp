#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>
#include <string>
#include <list>
#include <iostream>
#include <map>

using namespace std;

class RosbotGetOut {
    public:
        RosbotGetOut();
        void recenter();
        void navigate();

        RosbotClass rosbot;
};

