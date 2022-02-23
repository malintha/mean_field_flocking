//
// Created by malintha on 10/14/20.
//
//#include <iostream>
#include "ros/ros.h"
#include "simulator_utils/Waypoint.h"
#include "ros/console.h"

//using namespace std;

class Drone {
    int id;
    ros::Subscriber state_sub;
    ros::NodeHandle nh;
    void state_cb(const simulator_utils::WaypointConstPtr& wp);

public:
    simulator_utils::Waypoint state;
    Drone(int id, const ros::NodeHandle &n);
    simulator_utils::Waypoint get_state() const;


};

