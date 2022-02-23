//
// Created by malintha on 10/13/20.
//

#ifndef MRF_DYNAMICS_DEFINITIONS_H
#define MRF_DYNAMICS_DEFINITIONS_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <simulator_utils/Waypoint.h>

using namespace std;
using namespace Eigen;
using namespace geometry_msgs;

struct Label {
    Eigen::Vector3d u;
    Vector3d pt;
    Vector3d vt;
    Label();
    Label(const Vector3d& u_, const simulator_utils::Waypoint& wp, const double& tau);
};

// computations done in NED frame
Label::Label(const Vector3d& u_, const simulator_utils::Waypoint& wp, const double& tau) {
    Eigen::Vector3d p, v, a;
    this->u = u_;
    p << wp.position.x, wp.position.y, wp.position.z;
    v << wp.velocity.x, wp.velocity.y, wp.velocity.z;
    a << wp.acceleration.x, wp.acceleration.y, wp.acceleration.z;
    this->pt = 0.5 * u_ * pow(tau, 2) + v * tau + p;
    this->vt = u_ * tau + v;
}

Label::Label() = default;

class Node {

public:
    int id;
    double z{};
    vector<Label> X;
    Label x;
    vector<double> Qx;

    Node(vector<Label> &labels, int id) {
        this->id = id;
        this->X = labels;
    }
};





#endif //MRF_DYNAMICS_DEFINITIONS_H
