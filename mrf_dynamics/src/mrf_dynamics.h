//
// Created by malintha on 10/13/20.
//

#ifndef MRF_DYNAMICS_MRF_DYNAMICS_H
#define MRF_DYNAMICS_MRF_DYNAMICS_H

#include <eigen3/Eigen/Dense>
#include "Definitions.h"

using namespace std;
using namespace Eigen;

class mrf_dynamics {
    int num_nodes;
    int robot_id;
    Vector3d roosting_center;
    double tau;

    double Qi_tilde(vector<Node> &nodes, Node *n);
    bool dis_equal(vector<double> Q, vector<double> P);

    void do_mfa();
    double psi_u();
    double psi_p();

public:
    mrf_dynamics(int robot_id, int num_nodes);

};

#endif //MRF_DYNAMICS_MRF_DYNAMICS_H
