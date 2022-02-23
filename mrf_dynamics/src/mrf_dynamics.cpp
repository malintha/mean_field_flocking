//
// Created by malintha on 10/13/20.
//

#include "Definitions.h"
#include "simulator_utils/simulator_utils.h"
#include "Drone.h"
#include <algorithm>
#include "ros/console.h"
#include <chrono>
#include "visualization_msgs/Marker.h"

float a,b,ka,kr,KR;
static Vector3d rc(0,0,0);

void draw_edges(const ros::Publisher& edge_pub, vector<geometry_msgs::Point> neighbor_pos,Vector3d rgb) {
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.header.frame_id = "map";
    m.action = visualization_msgs::Marker::ADD;   
    m.pose.orientation.w = 1;

    for(int k=0;k < neighbor_pos.size();k++) {
        for(int l=0;l<neighbor_pos.size();l++) {
          geometry_msgs::Point p1 = neighbor_pos[k];
          geometry_msgs::Point p2 = neighbor_pos[l];

          p1.y = -p1.y;  
          p2.y = -p2.y;  
          
          m.points.push_back(p1);
          m.points.push_back(p2);          
        } 
    }
    m.scale.x = 0.05;
    m.color.r = rgb[0];
    m.color.g = rgb[1];
    m.color.b = rgb[2];

    m.color.a =1;
    edge_pub.publish(m);
}

void draw_circle(const ros::Publisher& edge_pub, geometry_msgs::Point pos ,Vector3d rgb) {
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.type = visualization_msgs::Marker::CYLINDER;
    m.header.frame_id = "map";
    m.action = visualization_msgs::Marker::ADD;   
    m.pose.orientation.w = 1;

    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = 0.01;

    m.pose.position.x = pos.x;
    m.pose.position.y = -pos.y;
    m.pose.position.z = pos.z;

    m.color.r = rgb[0];
    m.color.g = rgb[1];
    m.color.b = rgb[2];
    m.color.a =0.5;

    edge_pub.publish(m);
}

void publish_goal(const ros::Publisher& goal_pub) {
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.type = visualization_msgs::Marker::ARROW;
    m.header.frame_id = "map";
    m.action = visualization_msgs::Marker::ADD;
    m.color.r = 1;
    m.color.g = 0.1;
    m.color.b = 1;
    m.color.a = 1;

    m.scale.x = 1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.pose.position.x = rc[0];
    m.pose.position.y = -rc[1];
    m.pose.position.z = rc[2];

    m.pose.orientation.w = -0.707;
    m.pose.orientation.y = 0.707;
    m.pose.orientation.x = 0.0;
    m.pose.orientation.z = 0.0;

    goal_pub.publish(m);
}

double psi_p(const Label& xi, const Label& xj) {
    double d = (xi.pt - xj.pt).norm();
    return 5*(-a * exp(-(d * ka)) + b * exp(-(d * kr))) +1 ;
}

double psi_ue(const Label& l) {
    return 3*exp((l.pt - rc).norm()/KR);
}

double psi_u(const Label& l, Vector3d& pt) {
    return psi_ue(l);
}

double compatibility(Label li, Label lj) {
    double mu = 1.5;
    Vector3d max_vel = {1,1,0};
    bool vel_diff = (li.vt - lj.vt).norm();
    return mu*vel_diff;
}

double Qi_tilde_(vector<Node> &nodes, Node *n, const std::vector<simulator_utils::Waypoint>& states) {
    double sumEx = 0;
    for (int j = 0; j < nodes.size(); j++) {
        if(nodes[j].id==n->id)
            continue;
        double Ex = 0;
        Label li = n->x;
        // expectation wrt neighbours values
        for (int m = 0; m < nodes[j].X.size(); m++) {
            Label lj = nodes[j].X[m];
            double psi_p_ = psi_p(nodes[j].X[m], li);
            double ex = compatibility(li,lj)*nodes[j].Qx[m] * psi_p_;
            sumEx += ex;
        }
        sumEx += Ex;
    }
    return sumEx;
}

bool dis_equal(vector<double> Q, vector<double> P) {
    for (int m = 0; m < P.size(); m++) {
        if (abs(Q[m] - P[m]) > 1e-2) {
            return false;
        }
    }
    return true;
}

double kl_dis(vector<double> Q_o, vector<double> Q_n) {
    double dis;
    for(int i=0;i<Q_o.size();i++) {
        dis = dis + Q_o[i]*log(Q_o[i]/Q_n[i]);
    }
    return dis;
}

bool satisfy_label(vector<Label> &labels, Label &label, const std::vector<simulator_utils::Waypoint>& states) {

    for(int i=0;i<states.size();i++) {
        geometry_msgs::Point wp = states[i].position;
        Vector3d pos = {wp.x, wp.y, wp.z};
        if((pos - label.pt).norm() < 0.5) {
            return false;
        }
    }

    if(labels.empty())
        return true;
    else {
        for(auto & m : labels) {
            double dist = (m.pt - label.pt).norm();
            if(dist < 0.5) {
                return false;
            }
        }
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "~");
    
    int robot_id = std::atoi(argv[1]); 

    // K : number of neighbours considering
    int K = std::atoi(argv[2]); 
    int n_robots = std::atoi(argv[3]);
    double frequency = std::atof(argv[4]);


    int n_nodes = K+1;
    double dt = 1/frequency;

    ros::NodeHandle nh;
    nh.getParam("/a", a);
    nh.getParam("/b", b);
    double temp_ka, temp_kr;
    nh.getParam("/ka", temp_ka);
    nh.getParam("/kr", temp_kr);
    nh.getParam("/KR", KR);

    ka = 1/temp_ka;
    kr = 1/temp_kr;

    std::cout<<a<<" "<<b<<" "<<ka<<" "<<kr<<" "<<KR<<std::endl;
    ros::Publisher control_pub, goal_pub, edge_pub, circle_pub;
    stringstream ss, ss1, ss2;
    ss << "/robot_"<<robot_id<<"/"<<"desired_state";
    ss1 << "/robot_"<<robot_id<<"/"<<"graph";
    ss2 << "/robot_"<<robot_id<<"/"<<"circle";

    control_pub = nh.advertise<geometry_msgs::Point>(ss.str(),frequency);
    edge_pub = nh.advertise<visualization_msgs::Marker>(ss1.str(),frequency);
    circle_pub = nh.advertise<visualization_msgs::Marker>(ss2.str(),frequency);
    if(robot_id == 1)
        goal_pub = nh.advertise<visualization_msgs::Marker>("/mrf_goal",frequency);


    double max_acc = 1;

    vector<Drone*> drones;
    vector<Eigen::Vector3d> U;

    // create the control space
    for(double dx=-max_acc; dx <= max_acc; dx+=0.5) {
        for(double dy=-max_acc; dy <= max_acc; dy+=0.5) {
        //    for(double dz=-1; dz <= 1; dz+=0.5) {
                Eigen::Vector3d u;
                u << dx, dy, 0;
                U.push_back(u);
        //    }
        }
    }

    // create a list of all drones
    for(int n=1; n <= n_robots; n++) {
        auto* drone = new Drone(n,nh);
        drones.push_back(drone);
    }

    Vector3d prev_u;
    ros::Rate loop_rate(frequency);
    int ros_it = 0;
    while(ros::ok()) {
        // get the state of my drone
        simulator_utils::Waypoint wp_i = drones[robot_id-1]->get_state();
        std::vector<simulator_utils::Waypoint> states;
        Eigen::Vector3d p_i{wp_i.position.x, wp_i.position.y, wp_i.position.z};

        // get the states of the all the drones to filter the closest neighbours
        std::vector<std::pair<int, double>> neighbour_dist;
        for(int i=1; i <= n_robots; i++) {
            simulator_utils::Waypoint wp_j = drones[i-1]->get_state();
            Eigen::Vector3d p_j{wp_j.position.x, wp_j.position.y, wp_j.position.z};
            double dist = (p_i-p_j).norm();
            states.push_back(wp_j);
            neighbour_dist.emplace_back(pair<int,double>(i,dist));
        }

        // sort the neighbour distances
        typedef std::function<bool(std::pair<int, double>, std::pair<int, double>)> Comparator;
        Comparator compFunctor =
                [](std::pair<int, double> elem1, std::pair<int, double> elem2) {
                    return elem1.second < elem2.second;
                };
        std::sort(neighbour_dist.begin(), neighbour_dist.end(), compFunctor);

        // create nodes for K nearest neighbours (K+1)
        list<int> unprocessed;
        vector<Node> nodes;
        vector<geometry_msgs::Point> neighbor_pos;
        neighbor_pos.push_back(drones[robot_id-1]->get_state().position);

        for(int k=0;k<n_nodes;k++) {
            // get the id of the closest neighbours
            int neighbour_id = neighbour_dist[k].first;
            // create label set for the neighbour
            vector<Label> label_set;
            for(auto & u : U) {
                Label l(u/2, states[neighbour_id-1], 1);
                if(abs(l.vt[0]) < 1 && abs(l.vt[1]) < 1) {
                    label_set.push_back(l);
                }
            }
            Node node(label_set, neighbour_id);
            neighbor_pos.push_back(states[neighbour_id-1].position);

            nodes.push_back(node);
            unprocessed.push_back(neighbour_id);
        }

        // draw the neighborhood graph
        if(robot_id == 1) {
            draw_edges(edge_pub,neighbor_pos, Vector3d(1,0,0));
            draw_circle(circle_pub, neighbor_pos[0], Vector3d(1,0,0));
            publish_goal(goal_pub);

        }
        else if(robot_id == 5) {
            draw_edges(edge_pub,neighbor_pos, Vector3d(0,1,0));
            draw_circle(circle_pub, neighbor_pos[0], Vector3d(0,1,0));

        }


        // do mean field approximation
        // initialize with unary potentials
        for(int i=0; i < n_nodes ; i++) {
            int id = nodes[i].id;
            Vector3d curr_pos = {states[id-1].position.x,states[id-1].position.y,states[id-1].position.z};

            double zi = 0;
            int M = nodes[i].X.size();
            for (int m = 0; m < M; m++) {
                zi += exp(-psi_u(nodes[i].X[m],curr_pos));
            }
            for (int m = 0; m < M; m++) {
                double Qi_tilde = (exp(-psi_u(nodes[i].X[m],curr_pos)));
                double qx = Qi_tilde / zi;
                nodes[i].Qx.push_back(qx);
            }
        }

        // auto start = chrono::high_resolution_clock::now();

        int it = 0;
        while (!unprocessed.empty()) {
            int id = *unprocessed.begin();
            unprocessed.pop_front();
            // get the node with current id cause the unprocessed list changes
            Node* n;
            for(auto & node : nodes) {
                if(node.id == id) {
                    n = &node;
                }
            }
            int M = n->X.size();
            n->z = 0;
            vector<double> Qi_old = n->Qx;
            n->Qx.clear();
            // compute the summation of expectations with respect to neighbours values ~Qi(xi)
            Vector3d curr_pos = {states[id-1].position.x,states[id-1].position.y,states[id-1].position.z};

            for (int m = 0; m < M; m++) {
                n->x = n->X[m];
                double psi_u_ = psi_u(n->X[m], curr_pos);
                double Qi_tilde_xi = Qi_tilde_(nodes, n, states);
                double Qi_hat = exp(-psi_u_ -Qi_tilde_xi);
                n->Qx.push_back(Qi_hat);
                n->z += Qi_hat;
            }

            for(int m=0;m<M;m++){
                n->Qx[m] = n->Qx[m]/n->z;
            }

            if (!dis_equal(Qi_old, n->Qx)) {
                unprocessed.push_back(n->id);
            }
            // logs for experiments
//            if(robot_id == 1 && ros_it != 0)
//                ROS_DEBUG_STREAM("KL: "<<kl_dis(Qi_old, n->Qx)<<" it: "<<it);
        it++;
        }
        // auto stop = chrono::high_resolution_clock::now();
        // auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
    //    ROS_DEBUG_STREAM(duration.count());

        if(ros_it != 0) {
            Node n = nodes[0];
            auto maxel = max_element(n.Qx.begin(), n.Qx.end());
            int idx = std::distance(n.Qx.begin(), maxel);
            Vector3d u = n.X[idx].u;
            geometry_msgs::Point msg;
            msg.x = u[0];
            msg.y = u[1];
            msg.z = u[2];
            control_pub.publish(msg);

        }
        ros_it++;
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}