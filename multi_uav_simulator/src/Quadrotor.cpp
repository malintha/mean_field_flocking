//
// Created by malintha on 8/22/18.
//
#include "Quadrotor.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include "simulator_utils/simulator_utils.h"


Quadrotor::Quadrotor(int robot_id, double frequency, ros::NodeHandle &n)
        : robot_id(robot_id), frequency(frequency), nh(n) {
    sim_time = 0;
    this->dt = 1/frequency;
    this->initialize(1 / frequency);
    this->u << 0,0,0;
}

bool Quadrotor::initialize(double dt_) {
    m_state = State::Idle;
    // load quad params
    if (!load_params()) {
        ROS_ERROR_STREAM("Could not load the drone parameters");
        return false;
    }
    // load init params
    if (!load_init_vals()) {
        ROS_ERROR_STREAM("Could not load the drone initial values");
        return false;
    }

    stringstream ss;
    ss << localframe << to_string(robot_id);
    robot_link_name = ss.str();

    // set initial values
    state_space.position = init_vals.position;
    state_space.velocity = init_vals.velocity;
    state_space.R = init_vals.R;
    state_space.omega = init_vals.omega;

    dynamics = new DynamicsProvider(params, init_vals);
    controller = new ControllerImpl(params, init_vals, gains, dt_);

    this->setState(State::Autonomous);
    initPaths();

    desired_state_sub = nh.subscribe("desired_state", 10, &Quadrotor::desired_acc_cb, this);
    state_pub = nh.advertise<simulator_utils::Waypoint>("current_state", 10);
    ROS_DEBUG_STREAM("Drone initialized " << robot_id);
    ROS_INFO_STREAM("Desired state subscriber topic: " << "robot_"<<robot_id<<"/desired_state");
    ROS_INFO_STREAM("State publisher topic: " << "robot_"<<robot_id<<"/current_state");
    while (ctrl_pub.getNumSubscribers() < 1) {
        ROS_INFO_STREAM("Waiting for subscriber: "<<robot_id);
        ros::Duration(1).sleep();
    }
    this->set_init_target = false;
    return true;
}

void Quadrotor::initPaths() {
    marker_pub = nh.advertise<visualization_msgs::Marker>(
            ros::names::append(robot_link_name, "path"), 10);

    goal_pub = nh.advertise<visualization_msgs::Marker>(
            ros::names::append(robot_link_name, "goal"), 10);

    ctrl_pub = nh.advertise<visualization_msgs::Marker>(
            ros::names::append(robot_link_name, "control"), 10);

    vel_pub = nh.advertise<visualization_msgs::Marker>(
            ros::names::append(robot_link_name, "vel"), 10);

    g.header.stamp = m.header.stamp = ctrl.header.stamp = vel.header.stamp = ros::Time::now();
    m.type = visualization_msgs::Marker::LINE_STRIP;
    g.header.frame_id = m.header.frame_id = ctrl.header.frame_id = vel.header.frame_id = worldframe;
    m.action = visualization_msgs::Marker::ADD;
    m.id = this->robot_id;
    m.color.r = 0.5;
    m.color.g = 0.5;
    m.color.b = 0.5;
    m.color.a = 0.7;
    m.scale.x = 0.02;
    m.pose.orientation.w = 1.0;
    g.id = this->robot_id + 20;
    g.type = visualization_msgs::Marker::CUBE;
    g.action = visualization_msgs::Marker::ADD;
    g.color.r = 1;
    g.color.g = 0;
    g.color.b = 0;
    g.color.a = 1;
    g.scale.x = 0.1;
    g.scale.y = 0.1;
    g.scale.z = 0.1;

    ctrl.id = this->robot_id+30;
    ctrl.type = visualization_msgs::Marker::ARROW;
    ctrl.action = visualization_msgs::Marker::ADD;
    ctrl.color.r = 0;
    ctrl.color.g = 0;
    ctrl.color.b = 0;
    ctrl.color.a = 1;
    ctrl.scale.x = 0.05;
    ctrl.scale.y = 0.05;
    ctrl.pose.orientation.w = 1.0;

    vel.id = this->robot_id+40;
    vel.type = visualization_msgs::Marker::ARROW;
    vel.action = visualization_msgs::Marker::ADD;
    vel.color.r = 0;
    vel.color.g = 1;
    vel.color.b = 1;
    vel.color.a = 1;
    vel.scale.x = 0.05;
    vel.scale.y = 0.05;
    vel.pose.orientation.w = 1.0;
}

void Quadrotor::setState(State m_state_) {
    this->m_state = m_state_;
}

void Quadrotor::move(const desired_state_t &d_state) {
    control_out_t control = controller->get_control(dynamics->get_state(), d_state);
    dynamics->update(control, sim_time);

    // updating the model on rviz
    set_state_space();
    send_transform();
    dynamics->reset_dynamics();
    sim_time += dt;
}

/**
 * converts the dynamics state_space (NED) to NWU and stores for the quadrotor
 * This is for broadcasting the transformation
*/
void Quadrotor::set_state_space() {
    state_space_t ss = dynamics->get_state();
    state_space.position = simulator_utils::ned_nwu_rotation(ss.position);
    state_space.R = simulator_utils::ned_nwu_rotation(ss.R);
    state_space.velocity = simulator_utils::ned_nwu_rotation(ss.velocity);
    state_space.omega = simulator_utils::ned_nwu_rotation(ss.omega);
}

void Quadrotor::send_transform() {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    Vector3d position = state_space.position;
    Matrix3d R = state_space.R;
    Vector3d rpy = simulator_utils::R2RPY(R);
    transform.setOrigin(tf::Vector3(position[0], position[1], -position[2]));
    tf::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    transform.setRotation(q);
    if(quad_initialized && !isnan(q.x()))
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), worldframe, robot_link_name));
}

State Quadrotor::getState() {
    return this->m_state;
}

bool Quadrotor::load_params() {
    double temp;
    if (!nh.getParam("/uav/gravity", temp)) return false;
    params.gravity = temp;
    vector<double> J_;
    if (!nh.getParam("/uav/J", J_)) return false;
    Matrix3d J;
    J <<    J_[0], 0, 0,
            0, J_[1], 0,
            0, 0, J_[2];
    params.J = J;
    params.J_inv = J.inverse();
    if (!nh.getParam("/uav/m", temp)) return false;
    params.mass = temp;
    params.F = 0;
    Vector3d M(0, 0, 0);
    params.M = M;
    vector<double> gains_;
    if (!nh.getParam("/controller/gains", gains_)) return false;
    this->gains = {gains_[0], gains_[1], gains_[2], gains_[3]};

    ROS_DEBUG_STREAM("Loaded control parameters");
    return true;
}

bool Quadrotor::load_init_vals() {
    vector<double> position, vel, R, omega;
    stringstream ss;
    ss << "/robot_" << to_string(this->robot_id);
    string robot_name = ss.str();
    if (!nh.getParam(ros::names::append(robot_name, "position"), position)) return false;
    if (!nh.getParam(ros::names::append(robot_name, "velocity"), vel)) return false;
    if (!nh.getParam(ros::names::append(robot_name, "rotation"), R)) return false;
    if (!nh.getParam(ros::names::append(robot_name, "omega"), omega)) return false;
    if (!nh.getParam("/frame/fixed", worldframe)) return false;
    if (!nh.getParam("drone/frame/prefix", localframe)) return false;
    init_vals.position = Vector3d(position.data());
    init_vals.velocity = Vector3d(vel.data());
    init_vals.R = Matrix3d(R.data());
    init_vals.omega = Vector3d(omega.data());

    this->x0.position = simulator_utils::ned_nwu_rotation(init_vals.position);
    this->x0.velocity = simulator_utils::ned_nwu_rotation(init_vals.velocity);

    this->xd0.position = this->x0.position;


    ROS_DEBUG_STREAM("Loaded the drone initialization values");
    return true;
}

void Quadrotor::write_to_file() {
    std::ofstream out_pos, out_vel, out_acc;
    stringstream ss_p, ss_v, ss_a;
    ss_p << "/home/malintha/mrf_ws/src/multi_uav_simulator/data/"<<robot_id<<"_pos.txt";
    ss_v << "/home/malintha/mrf_ws/src/multi_uav_simulator/data/"<<robot_id<<"_vel.txt";
    ss_a << "/home/malintha/mrf_ws/src/multi_uav_simulator/data/"<<robot_id<<"_acc.txt";

    out_pos.open(ss_p.str(), std::ios_base::app);
    out_vel.open(ss_v.str(), std::ios_base::app);
    out_acc.open(ss_a.str(), std::ios_base::app);
//    if (!out_pos)
//        std::cerr << "Cannot open \"file\"!" << std::endl;

    for (int i = 0; i < positions.size(); i++) {
        out_pos << std::endl << positions[i][0] << " " << positions[i][1] << " " << 1;
    }
    for (int i = 0; i < velocities.size(); i++) {
        out_vel << std::endl << velocities[i][0] << " " << velocities[i][1] << " " << 0;
    }
    for (int i = 0; i < accelerations.size(); i++) {
        out_acc << std::endl << accelerations[i][0] << " " << accelerations[i][1] << " " << 0;
    }

    out_pos.close();
    out_vel.close();
    out_acc.close();

}

void Quadrotor::desired_acc_cb(const geometry_msgs::PointConstPtr &pt) {
    this->x0.position = this->dynamics->get_state().position;
    this->x0.velocity = this->dynamics->get_state().velocity;
    this->x0.acceleration = this->dynamics->get_state().acceleration;
    
    this->u << pt->x, pt->y, pt->z;
    this->u = (this->u);
    this->tau = 0;

    this->xd0.position = this->x0.position;
    this->xd0.velocity = this->x0.velocity;

}
// xd should be in NED frame and so does dynamics and controller.
void Quadrotor::iteration(const ros::TimerEvent &e) {
    // compute the next desired state using incoming control signal and current state
    Vector3d b1d(1, 0, 0);

    // integrate the desired control with the current state to get the desired position for the time step 
    xd0.position = xd0.position + xd0.velocity*dt + 0.5 * u * pow(dt, 2);
    // Vector3d vd =  x0.velocity + u * dt;
    xd0.velocity = xd0.velocity + u * dt;
    xd[0] = xd0.position[0];
    xd[1] = xd0.position[1];
    xd[2] = 0;


    desired_state_t dss = {xd, b1d};
    this->move(dss);
    this->publish_path();
    this->publish_state();
    // this->positions.push_back(xd);
    // this->velocities.push_back(vd);
    // this->accelerations.push_back(u);
    tau = tau + dt;

}

void Quadrotor::publish_path() {
    Vector3d x = this->dynamics->get_state().position;
    x[1] = -x[1];
    geometry_msgs::Point p;
    if(!isnan(x[0]) && !isnan(x[1])) {
        p.x = x[0];
        p.y = x[1];
        p.z = x[2];
        quad_initialized = true;
    }
    else
    {
        p.x = 0;
        p.y = 0;
        p.z = 0;
    }

    m.points.push_back(p);

    if (m.points.size() > 500)
        m.points.erase(m.points.begin());

    g.pose.position.x = this->target_pos[0];
    g.pose.position.y = -this->target_pos[1];
    g.pose.position.z = this->target_pos[2];
    g.pose.orientation.x = 0.0;
    g.pose.orientation.y = 0.0;
    g.pose.orientation.z = 0.0;
    g.pose.orientation.w = 1.0;

    // control arrow start position
    ctrl.points.clear();
    ctrl.points.push_back(p);
    // arrow end position. control input scaled and translated to start position
    geometry_msgs::Point control;
    control.x = (2*u[0] + p.x);
    control.y = (-2*u[1] + p.y);
    control.z = (2*u[2] + p.z);
    ctrl.points.push_back(control);

    // for velocity marker
    Vector3d v = this->dynamics->get_state().velocity;
    v[1] = -v[1];
    geometry_msgs::Point vp;
    vp.x = 2*v[0] + p.x;
    vp.y = 2*v[1] + p.y;
    vp.z = 2*v[2] + p.z;

    vel.points.clear();
    vel.points.push_back(p);
    vel.points.push_back(vp);

//    goal_pub.publish(g);
    marker_pub.publish(m);
    ctrl_pub.publish(ctrl);
    vel_pub.publish(vel);
}

void Quadrotor::run() {
    ros::Timer timer = nh.createTimer(ros::Duration(1 / frequency), &Quadrotor::iteration, this);
    ros::spin();
}

void Quadrotor::publish_state() {
    simulator_utils::Waypoint wp;
    geometry_msgs::Point p,v;
    state_space_t ned_state = this->dynamics->get_state();
    p.x = ned_state.position[0];
    p.y = ned_state.position[1];
    p.z = ned_state.position[2];
    v.x = ned_state.velocity[0];
    v.y = ned_state.velocity[1];
    v.z = ned_state.velocity[2];
    wp.position = p;
    wp.velocity = v;
    wp.acceleration.x = wp.acceleration.y = wp.acceleration.z = 0;
    this->state_pub.publish(wp);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "~");
    ros::NodeHandle n;
    int robot_id;
    n.getParam("drone/robot_id", robot_id);

    ROS_DEBUG_STREAM("initializing : " << robot_id << endl);
    stringstream ss;
    double frequency = 100;

    Quadrotor quad(robot_id, frequency, n);
    quad.run();

    return 0;
}