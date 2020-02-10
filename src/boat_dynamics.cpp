#include "boat_dynamics/boat_dynamics.h"

namespace boat_dynamics {

BoatDynamics::BoatDynamics() : t_prev_(0.0), t_initialized_(false), nh_private_("~")
{
    nh_ = ros::NodeHandle();

    boat_height_m_ = 2.0;
//    boat_speed_mps_ = 0.5;
    boat_speed_mps_ = nh_private_.param<double>("boat_speed", 0.0);

    T_0_boat_ = Xformd((Vector3d() << 0.0, 0.0, boat_height_m_).finished(), Quatd::Identity());
    T_NED_0_ = Xformd((Vector3d() << 0.0, 0.0, 0.0).finished(), Quatd::from_euler(M_PI, 0.0, 0.0)).inverse();

    truth_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("boat_truth_NED", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("boat_marker", 1);

    transform_.header.frame_id = "world";
    transform_.child_frame_id = "boat";

    truth_.header.frame_id = "NED";

    marker_.header.frame_id = "NED";
    marker_.id = 1;
    marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.mesh_resource = "package://boat_dynamics/mesh/ship_nwu_GCS.dae";
    double boat_scale = 20.0;
    marker_.scale.x = boat_scale;
    marker_.scale.y = boat_scale;
    marker_.scale.z = boat_scale;
    marker_.color.r = 1.0;
    marker_.color.g = 1.0;
    marker_.color.b = 1.0;
    marker_.color.a = 1.0;

    ros::Time Rt = ros::Time::now();
    setMessageStates(Rt);

    timer_ = nh_.createTimer(ros::Duration(ros::Rate(20)), &BoatDynamics::onUpdate, this);
}

void BoatDynamics::onUpdate(const ros::TimerEvent &)
{
    ros::Time Rt = ros::Time::now();
    double t = Rt.toSec();

    if (!t_initialized_)
    {
        t_prev_ = t;
        t_initialized_ = true;
        return;
    }

    double dt = t - t_prev_;
    t_prev_ = t;

    // update boat state
    T_0_boat_.t_(0) += boat_speed_mps_ * dt;
    // ++++ update orientation using from_two_unit_vectors ++++

    // update and send messages
    setMessageStates(Rt);
    tbr_.sendTransform(transform_);
    truth_pub_.publish(truth_);
    marker_pub_.publish(marker_);
}

void BoatDynamics::setMessageStates(ros::Time &rt)
{
    transform_.header.stamp = rt;
    transform_.transform.translation.x = T_0_boat_.t_(0);
    transform_.transform.translation.y = T_0_boat_.t_(1);
    transform_.transform.translation.z = T_0_boat_.t_(2);
    transform_.transform.rotation.w = T_0_boat_.q_.w();
    transform_.transform.rotation.x = T_0_boat_.q_.x();
    transform_.transform.rotation.y = T_0_boat_.q_.y();
    transform_.transform.rotation.z = T_0_boat_.q_.z();

    Xformd T_NED_boat = T_NED_0_ * T_0_boat_;

    truth_.header.stamp = rt;
    truth_.pose.position.x = T_NED_boat.t_(0);
    truth_.pose.position.y = T_NED_boat.t_(1);
    truth_.pose.position.z = T_NED_boat.t_(2);
    truth_.pose.orientation.w = T_NED_boat.q_.w();
    truth_.pose.orientation.x = T_NED_boat.q_.x();
    truth_.pose.orientation.y = T_NED_boat.q_.y();
    truth_.pose.orientation.z = T_NED_boat.q_.z();

    marker_.header.stamp = rt;
    marker_.pose = truth_.pose;
}

} // end namespace boat_dynamics
