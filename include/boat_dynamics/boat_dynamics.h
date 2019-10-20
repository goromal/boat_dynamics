#include <ros/ros.h>
#include "utils/xform.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>

using namespace transforms;
using namespace Eigen;

namespace boat_dynamics {

class BoatDynamics
{
public:
    BoatDynamics();

private:
    void onUpdate(const ros::TimerEvent &event);
    void setMessageStates(ros::Time &rt);

    ros::NodeHandle nh_;
    ros::Timer timer_;

    double boat_height_m_;
    double boat_speed_mps_;
    double t_prev_;
    bool t_initialized_;

    Xformd T_0_boat_;
    Xformd T_NED_0_;

    tf2_ros::TransformBroadcaster tbr_;
    ros::Publisher truth_pub_;
    ros::Publisher marker_pub_;

    geometry_msgs::TransformStamped transform_;
    geometry_msgs::PoseStamped truth_;
    visualization_msgs::Marker marker_;
};

} // end namespace boat_dynamics
