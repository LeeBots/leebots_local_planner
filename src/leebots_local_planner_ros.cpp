#include "leebots_local_planner/leebots_local_planner_ros.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(leebots_local_planner::LeebotsPlannerROS, nav_core::BaseLocalPlanner)

namespace leebots_local_planner{

LeebotsPlannerROS::LeebotsPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

LeebotsPlannerROS::LeebotsPlannerROS(std::string name, tf2_ros::Buffer* tf,
                           costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
    initialize(name, tf, costmap_ros);
}

LeebotsPlannerROS::~LeebotsPlannerROS() {}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void LeebotsPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                              costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        initialized_ = true;
    }
}

bool LeebotsPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& orig_global_plan
)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return true;
}

bool LeebotsPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return true;
}

bool LeebotsPlannerROS::isGoalReached()
{
    if(!initialized_)
    {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    return false;
}
}
