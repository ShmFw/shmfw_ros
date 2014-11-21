/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2014 by Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/


#ifndef ROS_SHMFW_BRIDGE_H
#define ROS_SHMFW_BRIDGE_H

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/UInt64.h>
#include <shmfw/forward_declarations.h>
#include <shmfw/allocator.h>
#include <shmfw/objects/agent_state.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace visualization_msgs{
  template <class ContainerAllocator> struct Marker_;
  typedef ::visualization_msgs::Marker_<std::allocator<void> > Marker;
}

namespace gazebo_msgs{
  template <class ContainerAllocator> struct ModelStates_;
typedef boost::shared_ptr< ::gazebo_msgs::ModelStates > ModelStatesPtr;
typedef boost::shared_ptr< ::gazebo_msgs::ModelStates const> ModelStatesConstPtr;
}

class Command {
public:
    Command();
    void initialize(ros::NodeHandle &n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> &shm_handler);
    double frequency_;
    double timeout_signal_;  ///onyl used if frequency_ < 0
    std::string shm_name_cmd_;
    boost::shared_ptr<ShmFw::Var<ShmFw::Twist> > shm_cmd_;
    boost::thread thread_;
    ros::Publisher pub_;
    void update();
};


class WayPoints {
public:
    WayPoints();
    void initialize(ros::NodeHandle &n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> &shm_handler);
    std::string shm_variable_name_;
    boost::shared_ptr<ShmFw::Vector<ShmFw::ModelState> > shm_waypoints_;
    double frequency_;
    boost::thread thread_;
    ros::Publisher pub_waypoints_;
    void update();
};


class Segments {
public:
    Segments();
    void initialize(ros::NodeHandle &n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> &shm_handler);
    std::string shm_variable_name_;
    boost::shared_ptr<ShmFw::Deque<ShmFw::RouteSegment> > shm_segments_ahead_;
    std::vector<ShmFw::Pose> &convertRoute2Path ( const std::vector<ShmFw::RouteSegment> &segments, std::vector<ShmFw::Pose> &path, double angle_resolution );
    void drawMarker ( const std::vector<ShmFw::RouteSegment> &segments );
    double frequency_;
    double angle_resolution_;
    boost::thread thread_;
    ros::Publisher pub_path_;
    ros::Publisher pub_waypoints_;
    ros::Publisher pub_marker_;
    boost::shared_ptr<visualization_msgs::Marker > marker_lines_;
    boost::shared_ptr<visualization_msgs::Marker > marker_text_;
    boost::shared_ptr<visualization_msgs::Marker > marker_end_;
    void update();
    void initMarker();
};

class Pose {
public:
    Pose();
    void initialize(ros::NodeHandle &n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> &shm_handler);
    std::string target_frame_;
    std::string source_frame_;
    std::string shm_name_pose_;
    std::string shm_name_agent_state_;
    double frequency_;
    std::string tf_prefix_;
    double trip_recorder_scale_;
    boost::shared_ptr<ShmFw::Var<ShmFw::Pose2DAGV> > shm_pose_;
    boost::shared_ptr<ShmFw::Var<ShmFw::AgentState> > shm_agent_state_;
    boost::thread thread_;
    tf::TransformListener listener_;
    ros::Subscriber sub_trip_recorder_;
    ros::Subscriber sub_command_current_;
    ros::Subscriber sub_command_target_;
    ros::Publisher pub_;
    void update();
    ShmFw::AgentState agent_state_;
    void callbackTripRecorder(const std_msgs::UInt64Ptr& trip_recorder);
    void callbackCmdCurrent(const geometry_msgs::TwistPtr& command_current);
    void callbackCmdTarget(const geometry_msgs::TwistPtr& command_target);
};

class Gazebo {
public:
    Gazebo();
    void initialize(ros::NodeHandle &n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> &shm_handler);
    std::string target_frame_;
    std::string shm_name_pose_;
    std::string shm_name_state_;
    std::string shm_name_init_state_;
    std::string shm_name_sim_time_;
    std::string gazebo_model_name_;
    boost::shared_ptr<ShmFw::Var<boost::posix_time::ptime> > shm_time_;
    boost::shared_ptr<ShmFw::Var<ShmFw::Pose2DAGV> > shm_pose_;
    boost::shared_ptr<ShmFw::Var<ShmFw::ModelState> > shm_state_;
    boost::shared_ptr<ShmFw::Var<ShmFw::ModelState> > shm_init_state_;
    boost::thread thread_;
    ros::Publisher pub_;
    ros::ServiceClient service_get_model_;
    ros::Publisher pub_set_model_;
    geometry_msgs::PoseStamped msg_;
    double frequency_;
    void update();
};

class ShmFwBridge {
public:
    ShmFwBridge ();
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    double frequency_;
    std::string shm_segment_name_;
    int shm_segment_size_;
    boost::shared_ptr<ShmFw::Handler> shm_handler_;
    bool bridge_pose_;
    boost::shared_ptr<Pose> pose_;
    bool bridge_command_;
    boost::shared_ptr<Command> command_;
    bool bridge_segments_;
    boost::shared_ptr<Segments> segments_;
    bool bridge_waypoints_;
    boost::shared_ptr<WayPoints> waypoints_;
    bool bridge_gazebo_;
    boost::shared_ptr<Gazebo> gazebo_;
private:
    void read_parameter();
};



#endif //ROS_SHMFW_BRIDGE_H
