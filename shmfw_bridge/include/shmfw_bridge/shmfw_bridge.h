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
#include <shmfw/forward_declarations.h>

namespace visualization_msgs{
  template <class ContainerAllocator> struct Marker_;
  typedef ::visualization_msgs::Marker_<std::allocator<void> > Marker;
}

namespace gazebo_msgs{
  template <class ContainerAllocator> struct ModelStates_;
typedef boost::shared_ptr< ::gazebo_msgs::ModelStates > ModelStatesPtr;
typedef boost::shared_ptr< ::gazebo_msgs::ModelStates const> ModelStatesConstPtr;
}


struct AGVInfo{
  AGVInfo()
  : id(15){    
  }
  int id;
};
class Command {
public:
    Command();
    void initialize(ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info);
    double frequency_;
    char shm_varible_name_[0xFF];
    std::string shm_varible_postfix_;
    boost::shared_ptr<ShmFw::Var<ShmFw::Twist> > cmd_;
    boost::thread thread_;
    ros::Publisher pub_;
    void update();
};


class WayPoints {
public:
    WayPoints();
    void initialize(ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info);
    char shm_varible_name_[0xFF];
    std::string shm_varible_postfix_;
    boost::shared_ptr<ShmFw::Vector<ShmFw::WayPoint> > waypoints_;
    double frequency_;
    boost::thread thread_;
    ros::Publisher pub_waypoints_;
    void update();
};


class Segments {
public:
    Segments();
    void initialize(ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info);
    char shm_varible_name_[0xFF];
    std::string shm_varible_postfix_;
    boost::shared_ptr<ShmFw::Deque<ShmFw::RouteSegment> > segments_ahead_;
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
    void initialize(ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info);
    std::string target_frame_;
    std::string source_frame_;
    char shm_varible_name_[0xFF];
    std::string shm_varible_postfix_;
    std::string tf_prefix_;
    boost::shared_ptr<ShmFw::Var<ShmFw::Pose2DAGV> > pose_;
    tf::TransformListener listener_;
    ros::Publisher pub_;
    void update();
};

class Gazebo {
public:
    Gazebo();
    void initialize(ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> &shm_handler, const AGVInfo &agv_info);
    std::string target_frame_;
    std::string shm_name_pose_gt_;
    std::string gazebo_model_name_;
    boost::shared_ptr<ShmFw::Var<ShmFw::Pose2DAGV> > pose_;
    boost::thread thread_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    geometry_msgs::PoseStamped msg_;
    void update(boost::shared_ptr<ShmFw::Handler> &shm_handler);
    void callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
};

class ShmFwBridge {
public:
    ShmFwBridge ();
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    double frequency_;
    AGVInfo agv_info_;
    std::string shm_segment_name_;
    int shm_segment_size_;
    boost::shared_ptr<ShmFw::Handler> shm_handler_;
    boost::shared_ptr<Pose> pose_;
    boost::shared_ptr<Command> command_;
    boost::shared_ptr<Segments> segments_;
    boost::shared_ptr<WayPoints> waypoints_;
    boost::shared_ptr<Gazebo> gazebo_;
    void read_parameter();
};



#endif //ROS_SHMFW_BRIDGE_H
