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

namespace visualization_msgs{
  template <class ContainerAllocator> struct Marker_;
  typedef ::visualization_msgs::Marker_<std::allocator<void> > Marker;
}

namespace ShmFw{
  template <class> class Var;
  template <class> class Deque;
  class Pose;
  class Pose2DAGV;
  class Velocity;
  class SegmentAGV;
  class Handler;
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
    std::string shm_varible_name_;
    boost::shared_ptr<ShmFw::Var<ShmFw::Velocity> > cmd_;
    boost::thread thread_;
    ros::Publisher pub_;
    void update();
    void exit();
    bool loop;
};

class Path {
public:
    Path();
    void initialize(ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info);
    boost::shared_ptr<ShmFw::Deque<ShmFw::SegmentAGV> > segments_ahead_;
    double frequency_;
    double resolution_;
    double scale_;
    boost::thread thread_;
    ros::Publisher pub_path_;
    ros::Publisher pub_waypoints_;
    ros::Publisher pub_marker_;
    boost::shared_ptr<visualization_msgs::Marker > segment_arrow_;
    boost::shared_ptr<visualization_msgs::Marker > points_start_;
    boost::shared_ptr<visualization_msgs::Marker > points_end_;
    void update();
    void initMarker();
    void exit();
    bool loop;
    std::vector<ShmFw::Pose> getWayPoints(const std::vector<ShmFw::SegmentAGV> &segments, std::vector<ShmFw::Pose> &path);
};

class Pose {
public:
    Pose();
    void initialize(ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info);
    std::string target_frame_;
    std::string source_frame_;
    std::string shm_varible_name_;
    std::string tf_prefix_;
    boost::shared_ptr<ShmFw::Var<ShmFw::Pose2DAGV> > pose_;
    tf::TransformListener listener_;
    ros::Publisher pub_;
    void update();
};

class ShmTFNode {
public:
    ShmTFNode ();
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
    boost::shared_ptr<Path> path_;
    void read_parameter();
};



#endif //ROS_SHMFW_BRIDGE_H
