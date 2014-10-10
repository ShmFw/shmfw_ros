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


#ifndef ROS_SHMFW_VIZUALIZATION_MARKER_NODE_H
#define ROS_SHMFW_VIZUALIZATION_MARKER_NODE_H

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <shmfw/objects/ros/header.h>
#include <shmfw/objects/ros/visualization_marker.h>
#include <shmfw/variable.h>
#include <shmfw/allocator.h>


class ShmFwVisualizationMarker {
  typedef boost::shared_ptr< ShmFw::Alloc<ShmFw::ros::VisualizationMarker> > ShmVisualizationMarkerPtr;
public:
    ShmFwVisualizationMarker ();
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    double frequency_;
    std::string shm_segment_name_;
    int shm_segment_size_;
    std::string  shm_variable_name_;
    std::vector<std::string>  shm_variable_names_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_markers_;
    visualization_msgs::Marker ros_visualization_marker_;
    visualization_msgs::MarkerArray ros_visualization_markers_;
    ShmVisualizationMarkerPtr shm_visualization_marker_;
    std::vector<ShmVisualizationMarkerPtr> shm_visualization_markers_;
    unsigned int timeout_count_;
private:
    void publish_marker();
    void publish_markers();
    void read_parameter();
};



#endif //ROS_SHMFW_VIZUALIZATION_MARKER_NODE_H
