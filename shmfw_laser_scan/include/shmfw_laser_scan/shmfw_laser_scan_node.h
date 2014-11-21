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


#ifndef ROS_SHMFW_LASER_SCAN_H
#define ROS_SHMFW_LASER_SCAN_H


#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <boost/thread.hpp>
#include <shmfw/objects/ros/header.h>
#include <shmfw/objects/ros/laser_scan.h>
#include <shmfw/variable.h>
#include <shmfw/allocator.h>


class ShmFwLaserScanNode {
  typedef boost::shared_ptr< ShmFw::Alloc<ShmFw::ros::LaserScan> > ShmLaserScanPtr;
public:
    ShmFwLaserScanNode ();
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    std::string shm_segment_name_;
    int shm_segment_size_;
    std::string shm_laser_name_;
    bool shm_unlook_;
    ros::Subscriber sub_laser_scan_;
    ShmLaserScanPtr laser_scan_;
private:
   void read_parameter();
    void callbackLaserScan(const sensor_msgs::LaserScanPtr& msg);
};
#endif //ROS_SHMFW_LASER_SCAN_H