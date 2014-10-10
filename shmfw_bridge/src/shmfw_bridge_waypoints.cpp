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


#include <shmfw_bridge/shmfw_bridge.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

#include <shmfw/variable.h>
#include <shmfw/objects/pose.h>
#include <shmfw/objects/waypoint.h>
#include <shmfw/serialization/vector.h>

WayPoints::WayPoints()
    : frequency_ ( 1.0 )
    , shm_variable_name_ ( "waypoints" ) {

}


void WayPoints::initialize ( ros::NodeHandle &n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> &shm_handler) {

    n_param.getParam ( "shm_variable_name", shm_variable_name_ );
    ROS_INFO ( "%s/shm_variable_name: %s", n_param.getNamespace().c_str(), shm_handler->resolve_namespace(shm_variable_name_).c_str() );

    n_param.getParam ( "frequency", frequency_ );
    ROS_INFO ( "%s/frequency: %5.2f, -1 means on update only", n_param.getNamespace().c_str(), frequency_ );


    shm_waypoints_ = boost::shared_ptr<ShmFw::Vector<ShmFw::WayPoint> > ( new ShmFw::Vector<ShmFw::WayPoint> ( shm_variable_name_, shm_handler ) );

    pub_waypoints_ = n.advertise<geometry_msgs::PoseArray> ( shm_variable_name_, 1 );
    thread_ = boost::thread ( boost::bind ( &WayPoints::update, this ) );

}

void WayPoints::update() {
    ros::Rate rate ( frequency_ );
    int timeout = 1000.0/frequency_;
    int timeout_count = 0;
    std::vector<ShmFw::WayPoint> points;
    points.resize ( 10 );
    geometry_msgs::PoseArray waypoints;
    while ( ros::ok() ) {
        bool read = false;
        if ( frequency_ < 0 ) {
            shm_waypoints_->wait();
            read = true;
        } else {
            shm_waypoints_->timed_wait ( timeout );
            read = true;
        }
        if ( read ) {
            timeout_count = 0;
            shm_waypoints_->get ( points );
            waypoints.header.frame_id = "map";
            waypoints.header.stamp = ros::Time::now();
            waypoints.poses.resize ( points.size() );
            for ( size_t i = 0; i < points.size(); i++ ) {
                points[i].pose.copyTo ( waypoints.poses[i] );
            }
            if ( points.size() > 0 ) {
                pub_waypoints_.publish ( waypoints );
            }
        } else {
            ROS_INFO ( "Command::update_motion timeout: %i", timeout_count );
            timeout_count++;
        }
        rate.sleep();
    }
}

