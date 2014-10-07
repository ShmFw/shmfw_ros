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
#include <visualization_msgs/Marker.h>

#include <shmfw/vector.h>
#include <shmfw/objects/marker.h>

Marker::Marker()
    : frequency_ ( 1.0 )
    , shm_varible_postfix_ ( "marker" ) {

}


void Marker::initialize ( ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info ) {

    n_param.getParam ( "shm_varible_postfix", shm_varible_postfix_ );
    ROS_INFO ( "%s/shm_varible_postfix: %s", n_param.getNamespace().c_str(), shm_varible_postfix_.c_str() );

    sprintf ( shm_varible_name_, "agv%03d_%s", agv_info.id, shm_varible_postfix_.c_str() );
    ROS_INFO ( "%s/shm_varible_name: %s", n_param.getNamespace().c_str(), shm_varible_name_ );

    n_param.getParam ( "frequency", frequency_ );
    ROS_INFO ( "%s/frequency: %5.2f, -1 means on update only", n_param.getNamespace().c_str(), frequency_ );


    shm_marker_ = boost::shared_ptr<ShmFw::Vector<ShmFw::Marker> > ( new ShmFw::Vector<ShmFw::Marker> ( shm_varible_name_, shm_handler ) );

    pub_marker_ = n.advertise<visualization_msgs::Marker> ( "visualization_marker", 10 );
    thread_ = boost::thread ( boost::bind ( &Marker::update, this ) );
}

void Marker::drawMarker ( const ShmFw::Marker &marker ) {

    visualization_msgs::Marker msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.ns = marker.getNS();
    msg.action = marker.action;
    msg.pose.orientation.w = 1.0;
    msg.id = marker.id;
    msg.type = marker.type;
    if(marker.type == ShmFw::Marker::MARKER_ARROW){
      marker.pose.copyTo(msg.pose);
      marker.scale.copyTo(msg.scale);
    }
    marker.color.copyTo(msg.color);
    msg.lifetime = ros::Duration(marker.lifetime);
    
    pub_marker_.publish ( msg );
}

void Marker::update() {
    ros::Rate rate ( frequency_ );
    int timeout = 1000.0/frequency_;
    int timeout_count = 0;
    std::vector<ShmFw::Marker> marker;
    while ( ros::ok() ) {
        bool read = false;
        if ( frequency_ < 0 ) {
            shm_marker_->wait();
            read = true;
        } else {
            shm_marker_->timed_wait ( timeout );
            read = true;
        }
        if ( read ) {
            timeout_count = 0;
            shm_marker_->get ( marker );
            if ( pub_marker_.getNumSubscribers() > 0 ) {
                for ( size_t i = 0; i < marker.size(); i++ ) {
                    drawMarker ( marker[i] );
                }
            }
        } else {
            ROS_INFO ( "Maker::update timeout: %i", timeout_count );
            timeout_count++;
        }
        rate.sleep();
    }
}

