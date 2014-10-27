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

#include <shmfw_visualization_marker_node.h>

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "shmfw_bridge" );
    ShmFwVisualizationMarker node;
    return 0;
}

ShmFwVisualizationMarker::ShmFwVisualizationMarker ()
    : n_ ()
    , n_param_ ( "~" )
    , frequency_ ( 10.0 )
    , shm_segment_name_ ( ShmFw::DEFAULT_SEGMENT_NAME() )
    , shm_segment_size_ ( ShmFw::DEFAULT_SEGMENT_SIZE() )
    , shm_marker_name_ ( "VisualizationMarker" )
    , shm_marker_name_array_ ( "VisualizationMarkerArray" )
    , timeout_count_ ( 0 ) {

    read_parameter();

    pub_marker_ = n_.advertise<visualization_msgs::Marker> ( "visualization_marker", 10 );
    pub_marker_array_ = n_.advertise<visualization_msgs::MarkerArray> ( "visualization_marker_array", 10 );

    thread_visualization_marker_ = boost::thread ( boost::bind ( &ShmFwVisualizationMarker::publish_marker, this ) );    
    thread_visualization_marker_array_ = boost::thread ( boost::bind ( &ShmFwVisualizationMarker::publish_marker_array, this ) );
    while( ros::ok()){
      sleep(1);
    }
}

void ShmFwVisualizationMarker::read_parameter() {
    std::stringstream ss;
    ROS_INFO ( "namespace: %s", n_param_.getNamespace().c_str() );

    n_param_.getParam ( "frequency", frequency_ );
    ROS_INFO ( "frequency: %5.2f", frequency_ );

    n_param_.getParam ( "shm_segment_name", shm_segment_name_ );
    ROS_INFO ( "shm_segment_name: %s", shm_segment_name_.c_str() );

    n_param_.getParam ( "shm_segment_size", shm_segment_size_ );
    ROS_INFO ( "shm_segment_size: %d", shm_segment_size_ );

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shm_segment_name_, shm_segment_size_ );
    shmHdl->setNamespace ( n_.getNamespace() );
    
    n_param_.getParam ( "shm_marker_name", shm_marker_name_ );
    ROS_INFO ( "%s/shm_marker_name: %s", n_param_.getNamespace().c_str(), shmHdl->resolve_namespace(shm_marker_name_).c_str() );

    n_param_.getParam ( "shm_marker_name_array", shm_marker_name_array_ );
    ROS_INFO ( "%s/shm_marker_name_array: %s", n_param_.getNamespace().c_str(), shmHdl->resolve_namespace(shm_marker_name_array_).c_str() );

    shm_visualization_marker_.reset ( new ShmFw::Alloc<ShmFw::ros::VisualizationMarker> ( shm_marker_name_, shmHdl ) );
    shm_visualization_marker_array_.reset ( new ShmFw::Alloc<ShmFw::ros::VisualizationMarkerArray> ( shm_marker_name_array_, shmHdl ) );
}

void ShmFwVisualizationMarker::publish_marker() {

    ros::Rate rate ( frequency_ );
    while ( ros::ok() ) {
        if ( pub_marker_.getNumSubscribers() == 0 ) {
	  sleep(1);
	  continue;
	}
        bool read = false;
        if ( frequency_ > 0 ) {
            shm_visualization_marker_->timed_wait ( 1.0/frequency_ );
            read = true;
        } else {
            shm_visualization_marker_->wait();
            read = true;
        }
        if ( read ) {
            if ( shm_visualization_marker_->get()->type == ShmFw::ros::VisualizationMarker::MARKER_NA ) {
                ROS_INFO ( "Marker type no set for: %s", shm_visualization_marker_->name().c_str() );
            } else {
                timeout_count_ = 0;
                shm_visualization_marker_->lock();
                shm_visualization_marker_->get()->copyTo ( ros_visualization_marker_ );
                shm_visualization_marker_->unlock();
                pub_marker_.publish ( ros_visualization_marker_ );
            }
        } else {
            ROS_INFO ( "ShmFwVisualizationMarker::publish_marker timeout: %i", timeout_count_ );
            timeout_count_++;
        }
        if ( frequency_ > 0 )  rate.sleep();
	else  usleep(100);
    }
}

void ShmFwVisualizationMarker::publish_marker_array() {
    ros::Rate rate ( frequency_ );
    while ( ros::ok() ) {
        if ( pub_marker_array_.getNumSubscribers() == 0 ) {
	  sleep(1);
	  continue;
	}
        bool read = false;
        if ( frequency_ > 0 ) {
            shm_visualization_marker_array_->timed_wait ( 1.0/frequency_ );
            read = true;
        } else {
            shm_visualization_marker_array_->wait();
            read = true;
        }
        if ( read ) {
            timeout_count_ = 0;
            shm_visualization_marker_array_->lock();
            ros_visualization_marker_array_.markers.resize ( shm_visualization_marker_array_->get()->markers.size() );
            for ( size_t i = 0; i < shm_visualization_marker_array_->get()->markers.size(); i++ ) {
                ShmFw::ros::VisualizationMarker &shm_visualization_marker = shm_visualization_marker_array_->get()->markers[i];
                visualization_msgs::Marker &ros_visualization_marker = ros_visualization_marker_array_.markers[i];
                shm_visualization_marker.copyTo ( ros_visualization_marker );
            }
            shm_visualization_marker_array_->unlock();
            pub_marker_array_.publish ( ros_visualization_marker_array_ );
        } else {
            ROS_INFO ( "ShmFwVisualizationMarker::publish_marker timeout: %i", timeout_count_ );
            timeout_count_++;
        }
        if ( frequency_ > 0 )  rate.sleep();
	else  usleep(100);
    }
}
