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
    , shm_variable_name_ ( "VisualizationMarker" )
    , timeout_count_ ( 0 ) {

    read_parameter();

    ros::Rate rate ( frequency_ );
    while ( ros::ok() ) {
        publish_marker();
        publish_markers();
        ros::spinOnce();
        rate.sleep(); 
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

    n_param_.getParam ( "shm_variable_name", shm_variable_name_ );
    ROS_INFO ( "shm_variable_name: %s", shm_variable_name_.c_str() );

    std::string names;
    n_param_.getParam ( "shm_variable_names", names );
    boost::erase_all(names, " ");
    boost::split ( shm_variable_names_, names, boost::is_any_of(","));
    for ( size_t i = 0; i < shm_variable_names_.size(); i++ ) ss << ( i!=0?", ":"" ) << shm_variable_names_[i];
    ROS_INFO ( "shm_variable_names: %s", ss.str().c_str() );

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shm_segment_name_, shm_segment_size_ );
    shmHdl->setNamespace(n_.getNamespace());
    
    shm_visualization_marker_.reset ( new ShmFw::Alloc<ShmFw::ros::VisualizationMarker> ( shm_variable_name_, shmHdl ) );

    shm_visualization_markers_.resize ( shm_variable_names_.size() );
    for ( size_t i = 0; i < shm_variable_names_.size(); i++ ) {
        shm_visualization_markers_[i].reset ( new ShmFw::Alloc<ShmFw::ros::VisualizationMarker> ( shm_variable_names_[i], shmHdl ) );
    }

    pub_marker_ = n_.advertise<visualization_msgs::Marker> ( "/visualization_marker", 10 );
    pub_markers_ = n_.advertise<visualization_msgs::MarkerArray> ( "/visualization_marker_array", 10 );
}

void ShmFwVisualizationMarker::publish_marker() {
    if ( pub_marker_.getNumSubscribers() == 0 ) return;
    bool read = false;
    if ( frequency_ < 0 ) {
        shm_visualization_marker_->wait();
        read = true;
    } else {
        shm_visualization_marker_->timed_wait ( 1.0/frequency_ );
        read = true;
    }
    if ( read ) {
        timeout_count_ = 0;
        shm_visualization_marker_->lock();
        shm_visualization_marker_->ref().copyTo ( ros_visualization_marker_ );
        shm_visualization_marker_->unlock();
        pub_marker_.publish ( ros_visualization_marker_ );
    } else {
        ROS_INFO ( "ShmFwVisualizationMarker::publish_marker timeout: %i", timeout_count_ );
        timeout_count_++;
    }
}

void ShmFwVisualizationMarker::publish_markers() {
    if ( pub_markers_.getNumSubscribers() == 0 ) return;
    ros_visualization_markers_.markers.resize ( shm_visualization_markers_.size() );
    for ( size_t i = 0; i < shm_visualization_markers_.size(); i++ ) {
        ShmVisualizationMarkerPtr shm_visualization_marker = shm_visualization_markers_[i];
        visualization_msgs::Marker &ros_visualization_marker = ros_visualization_markers_.markers[i];
        bool read = false;
        if ( frequency_ < 0 ) {
            shm_visualization_marker->wait();
            read = true;
        } else {
            shm_visualization_marker->timed_wait ( 1.0/frequency_ );
            read = true;
        }
        if ( read ) {
            timeout_count_ = 0;
            shm_visualization_marker->lock();
            shm_visualization_marker->ref().copyTo ( ros_visualization_marker );
            shm_visualization_marker->unlock();

        } else {
            ROS_INFO ( "ShmFwVisualizationMarker::publish_markers timeout: %i", timeout_count_ );
            timeout_count_++;
        }
    }
    pub_markers_.publish ( ros_visualization_markers_ );
}
