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
#include <shmfw/variable.h>
#include <shmfw/serialization/deque.h>

ShmFwBridge::ShmFwBridge ()
    : n_ ()
    , n_param_ ( "~" )
    , frequency_ ( 10.0 )
    , shm_segment_name_ ( ShmFw::DEFAULT_SEGMENT_NAME() )
    , shm_segment_size_ ( ShmFw::DEFAULT_SEGMENT_SIZE() )
    , bridge_pose_ ( true )
    , bridge_command_ ( true )
    , bridge_segments_ ( true )
    , bridge_waypoints_ ( true )
    , bridge_gazebo_ ( true ){

    read_parameter();

    shm_handler_ = ShmFw::Handler::create ( shm_segment_name_, shm_segment_size_ );
    shm_handler_->setNamespace(n_.getNamespace());
    
    if ( bridge_command_ ) {
        command_ = boost::shared_ptr<Command> ( new Command );
        command_->initialize ( n_, ros::NodeHandle ( n_param_,"cmd" ), shm_handler_);
    }
    if ( bridge_segments_ ) {
        segments_ = boost::shared_ptr<Segments> ( new Segments );
        segments_->initialize ( n_, ros::NodeHandle ( n_param_,"path" ), shm_handler_);
    }
    if ( bridge_waypoints_ ) {
        waypoints_ = boost::shared_ptr<WayPoints> ( new WayPoints );
        waypoints_->initialize ( n_, ros::NodeHandle ( n_param_,"waypoints" ), shm_handler_);
    }
    if ( bridge_pose_ ) {
        pose_ = boost::shared_ptr<Pose> ( new Pose );
        pose_->initialize ( n_, ros::NodeHandle ( n_param_,"pose" ), shm_handler_);
    }
    if ( bridge_gazebo_ ) {
        gazebo_ = boost::shared_ptr<Gazebo> ( new Gazebo );
        gazebo_->initialize ( n_, ros::NodeHandle ( n_param_,"gazebo" ), shm_handler_);
    }
    ros::Rate rate ( frequency_ );
    ros::spin();
}

void ShmFwBridge::read_parameter() {
    ROS_INFO ( "namespace: %s", n_param_.getNamespace().c_str() );

    n_param_.getParam ( "frequency", frequency_ );
    ROS_INFO ( "frequency: %5.2f", frequency_ );

    n_param_.getParam ( "shm_segment_name", shm_segment_name_ );
    ROS_INFO ( "shm_segment_name: %s", shm_segment_name_.c_str() );

    n_param_.getParam ( "shm_segment_size", shm_segment_size_ );
    ROS_INFO ( "shm_segment_size: %d", shm_segment_size_ );

    n_param_.getParam ( "bridge_pose", bridge_pose_ );
    ROS_INFO ( "bridge_pose: %s", ( bridge_pose_?"ture":"flase" ) );

    n_param_.getParam ( "bridge_command", bridge_command_ );
    ROS_INFO ( "bridge_command: %s", ( bridge_command_?"ture":"flase" ) );

    n_param_.getParam ( "bridge_segments", bridge_segments_ );
    ROS_INFO ( "bridge_segments: %s", ( bridge_segments_?"ture":"flase" ) );

    n_param_.getParam ( "bridge_waypoints", bridge_waypoints_ );
    ROS_INFO ( "bridge_waypoints: %s", ( bridge_waypoints_?"ture":"flase" ) );

    n_param_.getParam ( "bridge_gazebo", bridge_gazebo_ );
    ROS_INFO ( "bridge_gazebo: %s", ( bridge_gazebo_?"ture":"flase" ) );
    
}

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "shmfw_bridge" );
    ShmFwBridge node;
    return 0;
}
