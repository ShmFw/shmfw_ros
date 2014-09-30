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

ShmTFNode::ShmTFNode ()
    : n_ ()
    , n_param_ ( "~" )
    , frequency_ ( 10.0 )
    , shm_segment_name_ ( DEFAULT_SEGMENT_NAME )
    , shm_segment_size_ ( DEFAULT_SEGMENT_SIZE ) {

    read_parameter();
    
    shm_handler_ = ShmFw::Handler::create ( shm_segment_name_, shm_segment_size_ );

    command_ = boost::shared_ptr<Command> ( new Command ); 
    command_->initialize(n_, ros::NodeHandle(n_param_,"cmd"), shm_handler_, agv_info_);
    
    path_ = boost::shared_ptr<Path> ( new Path ); 
    path_->initialize(n_, ros::NodeHandle(n_param_,"path"), shm_handler_, agv_info_);
    
    pose_ = boost::shared_ptr<Pose> ( new Pose ); 
    pose_->initialize(n_, ros::NodeHandle(n_param_,"pose"), shm_handler_, agv_info_);

    ros::Rate rate ( frequency_ );
    while ( ros::ok() ) {
        pose_->update();
        ros::spinOnce();
        rate.sleep();
    }
}

void ShmTFNode::read_parameter() {
    ROS_INFO ( "namespace: %s", n_param_.getNamespace().c_str());
    
    n_param_.getParam ( "frequency", frequency_ );
    ROS_INFO ( "frequency: %5.2f", frequency_ );
    
    n_param_.getParam ( "id_agv", agv_info_.id );
    ROS_INFO ( "id_agv: %d", agv_info_.id );  
        
    n_param_.getParam ( "shm_segment_name", shm_segment_name_ );
    ROS_INFO ( "shm_segment_name: %s", shm_segment_name_.c_str() );
    
    n_param_.getParam ( "shm_segment_size", shm_segment_size_ );
    ROS_INFO ( "shm_segment_size: %d", shm_segment_size_ );  
}

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "shmfw_bridge" );
    ShmTFNode node;
    return 0;
}
