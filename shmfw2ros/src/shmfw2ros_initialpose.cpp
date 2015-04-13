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

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <shmfw/objects/model_state.h>
#include <shmfw/variable.h>

class ShmFw2RosPose {
  typedef boost::shared_ptr< ShmFw::Var<ShmFw::ModelState> > ShmModelStatePtr;
public:
    ShmFw2RosPose ();
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    double frequency_;
    std::string shm_segment_name_;
    int shm_segment_size_;
    std::string  shm_model_state_name_;
    std::string  target_frame_;
    ros::Publisher pub_;
    ShmModelStatePtr shm_model_state_;
    geometry_msgs::PoseWithCovarianceStamped msg_init_pose_;
private:
    void convert(const ShmFw::ModelState &src, geometry_msgs::PoseWithCovarianceStamped &des);
    void publish();
    void read_parameter();
};

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "shmfw_twist" );
    ShmFw2RosPose node;
    return 0;
}

ShmFw2RosPose::ShmFw2RosPose ()
    : n_ ()
    , n_param_ ( "~" )
    , frequency_ ( 1.0 )
    , shm_segment_name_ ( ShmFw::DEFAULT_SEGMENT_NAME() )
    , shm_segment_size_ ( ShmFw::DEFAULT_SEGMENT_SIZE() )
    , shm_model_state_name_ ( "init_state" )
    , target_frame_ ( "map" ) {
    read_parameter();
    publish();
}

void ShmFw2RosPose::read_parameter() {
    std::stringstream ss;
    ROS_INFO ( "namespace: %s", n_param_.getNamespace().c_str() );

    n_param_.getParam ( "frequency", frequency_ );
    ROS_INFO ( "frequency: %5.2f", frequency_ );

    n_param_.getParam ( "shm_segment_name", shm_segment_name_ );
    ROS_INFO ( "shm_segment_name: %s", shm_segment_name_.c_str() );

    n_param_.getParam ( "shm_segment_size", shm_segment_size_ );
    ROS_INFO ( "shm_segment_size: %d", shm_segment_size_ );

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shm_segment_name_, shm_segment_size_, n_.getNamespace() );
  
    n_param_.getParam ( "shm_model_state_name", shm_model_state_name_ );
    ROS_INFO ( "%s/shm_model_state_name: %s", n_param_.getNamespace().c_str(), shmHdl->resolve_namespace(shm_model_state_name_).c_str() );
        
    n_param_.getParam ( "target_frame", target_frame_ );
    ROS_INFO ( "%s/target_frame: %s", n_param_.getNamespace().c_str(), shmHdl->resolve_namespace(target_frame_).c_str() );

    
    shm_model_state_.reset ( new ShmFw::Var<ShmFw::ModelState> ( shm_model_state_name_, shmHdl ) ); /// create shm variable    
    pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "initialpose", 10 );    /// create publisher
    
    msg_init_pose_.header.frame_id = target_frame_;
    msg_init_pose_.header.seq = 0;
    
}

void ShmFw2RosPose::convert(const ShmFw::ModelState &src, geometry_msgs::PoseWithCovarianceStamped &des) {
  des.header.frame_id = target_frame_;
  des.header.stamp.fromBoost(shm_model_state_->timestampLocal());
  ShmFw::ModelState tmp = src;
  tmp.pose.orientation.setRotation(ShmFw::Vector3<double>(0,0,1), 0);  /// Quickhack
  tmp.pose.copyTo(des.pose.pose);
  auto &c = des.pose.covariance;
  c[ 0] = 0.25, c[ 1] = 0.00, c[ 2] = 0.00, c[ 3] = 0.00, c[ 4] = 0.00, c[ 5] = 0.00;
  c[ 6] = 0.00, c[ 7] = 0.25, c[ 8] = 0.00, c[ 9] = 0.00, c[10] = 0.00, c[11] = 0.00;
  c[12] = 0.00, c[13] = 0.00, c[14] = 0.00, c[15] = 0.00, c[16] = 0.00, c[17] = 0.00;
  c[18] = 0.00, c[19] = 0.00, c[20] = 0.00, c[21] = 0.00, c[22] = 0.00, c[23] = 0.00;
  c[24] = 0.00, c[25] = 0.00, c[26] = 0.00, c[27] = 0.00, c[28] = 0.00, c[29] = 0.00;
  c[30] = 0.00, c[31] = 0.00, c[32] = 0.00, c[33] = 0.00, c[34] = 0.00, c[35] = 0.06853891945200942;
}

void ShmFw2RosPose::publish() {

    double update_freq = frequency_;
    if ( frequency_ < 0 ) update_freq = 100;
    ros::Rate rate ( update_freq );
    int timeout = 1000.0/update_freq;
    int timeout_count = 0;
    ShmFw::ModelState model_State;
    boost::posix_time::ptime next_stop_command =  ShmFw::now();
    bool publish;
    while ( ros::ok() ) {
        publish = false;
        if ( shm_model_state_->timed_wait ( timeout ) ) {
            shm_model_state_->get ( model_State );
            convert(model_State, msg_init_pose_);
            publish = true;
        } else {
            if ( frequency_ > 0 ) {
                timeout_count++;
            }
        }
        if ( publish ) {
            timeout_count = 0;
            pub_.publish ( msg_init_pose_ );
        }
        rate.sleep();
    }
}
