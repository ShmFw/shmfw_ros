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

#include <shmfw_twist_node.h>

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "shmfw_twist" );
    ShmFwTwist node;
    return 0;
}

ShmFwTwist::ShmFwTwist ()
    : n_ ()
    , n_param_ ( "~" )
    , frequency_ ( 10.0 )
    , shm_segment_name_ ( ShmFw::DEFAULT_SEGMENT_NAME() )
    , shm_segment_size_ ( ShmFw::DEFAULT_SEGMENT_SIZE() )
    , shm_twist_name_ ( "twist" )
    , timeout_signal_ ( 1.0 ) {

    read_parameter();
    publish();
}

void ShmFwTwist::read_parameter() {
    std::stringstream ss;
    ROS_INFO ( "namespace: %s", n_param_.getNamespace().c_str() );

    n_param_.getParam ( "frequency", frequency_ );
    ROS_INFO ( "frequency: %5.2f", frequency_ );

    n_param_.getParam ( "shm_segment_name", shm_segment_name_ );
    ROS_INFO ( "shm_segment_name: %s", shm_segment_name_.c_str() );

    n_param_.getParam ( "shm_segment_size", shm_segment_size_ );
    ROS_INFO ( "shm_segment_size: %d", shm_segment_size_ );

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shm_segment_name_, shm_segment_size_, n_.getNamespace() );
  
    n_param_.getParam ( "shm_twist_name", shm_twist_name_ );
    ROS_INFO ( "%s/shm_twist_name: %s", n_param_.getNamespace().c_str(), shmHdl->resolve_namespace(shm_twist_name_).c_str() );

    n_param_.getParam ( "timeout_signal", timeout_signal_ );
    ROS_INFO ( "%s/timeout_signal: %f sec", n_param_.getNamespace().c_str(), timeout_signal_ );

    ros_twist_default_.angular.x = 0;
    ros_twist_default_.angular.y = 0;
    ros_twist_default_.angular.z = 0;
    ros_twist_default_.linear.x = 0;
    ros_twist_default_.linear.y = 0;
    ros_twist_default_.linear.z = 0;
    
    shm_twist_.reset ( new ShmFw::Var<ShmFw::Twist> ( shm_twist_name_, shmHdl ) );
    
    pub_twist_ = n_.advertise<geometry_msgs::Twist> ( shm_twist_name_, 10 );
    
}

void ShmFwTwist::publish() {

    boost::posix_time::time_duration duration_timeout ( boost::posix_time::milliseconds ( timeout_signal_ * 1000 ) );
    double update_freq = frequency_;
    if ( frequency_ < 0 ) update_freq = 100;
    ros::Rate rate ( update_freq );
    int timeout = 1000.0/update_freq;
    int timeout_count = 0;
    ShmFw::Twist twist;
    boost::posix_time::ptime next_stop_command =  ShmFw::now();
    bool publish;
    while ( ros::ok() ) {
        publish = false;
        if ( shm_twist_->timed_wait ( timeout ) ) {
            shm_twist_->get ( twist );
            twist.copyTo ( ros_twist_ );
            publish = true;
        } else {
            if ( frequency_ > 0 ) {
                ROS_INFO ( "timeout_count: %i cycles", timeout_count );
                timeout_count++;
            }
        }
        boost::posix_time::time_duration duration_difference =  ShmFw::now() - shm_twist_->timestampShm();
        if (( duration_difference > duration_timeout ) && ( ShmFw::now() > next_stop_command)) {
            next_stop_command =  ShmFw::now() + duration_timeout;
	    ros_twist_ = ros_twist_default_;
            publish = true;
        }
        if ( publish ) {
            timeout_count = 0;
            pub_twist_.publish ( ros_twist_ );
        }
        rate.sleep();
    }
}
