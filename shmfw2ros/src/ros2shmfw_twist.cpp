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
#include <geometry_msgs/Twist.h>
#include <shmfw/objects/twist.h>
#include <shmfw/variable.h>
#include <boost/bind.hpp>

class Ros2ShmFwTwist {
public:
    Ros2ShmFwTwist ();
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    std::string shm_segment_name_;
    int shm_segment_size_;
    std::vector<std::string>  shm_twist_name_;
    std::vector<std::string>  msg_twist_name_;
    std::vector<ros::Subscriber> sub_twist_;
    std::vector< boost::shared_ptr< ShmFw::Var<ShmFw::Twist> > > shm_twist_;
    ShmFw::Twist local_twist_;
    void callbackTwist ( const geometry_msgs::TwistConstPtr& msg, int i );
private:
    void publish();
    void read_parameter();
};

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "shmfw_twist" );
    Ros2ShmFwTwist node;
    return 0;
}

Ros2ShmFwTwist::Ros2ShmFwTwist ()
    : n_ ()
    , n_param_ ( "~" )
    , shm_segment_name_ ( ShmFw::DEFAULT_SEGMENT_NAME() )
    , shm_segment_size_ ( ShmFw::DEFAULT_SEGMENT_SIZE() )
    , shm_twist_name_ ()
    , msg_twist_name_ () {

    read_parameter();

    ros::Rate rate ( 1 );
    while ( ros::ok() ) {
        ros::spinOnce();
        rate.sleep();
    }
}

void Ros2ShmFwTwist::callbackTwist ( const geometry_msgs::TwistConstPtr& msg, int i ) {
    local_twist_.copyFrom(*msg);
    shm_twist_[i]->set(local_twist_);
}

void Ros2ShmFwTwist::read_parameter() {
    std::stringstream ss;
    ROS_INFO ( "namespace: %s", n_param_.getNamespace().c_str() );

    n_param_.getParam ( "shm_segment_name", shm_segment_name_ );
    ROS_INFO ( "shm_segment_name: %s", shm_segment_name_.c_str() );

    n_param_.getParam ( "shm_segment_size", shm_segment_size_ );
    ROS_INFO ( "shm_segment_size: %d", shm_segment_size_ );

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shm_segment_name_, shm_segment_size_, n_.getNamespace() );

    std::string shm_twist_name_tmp;
    n_param_.getParam ( "shm_twist_name", shm_twist_name_tmp );
    boost::erase_all ( shm_twist_name_tmp, " " );
    boost::split ( shm_twist_name_, shm_twist_name_tmp, boost::is_any_of ( "," ) );

    for ( unsigned int i = 0; i < shm_twist_name_.size(); i++ ) {
        ROS_INFO ( "%s/shm_twist_name: %s", n_param_.getNamespace().c_str(), shmHdl->resolve_namespace ( shm_twist_name_[i] ).c_str() );
    }

    std::string msg_twist_name_tmp;
    n_param_.getParam ( "msg_twist_name", msg_twist_name_tmp );
    boost::erase_all ( msg_twist_name_tmp, " " );
    boost::split ( msg_twist_name_, msg_twist_name_tmp, boost::is_any_of ( "," ) );

    for ( unsigned int i = 0; i < msg_twist_name_.size(); i++ ) {
        ROS_INFO ( "%s/shm_twist_name: %s", n_param_.getNamespace().c_str(), shmHdl->resolve_namespace ( msg_twist_name_[i] ).c_str() );
    }

    if ( ( msg_twist_name_.size() == 0 ) || ( shm_twist_name_.size() == 0 ) ) {
        ROS_FATAL ( "You have to name at least one shm_twist_name or msg_twist_name!" );
    }
    if ( msg_twist_name_.size() != shm_twist_name_.size() ) {
        ROS_FATAL ( "Number of shm_twist_name and msg_twist_name name parameters must be the same!" );
    } else {
        shm_twist_.resize ( msg_twist_name_.size() );
        sub_twist_.resize ( msg_twist_name_.size() );
    }

    for ( int i = 0; i < msg_twist_name_.size(); i++ ) {
        shm_twist_[i].reset ( new ShmFw::Var<ShmFw::Twist> ( shm_twist_name_[i], shmHdl ) );
        sub_twist_[i] = n_.subscribe<geometry_msgs::Twist> (msg_twist_name_[i], 10, boost::bind(&Ros2ShmFwTwist::callbackTwist, this, _1, i ));
    }

}
