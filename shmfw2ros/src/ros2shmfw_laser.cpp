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
#include <sensor_msgs/LaserScan.h>
#include <shmfw/objects/ros/laser_scan.h>
#include <shmfw/allocator.h>
#include <boost/bind.hpp>

class Ros2ShmFwLaser {
  typedef boost::shared_ptr< ShmFw::Alloc<ShmFw::ros::LaserScan> > ShmLaserScanPtr;
public:
    Ros2ShmFwLaser ();
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    std::string shm_segment_name_;
    int shm_segment_size_;
    std::vector<std::string>  shm_varibale_names_;
    std::vector<std::string>  ros_msg_names_;
    std::vector<ros::Subscriber> subscribers_;
    std::vector< ShmLaserScanPtr > shm_variables_;
    void callback(const sensor_msgs::LaserScanConstPtr& msg, int i);
private:
    void publish();
    void read_parameter();
};

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "shmfw_twist" );
    Ros2ShmFwLaser node;
    return 0;
}

Ros2ShmFwLaser::Ros2ShmFwLaser ()
    : n_ ()
    , n_param_ ( "~" )
    , shm_segment_name_ ( ShmFw::DEFAULT_SEGMENT_NAME() )
    , shm_segment_size_ ( ShmFw::DEFAULT_SEGMENT_SIZE() )
    , shm_varibale_names_ () {

    read_parameter();

    ros::Rate rate ( 1 );
    while ( ros::ok() ) {
        ros::spinOnce();
        rate.sleep();
    }
}

void Ros2ShmFwLaser::callback(const sensor_msgs::LaserScanConstPtr& msg, int i) {
  shm_variables_[i]->lock();
  shm_variables_[i]->get()->copyFrom(*msg);
  shm_variables_[i]->itHasChanged();
  shm_variables_[i]->unlock();
}

void Ros2ShmFwLaser::read_parameter() {
    std::stringstream ss;
    std::string str_tmp;
    
    ROS_INFO ( "ros namespace: %s", n_param_.getNamespace().c_str() );

    n_param_.getParam ( "shm_segment_name", shm_segment_name_ );
    ROS_INFO ( "shm_segment_name: %s", shm_segment_name_.c_str() );

    n_param_.getParam ( "shm_segment_size", shm_segment_size_ );
    ROS_INFO ( "shm_segment_size: %d", shm_segment_size_ );

    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shm_segment_name_, shm_segment_size_, n_.getNamespace() );

    n_param_.getParam ( "ros_msg_names", str_tmp );
    boost::erase_all ( str_tmp, " " );
    boost::split ( ros_msg_names_, str_tmp, boost::is_any_of ( "," ) );
    for ( unsigned int i = 0; i < ros_msg_names_.size(); i++ ) {
        ROS_INFO ( "%s/ros_msg_names: %s", n_param_.getNamespace().c_str(), shmHdl->resolve_namespace ( ros_msg_names_[i] ).c_str() );
    }
    
    n_param_.getParam ( "shm_variable_names", str_tmp );
    boost::erase_all ( str_tmp, " " );
    boost::split ( shm_varibale_names_, str_tmp, boost::is_any_of ( "," ) );
    
    if ( ros_msg_names_.empty() ) {
        ROS_FATAL ( "You have to name at least one ros message name in ros_msg_names!" );
    }  
    if ( shm_varibale_names_.empty() ) {
        ROS_INFO ( "shm_varibale_names is empty, I will use the names of ros_msg_names!" );
	shm_varibale_names_ = ros_msg_names_;
    } 
    if ( shm_varibale_names_.size() != ros_msg_names_.size() ) {
        ROS_FATAL ( "Unequal number of parameters in ros_msg_names and shm_varibale_names!" );
    } else {
        shm_variables_.resize ( shm_varibale_names_.size() );
        subscribers_.resize ( ros_msg_names_.size() );
    }

    for ( unsigned int i = 0; i < shm_varibale_names_.size(); i++ ) {
        ROS_INFO ( "ros -> shm: %25s -> %s", shmHdl->resolve_namespace ( shm_varibale_names_[i] ).c_str(), shmHdl->resolve_namespace ( ros_msg_names_[i] ).c_str() );
    }
    
    for ( int i = 0; i < shm_varibale_names_.size(); i++ ) {
        shm_variables_[i].reset ( new ShmFw::Alloc<ShmFw::ros::LaserScan> ( shm_varibale_names_[i], shmHdl ) );
	subscribers_[i] = n_.subscribe<sensor_msgs::LaserScan> ( ros_msg_names_[i], 10, boost::bind(&Ros2ShmFwLaser::callback, this, _1, i ));
    }

}
