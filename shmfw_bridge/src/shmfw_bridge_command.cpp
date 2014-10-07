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
#include <shmfw/objects/twist.h>

Command::Command()
    : shm_varible_postfix_("cmd_vel")
    , frequency_ ( 1.0 ) {

}

void Command::initialize ( ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info ) {

    sprintf ( shm_varible_name_, "agv%03d_cmd_vel", agv_info.id );
    ROS_INFO ( "%s/shm_varible_name: %s", n_param.getNamespace().c_str(), shm_varible_name_ );
    
    n_param.getParam ( "frequency", frequency_ );
    ROS_INFO ( "%s/frequency: %5.2f, -1 means on update only", n_param.getNamespace().c_str(), frequency_ );

    
    shm_cmd_ = boost::shared_ptr<ShmFw::Var<ShmFw::Twist> > ( new ShmFw::Var<ShmFw::Twist> ( shm_varible_name_, shm_handler ) );

    pub_ = n.advertise<geometry_msgs::Twist> ( shm_varible_postfix_, 1 );
    thread_ = boost::thread ( boost::bind ( &Command::update, this ) );

}

void Command::update() {
    ros::Rate rate(frequency_);
    int timeout = 1000.0/frequency_;
    int timeout_count = 0;
    geometry_msgs::Twist cmd;
    ShmFw::Twist twist;
    cmd.linear.x = cmd.linear.y = cmd.linear.z = cmd.angular.x = 0,  cmd.angular.y = 0,  cmd.angular.z = 0;
    while ( ros::ok() ) {
        bool read = false;
        if ( frequency_ < 0 ) {
            shm_cmd_->wait();
            read = true;
        } else {
            shm_cmd_->timed_wait ( timeout );
            read = true;
        }
        if ( read ) {
            timeout_count = 0;
	    shm_cmd_->get(twist);
	    twist.copyTo(cmd);
            pub_.publish ( cmd );
        } else {
            ROS_INFO ( "Command::update_motion timeout: %i", timeout_count );
            timeout_count++;
        }
        rate.sleep();
    }
}
