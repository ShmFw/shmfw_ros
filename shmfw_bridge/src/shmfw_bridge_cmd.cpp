/*
* Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include <shmfw_bridge/shmfw_bridge.h>

Command::Command()
    : shm_varible_name_ ( "cmd_vel" )
    , frequency_ ( 1.0 ) {

}

void Command::initialize ( ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info ) {

    n_param.getParam ( "shm_varible_name", shm_varible_name_ );
    ROS_INFO ( "%s/shm_varible_name: %s", n_param.getNamespace().c_str(), shm_varible_name_.c_str() );
    n_param.getParam ( "frequency", frequency_ );
    ROS_INFO ( "%s/frequency: %5.2f, -1 means on update only", n_param.getNamespace().c_str(), frequency_ );

    cmd_ = boost::shared_ptr<ShmFw::Var<ShmFw::Velocity> > ( new ShmFw::Var<ShmFw::Velocity> ( shm_varible_name_, shm_handler ) );

    pub_ = n.advertise<geometry_msgs::Twist> ( "cmd_vel", 1 );
    thread_ = boost::thread ( boost::bind ( &Command::update, this ) );

}

void Command::exit() {
    loop = false;
};

void Command::update() {
    int timeout = 1000.0/frequency_;
    loop = true;
    int timeout_count = 0;
    geometry_msgs::Twist cmd;
    ShmFw::Velocity velocity;
    cmd.linear.x = cmd.linear.y = cmd.linear.z = cmd.angular.x = 0,  cmd.angular.y = 0,  cmd.angular.z = 0;
    while ( loop ) {
        bool read = false;
        if ( frequency_ < 0 ) {
            cmd_->wait();
            read = true;
        } else {
            cmd_->timed_wait ( timeout );
            read = true;
        }
        if ( read ) {
            timeout_count = 0;
	    cmd_->get(velocity);
            cmd.linear.x = velocity.vx;
            cmd.linear.y = velocity.vy;
            cmd.linear.z = velocity.vz;
            cmd.angular.x = velocity.wx;
            cmd.angular.y = velocity.wy;
            cmd.angular.z = velocity.wz;
            pub_.publish ( cmd );
        } else {
            ROS_INFO ( "Command::update_motion timeout: %i", timeout_count );
            timeout_count++;
        }
    }
}
