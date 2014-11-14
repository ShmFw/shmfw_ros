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
    : shm_name_cmd_ ( "cmd_vel" )
    , timeout_signal_ ( 1.0 )
    , frequency_ ( 10.0 ) {

}

void Command::initialize ( ros::NodeHandle &n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> &shm_handler ) {

    n_param.getParam ( "shm_name_cmd", shm_name_cmd_ );
    ROS_INFO ( "%s/shm_name_cmd: %s", n_param.getNamespace().c_str(), shm_handler->resolve_namespace ( shm_name_cmd_ ).c_str() );

    n_param.getParam ( "timeout_signal", timeout_signal_ );
    ROS_INFO ( "%s/timeout_signal: %f sec", n_param.getNamespace().c_str(), timeout_signal_ );

    n_param.getParam ( "frequency", frequency_ );
    ROS_INFO ( "%s/frequency: %5.2f, -1 means on update only", n_param.getNamespace().c_str(), frequency_ );


    shm_cmd_ = boost::shared_ptr<ShmFw::Var<ShmFw::Twist> > ( new ShmFw::Var<ShmFw::Twist> ( shm_name_cmd_, shm_handler ) );

    pub_ = n.advertise<geometry_msgs::Twist> ( shm_name_cmd_, 1 );
    thread_ = boost::thread ( boost::bind ( &Command::update, this ) );

}

void Command::update() {
    boost::posix_time::time_duration duration_timeout ( boost::posix_time::milliseconds ( timeout_signal_ * 1000 ) );
    double update_freq = frequency_;
    if ( frequency_ < 0 ) update_freq = 100;
    ros::Rate rate ( update_freq );
    int timeout = 1000.0/update_freq;
    int timeout_count = 0;
    geometry_msgs::Twist cmd;
    ShmFw::Twist twist;
    boost::posix_time::ptime next_stop_command =  ShmFw::now();
    bool dead_mean_stop_ready;
    while ( ros::ok() ) {
        bool read = false;
        if ( shm_cmd_->timed_wait ( timeout ) ) {
            read = true;
            dead_mean_stop_ready = true;
        } else {
            if ( frequency_ > 0 ) {
                ROS_INFO ( "Command::update_motion timeout: %i", timeout_count );
                read = true;
                timeout_count++;
            }
        }
        boost::posix_time::time_duration duration_difference =  ShmFw::now() - shm_cmd_->timestampShm();
        if ( duration_difference > duration_timeout ) {
            read = false;
            if ( ShmFw::now() > next_stop_command ) {
                next_stop_command =  ShmFw::now() + duration_timeout;
                if ( dead_mean_stop_ready ) {
                    ROS_INFO ( "Command::update signal timeout: %f sec -> send stop", timeout_signal_ );
                    cmd.linear.x = cmd.linear.y = cmd.linear.z = cmd.angular.x = 0,  cmd.angular.y = 0,  cmd.angular.z = 0;
                    pub_.publish ( cmd );
                    dead_mean_stop_ready = false;
                }
            }
        }
        if ( read ) {
            timeout_count = 0;
            shm_cmd_->get ( twist );
            twist.copyTo ( cmd );
            pub_.publish ( cmd );
        }
        rate.sleep();
    }
}
