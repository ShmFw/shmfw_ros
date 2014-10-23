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
#include <shmfw/objects/pose.h>
#include <shmfw/objects/pose2d_agv.h>
#include <shmfw/objects/model_state.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PoseStamped.h>

Gazebo::Gazebo()
    : target_frame_ ( "map" )
    , shm_name_pose_ ( "pose_gt" )
    , shm_name_state_ ( "state_gt" )
    , gazebo_model_name_ ( "r1" )
    , frequency_ ( 10 ) {

}

void Gazebo::initialize ( ros::NodeHandle &n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> &shm_handler ) {

    n_param.getParam ( "shm_name_pose", shm_name_pose_ );
    ROS_INFO ( "%s/shm_name_pose: %s", n_param.getNamespace().c_str(), shm_handler->resolve_namespace ( shm_name_pose_ ).c_str() );

    n_param.getParam ( "shm_name_state", shm_name_state_ );
    ROS_INFO ( "%s/shm_name_state: %s", n_param.getNamespace().c_str(), shm_handler->resolve_namespace ( shm_name_state_ ).c_str() );

    n_param.getParam ( "target_frame", target_frame_ );
    ROS_INFO ( "%s/target_frame: %s", n_param.getNamespace().c_str(), target_frame_.c_str() );

    n_param.getParam ( "gazebo_model_name", gazebo_model_name_ );
    ROS_INFO ( "%s/gazebo_model_name: %s", n_param.getNamespace().c_str(), gazebo_model_name_.c_str() );

    n_param.getParam ( "frequency", frequency_ );
    ROS_INFO ( "%s/frequency: %5.2f", n_param.getNamespace().c_str(), frequency_ );

    shm_pose_ = boost::shared_ptr<ShmFw::Var<ShmFw::Pose2DAGV> > ( new ShmFw::Var<ShmFw::Pose2DAGV> ( shm_name_pose_, shm_handler ) );
    shm_state_ = boost::shared_ptr<ShmFw::Var<ShmFw::ModelState> > ( new ShmFw::Var<ShmFw::ModelState> ( shm_name_state_, shm_handler ) );
    msg_.header.frame_id = target_frame_;
    msg_.header.seq = 0;
    pub_ = n.advertise<geometry_msgs::PoseStamped> ( "pose_gt", 1 );
    service_client_ = n.serviceClient<gazebo_msgs::GetModelState> ( "/gazebo/get_model_state" );
    thread_ = boost::thread ( boost::bind ( &Gazebo::update, this ) );
}

void Gazebo::update() {
    ShmFw::Pose2DAGV pose2D;
    ShmFw::Pose pose3D;
    ShmFw::ModelState modelState;
    ros::Rate rate ( frequency_ );
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = gazebo_model_name_;
    unsigned int service_call_failure = 0;
    while ( ros::ok() ) {
        if ( pub_.getNumSubscribers() > 0 ) {
            if ( service_client_.call ( srv ) ) {
                pose3D.copyFrom ( srv.response.pose );
                modelState.copyFrom ( srv.response );
                pose3D.getPose2D ( pose2D );
                shm_pose_->set ( pose2D );
                shm_state_->set ( modelState );
                msg_.header.stamp = srv.response.header.stamp;
                pose3D.copyTo ( msg_.pose );
                pub_.publish ( msg_ );
            } else {
                service_call_failure++;
                if ( service_call_failure > 100 ) {
                    ROS_ERROR ( "Failed get gazebo robot pose of %s for %d time in a row.", gazebo_model_name_.c_str(), service_call_failure );
                    service_call_failure = 0;
                }
            }
        }
        rate.sleep();
    }

}
