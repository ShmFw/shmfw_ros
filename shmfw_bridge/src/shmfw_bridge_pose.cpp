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
#include <shmfw/objects/pose2d_agv.h>
#include <shmfw/objects/agent_state.h>

Pose::Pose()
    : target_frame_ ( "map" )
    , source_frame_ ( "base_link" )
    , shm_name_pose_ ( "pose" )
    , shm_name_agent_state_ ( "agent_state" )
    , trip_recorder_scale_(1000)
    , tf_prefix_ ()
    , frequency_ ( 10 ) {

}

void Pose::initialize ( ros::NodeHandle &n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> &shm_handler ) {

    n_param.getParam ( "shm_name_pose", shm_name_pose_ );
    ROS_INFO ( "%s/shm_name_pose: %s", n_param.getNamespace().c_str(), shm_handler->resolve_namespace ( shm_name_pose_ ).c_str() );
    n_param.getParam ( "shm_name_agent_state", shm_name_agent_state_ );
    ROS_INFO ( "%s/shm_name_agent_state: %s", n_param.getNamespace().c_str(), shm_handler->resolve_namespace ( shm_name_agent_state_ ).c_str() );
    n_param.getParam ( "target_frame", target_frame_ );
    ROS_INFO ( "%s/target_frame: %s", n_param.getNamespace().c_str(), target_frame_.c_str() );
    n_param.getParam ( "source_frame", source_frame_ );
    ROS_INFO ( "%s/source_frame: %s", n_param.getNamespace().c_str(), source_frame_.c_str() );
    n_param.getParam ( "tf_prefix", tf_prefix_ );
    ROS_INFO ( "%s/tf_prefix: %s", n_param.getNamespace().c_str(), tf_prefix_.c_str() );

    n_param.getParam ( "trip_recorder_scale", trip_recorder_scale_ );
    ROS_INFO ( "%s/trip_recorder_scale: %f", n_param.getNamespace().c_str(), trip_recorder_scale_ );

    shm_pose_ = boost::shared_ptr<ShmFw::Var<ShmFw::Pose2DAGV> > ( new ShmFw::Var<ShmFw::Pose2DAGV> ( shm_name_pose_, shm_handler ) );
    std::string target_frame_id = tf::resolve ( tf_prefix_, target_frame_ );
    std::string source_frame_id = tf::resolve ( tf_prefix_, source_frame_ );
    shm_pose_->info_text ( "Pose computed form TF " + source_frame_id + " -> " + target_frame_id + " frame" );
    shm_agent_state_ = boost::shared_ptr<ShmFw::Var<ShmFw::AgentState> > ( new ShmFw::Var<ShmFw::AgentState> ( shm_name_agent_state_, shm_handler ) );
    thread_ = boost::thread ( boost::bind ( &Pose::update, this ) );
    sub_trip_recorder_ = n.subscribe ( "trip_recorder", 1000, &Pose::callbackTripRecorder, this );
    sub_command_current_ = n.subscribe ( "cmd_vel_current", 1000, &Pose::callbackCmdCurrent, this );
    sub_command_target_ = n.subscribe ( "cmd_vel_target", 1000, &Pose::callbackCmdTarget, this );
}

void Pose::update() {
    ros::Rate rate ( frequency_ );
    while ( ros::ok() ) {
        tf::StampedTransform transform;
        std::string target_frame_id = tf::resolve ( tf_prefix_, target_frame_ );
        std::string source_frame_id = tf::resolve ( tf_prefix_, source_frame_ );
        tfScalar yaw, pitch, roll;
        ShmFw::Pose2DAGV p;
        try {
            listener_.lookupTransform ( target_frame_id, source_frame_id, ros::Time ( 0 ), transform );
            transform.getBasis().getRPY ( roll, pitch, yaw );
            p.position.x = transform.getOrigin() [0];
            p.position.y = transform.getOrigin() [1];
            p.orientation = yaw;
            shm_pose_->set ( p );
        } catch ( tf::TransformException ex ) {
            //ROS_ERROR ( "%s",ex.what() );
            ros::Duration ( 1.0 ).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void Pose::callbackTripRecorder ( const std_msgs::UInt64Ptr& trip_recorder ) {
    agent_state_.trip_recorder = trip_recorder->data / trip_recorder_scale_;
    agent_state_.sub_meter_trip_recorder = ((double)( trip_recorder->data % (uint64) trip_recorder_scale_ ))  / trip_recorder_scale_;
}

void Pose::callbackCmdCurrent ( const geometry_msgs::TwistPtr& command_current ) {
    agent_state_.current.copyFrom ( *command_current );
}
void Pose::callbackCmdTarget ( const geometry_msgs::TwistPtr& command_target ) {
    agent_state_.target.copyFrom ( *command_target );
    shm_agent_state_->set ( agent_state_ );
}

/*
void Pose::callbackAgentState ( const gazebo_msgs::AgentState::ConstPtr& msg ) {
ShmFw::AgentState s;
s.trip_recorder = msg->trip_recorder;
s.sub_meter_trip_recorder = msg->sub_meter_trip_recorder;
s.current.copyFrom ( msg->current );
s.target.copyFrom ( msg->target );
shm_agent_state_->set ( s );
}
*/
