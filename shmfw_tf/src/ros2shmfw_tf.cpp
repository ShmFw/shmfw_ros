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
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <shmfw/objects/pose.h>
#include <shmfw/variable.h>

class Ros2ShmFwTf {
public:
    Ros2ShmFwTf ();
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    double frequency_;
    double timeout_signal_;  ///onyl used if frequency_ < 0
    std::string shm_segment_name_;
    int shm_segment_size_;
    std::string target_frame_;
    std::string target_frame_id_;
    std::string source_frame_;
    std::string source_frame_id_;
    std::string tf_prefix_;
    std::string shm_name_pose_;
    std::string shm_name_pose2d_;
    ShmFw::Var<ShmFw::Pose> shm_pose_;
    ShmFw::Var<ShmFw::Pose2D> shm_pose2d_;
    tf::TransformListener listener_;
private:
    void loop();
    void read_parameter();
};

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "shmfw_twist" );
    Ros2ShmFwTf node;
    return 0;
}

Ros2ShmFwTf::Ros2ShmFwTf ()
    : n_ ()
    , n_param_ ( "~" )
    , frequency_ ( 10.0 )
    , shm_segment_name_ ( ShmFw::DEFAULT_SEGMENT_NAME() )
    , shm_segment_size_ ( ShmFw::DEFAULT_SEGMENT_SIZE() )
    , target_frame_ ( "map" )
    , source_frame_ ( "base_link" )
    , tf_prefix_ ()
    , shm_name_pose_ ( "pose3d" )
    , shm_name_pose2d_ ( "pose" ) {

    read_parameter();
    loop();
}

void Ros2ShmFwTf::read_parameter() {
    std::stringstream ss;
    ROS_INFO ( "namespace: %s", n_param_.getNamespace().c_str() );

    n_param_.getParam ( "frequency", frequency_ );
    ROS_INFO ( "frequency: %5.2f", frequency_ );

    n_param_.getParam ( "shm_segment_name", shm_segment_name_ );
    ROS_INFO ( "shm_segment_name: %s", shm_segment_name_.c_str() );

    n_param_.getParam ( "shm_segment_size", shm_segment_size_ );
    ROS_INFO ( "shm_segment_size: %d", shm_segment_size_ );

    n_param_.getParam ( "target_frame", target_frame_ );
    ROS_INFO ( "%s/target_frame: %s", n_param_.getNamespace().c_str(), target_frame_.c_str() );

    n_param_.getParam ( "source_frame", source_frame_ );
    ROS_INFO ( "%s/source_frame: %s", n_param_.getNamespace().c_str(), source_frame_.c_str() );

    n_param_.getParam ( "tf_prefix", tf_prefix_ );
    ROS_INFO ( "%s/tf_prefix: %s", n_param_.getNamespace().c_str(), tf_prefix_.c_str() );


    ShmFw::HandlerPtr shmHdl = ShmFw::Handler::create ( shm_segment_name_, shm_segment_size_, n_.getNamespace() );

    n_param_.getParam ( "shm_name_pose", shm_name_pose_ );
    ROS_INFO ( "%s/shm_name_pose: %s", n_param_.getNamespace().c_str(), shmHdl->resolve_namespace ( shm_name_pose_ ).c_str() );

    target_frame_id_ = tf::resolve ( tf_prefix_, target_frame_ );
    source_frame_id_ = tf::resolve ( tf_prefix_, source_frame_ );
    ROS_INFO ( "Waiting for TF %s -> %s",  target_frame_id_.c_str() ,  source_frame_id_.c_str() );

    shm_pose_.construct ( shm_name_pose_, shmHdl );
    shm_pose_.info_text ( "Pose computed form TF " + target_frame_id_ + " -> " + source_frame_id_ + " frame" );

    n_param_.getParam ( "shm_name_pose2d", shm_name_pose2d_ );
    ROS_INFO ( "%s/shm_name_pose2d: %s", n_param_.getNamespace().c_str(), shmHdl->resolve_namespace ( shm_name_pose2d_ ).c_str() );

    shm_pose2d_.construct ( shm_name_pose2d_, shmHdl );
    shm_pose2d_.info_text ( "Pose computed form TF " + target_frame_id_ + " -> " + source_frame_id_ + " frame" );
}

void Ros2ShmFwTf::loop() {


    ShmFw::Pose pose;
    ShmFw::Pose2D pose2d;
    ros::Rate rate ( frequency_ );
    while ( ros::ok() ) {
        tf::StampedTransform transform;
        tfScalar yaw, pitch, roll;
        try {
            listener_.lookupTransform ( target_frame_id_, source_frame_id_, ros::Time ( 0 ), transform );
            tf::Quaternion q = transform.getRotation();
            tf::Vector3 t = transform.getOrigin();
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            pose.position.x = t.x();
            pose.position.y = t.y();
            pose.position.z = t.z();
            shm_pose_.set ( pose );

            pose.getPose2D ( pose2d );
            shm_pose2d_.set ( pose2d );
        } catch ( tf::TransformException ex ) {
            //ROS_ERROR ( "%s",ex.what() );
            ros::Duration ( 1.0 ).sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
}
