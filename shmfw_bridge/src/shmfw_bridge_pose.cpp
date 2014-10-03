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

Pose::Pose()
    : target_frame_ ( "map" )
    , source_frame_ ( "base_link" )
    , shm_varible_postfix_ ( "pose" )
    , tf_prefix_ () {

}

void Pose::initialize ( ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info ) {

    n_param.getParam ( "shm_varible_postfix", shm_varible_postfix_ );
    ROS_INFO ( "%s/shm_varible_postfix: %s", n_param.getNamespace().c_str(), shm_varible_postfix_.c_str() );
    
    sprintf ( shm_varible_name_, "agv%03d_%s", agv_info.id, shm_varible_postfix_.c_str() );
    ROS_INFO ( "%s/shm_varible_name: %s", n_param.getNamespace().c_str(), shm_varible_name_ );
    
    n_param.getParam ( "target_frame", target_frame_ );
    ROS_INFO ( "%s/target_frame: %s", n_param.getNamespace().c_str(), target_frame_.c_str() );
    n_param.getParam ( "source_frame", source_frame_ );
    ROS_INFO ( "%s/source_frame: %s", n_param.getNamespace().c_str(), source_frame_.c_str() );
    n_param.getParam ( "tf_prefix", tf_prefix_ );
    ROS_INFO ( "%s/tf_prefix: %s", n_param.getNamespace().c_str(), tf_prefix_.c_str() );

    shm_pose_ = boost::shared_ptr<ShmFw::Var<ShmFw::Pose2DAGV> > ( new ShmFw::Var<ShmFw::Pose2DAGV> ( shm_varible_name_, shm_handler ) );
}

void Pose::update() {
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

}
