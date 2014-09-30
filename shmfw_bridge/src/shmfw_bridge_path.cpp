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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

#include <shmfw/variable.h>
#include <shmfw/objects/pose.h>
#include <shmfw/objects/pose2d_agv.h>
#include <shmfw/objects/segment_agv.h>
#include <shmfw/serialization/deque.h>

Path::Path()
    : frequency_ ( 1.0 )
    , scale_ ( 1.0 )
    , resolution_ ( 0.2 ) {

}

void Path::initialize ( ros::NodeHandle n, ros::NodeHandle n_param, boost::shared_ptr<ShmFw::Handler> shm_handler, const AGVInfo &agv_info ) {

    char name_segments_ahead[0xFF];
    sprintf ( name_segments_ahead, "agv%03d_segmentsAhead", agv_info.id );
    ROS_INFO ( "%s/shm_varible_name_segmentsAhead: %s", n_param.getNamespace().c_str(), name_segments_ahead );

    n_param.getParam ( "scale", scale_ );
    ROS_INFO ( "%s/scale: %5.2f", n_param.getNamespace().c_str(), scale_ );

    n_param.getParam ( "resolution", resolution_ );
    ROS_INFO ( "%s/resolution: %5.2f", n_param.getNamespace().c_str(), resolution_ );

    n_param.getParam ( "frequency", frequency_ );
    ROS_INFO ( "%s/frequency: %5.2f, -1 means on update only", n_param.getNamespace().c_str(), frequency_ );


    segments_ahead_ = boost::shared_ptr<ShmFw::Deque<ShmFw::SegmentAGV> > ( new ShmFw::Deque<ShmFw::SegmentAGV> ( name_segments_ahead, shm_handler ) );

    pub_path_ = n.advertise<nav_msgs::Path> ( "segments_ahead", 1 );
    pub_waypoints_ = n.advertise<geometry_msgs::PoseArray> ( "waypoints", 1 );
    pub_marker_ = n.advertise<visualization_msgs::Marker> ( "visualization_marker", 10 );
    thread_ = boost::thread ( boost::bind ( &Path::update, this ) );

    initMarker();

}
void Path::initMarker() {

    segment_arrow_     = boost::shared_ptr<visualization_msgs::Marker > ( new visualization_msgs::Marker );
    points_start_ = boost::shared_ptr<visualization_msgs::Marker > ( new visualization_msgs::Marker );
    points_end_  = boost::shared_ptr<visualization_msgs::Marker > ( new visualization_msgs::Marker );

    segment_arrow_->header.frame_id = points_start_->header.frame_id = points_end_->header.frame_id = "map";
    segment_arrow_->header.stamp = points_start_->header.stamp = points_end_->header.stamp = ros::Time::now();
    segment_arrow_->ns = "segments";
    points_start_->ns = points_end_->ns = "segment_points";
    segment_arrow_->action = points_start_->action = points_end_->action = visualization_msgs::Marker::ADD;
    segment_arrow_->pose.orientation.w = points_start_->pose.orientation.w = points_end_->pose.orientation.w = 1.0;
    segment_arrow_->id = 0;
    points_start_->id = 1;
    points_end_->id = 2;
    segment_arrow_->type = visualization_msgs::Marker::ARROW;
    points_start_->type = visualization_msgs::Marker::POINTS;
    points_end_->type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    segment_arrow_->scale.x = 0.01;
    segment_arrow_->scale.y = 0.02;
    segment_arrow_->scale.z = segment_arrow_->scale.y*2;
    points_start_->scale.x = points_start_->scale.y = 0.1;
    points_end_->scale.x = points_end_->scale.y = 0.1;

    // start points are green
    segment_arrow_->color.g = 1.0f;
    segment_arrow_->color.a = 1.0;

    // center points are blue
    points_start_->color.b = 1.0;
    points_start_->color.a = 1.0;

    // end points are red
    points_end_->color.r = 1.0;
    points_end_->color.a = 1.0;

}

void Path::exit() {
    loop = false;
}

void Path::update() {
    int timeout = 1000.0/frequency_;
    loop = true;
    int timeout_count = 0;
    std::vector<ShmFw::SegmentAGV> segments;
    std::vector<ShmFw::Pose> poses;
    nav_msgs::Path path;
    geometry_msgs::PoseArray waypoints;
    while ( loop ) {
        bool read = false;
        if ( frequency_ < 0 ) {
            segments_ahead_->wait();
            read = true;
        } else {
            segments_ahead_->timed_wait ( timeout );
            read = true;
        }
        if ( read ) {
            timeout_count = 0;
            segments_ahead_->get ( segments );
            getWayPoints ( segments, poses );
            path.header.frame_id = "map";
            path.header.stamp = ros::Time::now();
            path.poses.resize ( poses.size() );
            waypoints.header = path.header;
            waypoints.poses.resize ( poses.size() );
            for ( size_t i = 0; i < poses.size(); i++ ) {
                path.poses[i].header = path.header;
                path.poses[i].pose.position.x = poses[i].position.x * scale_;
                path.poses[i].pose.position.y = poses[i].position.y * scale_;
                path.poses[i].pose.position.z = poses[i].position.z * scale_;
                path.poses[i].pose.orientation.x = poses[i].orientation.x;
                path.poses[i].pose.orientation.y = poses[i].orientation.y;
                path.poses[i].pose.orientation.z = poses[i].orientation.z;
                path.poses[i].pose.orientation.w = poses[i].orientation.w;

                waypoints.poses[i] = path.poses[i].pose;
            }
            pub_path_.publish ( path );
            pub_waypoints_.publish ( waypoints );
        } else {
            ROS_INFO ( "Command::update_motion timeout: %i", timeout_count );
            timeout_count++;
        }

    }
}

std::vector<ShmFw::Pose> Path::getWayPoints ( const std::vector<ShmFw::SegmentAGV> &segments, std::vector<ShmFw::Pose> &path ) {

    segment_arrow_->points.clear();
    points_end_->points.clear();
    points_start_->points.clear();
    double resolution = ( resolution_/scale_ );
    ShmFw::Point2D p1;
    double carry_over = 0;
    for ( int i = 0; i < segments.size(); i++ ) {
        const ShmFw::SegmentAGV &s = segments[i];
        //std::cout  << s << std::endl;

        if ( segment_arrow_ ) {
            // for debugging
            segment_arrow_->id = i;
            segment_arrow_->points.clear();
            geometry_msgs::Point p;
            p.x = s.start.position.x, p.y = s.start.position.y, p.z = s.level;
            segment_arrow_->points.push_back ( p );
            p.x = s.end.position.x, p.y = s.end.position.y, p.z = s.level;
            segment_arrow_->points.push_back ( p );
            switch ( s.type ) {
            case ShmFw::SegmentAGV::TYPE_LINE:
                segment_arrow_->color.a = 1.0f,  segment_arrow_->color.r = 1.0, segment_arrow_->color.g = 0.0, segment_arrow_->color.b = 0.0;
                break;
            case ShmFw::SegmentAGV::TYPE_ARC:
                segment_arrow_->color.a = 1.0f,  segment_arrow_->color.r = 0.0, segment_arrow_->color.g = 0.0, segment_arrow_->color.b = 1.0;
                break;
            };
            pub_marker_.publish ( *segment_arrow_ );
        }

        if ( s.type == ShmFw::SegmentAGV::TYPE_LINE ) {

            ShmFw::Vector2<double> v ( s.end.position.x-s.start.position.x, s.end.position.y - s.start.position.y );
            double d = v.norm();
            ShmFw::Vector2<double> u = v/d;
            double l = carry_over;
            if ( carry_over == 0. ) {
                p1 = ShmFw::Point2D ( s.start.position.x + u.x * l, s.start.position.y + u.y * l );
            }
            while ( l < d ) {
                l+=resolution;
                ShmFw::Point2D p0 ( p1 );
                p1 = ShmFw::Point2D ( s.start.position.x + u.x * l, s.start.position.y + u.y * l );
                path.push_back ( ShmFw::Pose ( ShmFw::Pose2D ( p0,p1 ), s.level ) );
            }
            carry_over = l - d;


        }
        if ( s.type == ShmFw::SegmentAGV::TYPE_ARC ) {
            double dx0 = s.start.position.x - s.center.position.x;
            double dy0 = s.start.position.y - s.center.position.y;
            double a0 = atan2 ( dy0, dx0 );

            double dx1 = s.end.position.x - s.center.position.x;
            double dy1 = s.end.position.y - s.center.position.y;
            double a1 = atan2 ( dy1, dx1 );

            double r0 = sqrt ( dx0*dx0 + dy0*dy0 );
            double angle_resolution = resolution/r0;


            double a = a0 + carry_over/r0;
            if ( carry_over == 0. ) {
                ShmFw::Point2D p1 ( s.center.position.x + cos ( a ) * r0, s.center.position.y+sin ( a ) * r0 );
            }
            while ( a < a1 ) {
                a+=angle_resolution;
                ShmFw::Point2D p0 ( p1 );
                p1 = ShmFw::Point2D ( s.center.position.x + cos ( a ) * r0, s.center.position.y + sin ( a ) * r0 );
                path.push_back ( ShmFw::Pose ( ShmFw::Pose2D ( p0,p1 ), s.level ) );
            }
            carry_over = ( a - a1 ) * r0;
        }
    }
}
