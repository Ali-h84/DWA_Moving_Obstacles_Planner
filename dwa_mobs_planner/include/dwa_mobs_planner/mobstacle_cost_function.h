/*
 * mobstacle_cost_function.h
 * ROS wrapper for DWAMobsPlanner, developed at RAMP, SFU
 * 
 * Modified obstacle cost function from base_local_planner for DWA Mobs Planner
 */

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2014 Robotic Algorithms & Motion Planning Laboratory
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TKruse, Alireza Hekmati
 *********************************************************************/

#ifndef MOBSTACLE_COST_FUNCTION_H_
#define MOBSTACLE_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>

// for mobs planner
#include "ros/ros.h"
#include <string>
#include <boost/thread.hpp>
#include "dwa_mobs_planner/OdometryMovingObstacles.h"

namespace base_local_planner {

/**
 * class MObstacleCostFunction
 * @brief Uses obstacle information to assign negative costs if robot footprint
 * is in obstacle's trajectory on any point of the robot's trajectory.
 */
class MObstacleCostFunction : public TrajectoryCostFunction {

public:
    MObstacleCostFunction();
    ~MObstacleCostFunction();

    bool prepare();
    double scoreTrajectory( Trajectory &traj );

    void setSumScores( bool score_sums ){ sum_scores_=score_sums; }

    void setParams( double max_trans_vel, double max_scaling_factor, double scaling_speed );
    void setFootprint( std::vector<geometry_msgs::Point> footprint_spec );

    // helper functions, made static for easy unit testing
    static double getScalingFactor(
        Trajectory &traj,
        double scaling_speed,
        double max_trans_vel,
        double max_scaling_factor
    );
/*    static double footprintCost(
        const double& x,
        const double& y,
        const double& th,
        double scale,
        std::vector<geometry_msgs::Point> footprint_spec,
        costmap_2d::Costmap2D* costmap,
        base_local_planner::WorldModel* world_model
    );
*/
    // for mobs planner ->//
    void mcf_cb( const dwa_mobs_planner::OdometryMovingObstaclesConstPtr&);

private:
    std::vector<geometry_msgs::Point> footprint_spec_;
    double max_trans_vel_;
    bool sum_scores_;
    //footprint scaling with velocity;
    double max_scaling_factor_, scaling_speed_;

    ros::NodeHandle mcf_nh; // node handle
    ros::Subscriber sub_oo;
    std::string oo_i_t; // incoming obs odom topic
    dwa_mobs_planner::OdometryMovingObstacles mobs_c; // current mobs
    bool mobs_ready; // flag to indicate at least one msg was received
    tf::TransformListener tf_li_oo;
};

} /* namespace base_local_planner */
#endif /* OBSTACLE_COST_FUNCTION_H_ */
