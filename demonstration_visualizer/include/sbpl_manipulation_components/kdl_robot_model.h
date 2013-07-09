/*
 * Copyright (c) 2010, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#ifndef _KDL_ROBOT_MODEL_
#define _KDL_ROBOT_MODEL_

#include <string>
#include <vector>
#include <ros/console.h>
#include <angles/angles.h>
#include <urdf/model.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
//#include <sbpl_geometry_utils/interpolation.h>
#include <sbpl_manipulation_components/robot_model.h>
#include <pr2_arm_kinematics/pr2_arm_ik_solver.h>

using namespace std;

namespace sbpl_arm_planner {

class KDLRobotModel : public RobotModel {

  public:

    KDLRobotModel();
    
    ~KDLRobotModel();
   
    /* Initialization */
    virtual bool init(std::string robot_description, std::vector<std::string> planning_joints);

    bool getJointLimits();

    /* Joint Limits */
    virtual bool checkJointLimits(const std::vector<double> &angles);
   
    /* Forward Kinematics */
    virtual bool computeFK(const std::vector<double> &angles, std::string name, KDL::Frame &f);

    virtual bool computeFK(const std::vector<double> &angles, std::string name, std::vector<double> &pose);

    virtual bool computePlanningLinkFK(const std::vector<double> &angles, std::vector<double> &pose);

    /* Inverse Kinematics */
    virtual bool computeIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution);

    virtual bool computeFastIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution);

    /* Debug Output */
    virtual void printKinematicModelInformation(std::string stream);

  private:

    boost::shared_ptr<urdf::Model> urdf_;

    std::string chain_root_name_;
    std::string chain_tip_name_;

    KDL::Tree ktree_;
    KDL::Chain kchain_;
    KDL::JntArray jnt_pos_in_;
    KDL::JntArray jnt_pos_out_;
    KDL::Frame p_out_;
    KDL::ChainFkSolverPos_recursive *fk_solver_;
    KDL::ChainIkSolverVel_pinv *ik_vel_solver_;
    KDL::ChainIkSolverPos_NR *ik_solver_;
    pr2_arm_kinematics::PR2ArmIKSolver* pr2_ik_solver_;

    std::vector<bool> continuous_;
    std::vector<double> min_limits_;
    std::vector<double> max_limits_;
    std::map<std::string, int> joint_map_;

    bool getJointLimits(std::vector<std::string> &joint_names, std::vector<double> &min_limits, std::vector<double> &max_limits, std::vector<bool> &continuous);
    bool getJointLimits(std::string joint_name, double &min_limit, double &max_limit, bool &continuous);
};

}
#endif
