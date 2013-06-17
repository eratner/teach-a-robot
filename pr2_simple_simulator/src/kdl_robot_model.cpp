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
 /** \author Benjamin Cohen */

#include <sbpl_manipulation_components/kdl_robot_model.h>
#include <ros/ros.h>

using namespace std;

namespace sbpl_arm_planner {

KDLRobotModel::KDLRobotModel() : fk_solver_(NULL), ik_vel_solver_(NULL), ik_solver_(NULL), pr2_ik_solver_(NULL) 
{
  chain_root_name_ = "torso_lift_link";
  chain_tip_name_ = "r_wrist_roll_link";
}

KDLRobotModel::~KDLRobotModel()
{
  if(fk_solver_)
    delete fk_solver_;
  if(ik_vel_solver_)
    delete ik_vel_solver_;
  if(ik_solver_)
    delete fk_solver_;
  if(pr2_ik_solver_)
    delete pr2_ik_solver_;
}

bool KDLRobotModel::init(std::string robot_description, std::vector<std::string> planning_joints)
{
  urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
  if (!urdf_->initString(robot_description))
  {
    ROS_ERROR("Failed to parse the URDF.");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(*urdf_, ktree_))
  {
    ROS_ERROR("Failed to parse the kdl tree from robot description.");
    return false;
  }

  if(!ktree_.getChain(chain_root_name_, chain_tip_name_, kchain_))
  {
    ROS_ERROR("Failed to fetch the KDL chain for the robot. (root: %s, tip: %s)", chain_root_name_.c_str(), chain_tip_name_.c_str());
    return false;
  }

  // FK solver
  fk_solver_ = new KDL::ChainFkSolverPos_recursive(kchain_);
  jnt_pos_in_.resize(kchain_.getNrOfJoints());
  jnt_pos_out_.resize(kchain_.getNrOfJoints());

  // IK solver
  ik_vel_solver_ = new KDL::ChainIkSolverVel_pinv(kchain_);
  ik_solver_ = new KDL::ChainIkSolverPos_NR(kchain_, *fk_solver_, *ik_vel_solver_, 200);
  
  planning_joints_ = planning_joints;
  min_limits_.resize(planning_joints.size());
  max_limits_.resize(planning_joints.size());
  continuous_.resize(planning_joints.size());
  if(!getJointLimits(planning_joints_, min_limits_, max_limits_, continuous_))
  {
    ROS_ERROR("Failed to get the joint limits.");
    return false;
  }

  // joint name -> index mapping
  for(size_t i = 0, j = 0; i < kchain_.getNrOfSegments(); ++i)
  {
    if(kchain_.getSegment(i).getJoint().getName() == planning_joints_[j])
    {
      joint_map_[planning_joints_[j]] = i;
      j++;
    }
  }

  std::map<std::string, int>::iterator it;
  for(it = joint_map_.begin(); it != joint_map_.end(); ++it)
    ROS_INFO("%s --> %d", (it->first).c_str(), it->second);

  // for(size_t i = 0; i < planning_joints_.size(); ++i)
  //   joint_map_[planning_joints_[i]] = i;


  // PR2 Specific IK Solver
  pr2_ik_solver_ = new pr2_arm_kinematics::PR2ArmIKSolver(*urdf_, "torso_lift_link", "r_wrist_roll_link", 0.02, 2);
  if(!pr2_ik_solver_->active_)
  {
    ROS_ERROR("The pr2 IK solver is NOT active. Exiting.");
    return false;
  }

  initialized_ = true;
  return true;
}

bool KDLRobotModel::getJointLimits(std::vector<std::string> &joint_names, std::vector<double> &min_limits, std::vector<double> &max_limits, std::vector<bool> &continuous)
{
  for(size_t i = 0; i < joint_names.size(); ++i)
  {
    bool c;
    if(!getJointLimits(joint_names[i], min_limits[i], max_limits[i], c))
    {
      ROS_ERROR("Joint limits were not found for %s.", joint_names[i].c_str());
      return false;
    }
    continuous[i] = c;
  }
  return true;
}

bool KDLRobotModel::getJointLimits(std::string joint_name, double &min_limit, double &max_limit, bool &continuous)
{
  bool found_joint = false;
  boost::shared_ptr<const urdf::Link> link = urdf_->getLink(chain_tip_name_);
  while(link && (link->name != chain_root_name_) && !found_joint)
  {
    boost::shared_ptr<const urdf::Joint> joint = urdf_->getJoint(link->parent_joint->name);
    if(joint->name.compare(joint_name) == 0)
    {
      if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      {
        if(joint->type != urdf::Joint::CONTINUOUS)
        {
          continuous = false;

          if(joint->safety == NULL)
          {
            min_limit = joint->limits->lower;
            max_limit = joint->limits->upper;
          }
          else
          {
            min_limit = joint->safety->soft_lower_limit;
            max_limit = joint->safety->soft_upper_limit;
          }
        }
        else
        {
          min_limit = -M_PI;
          max_limit = M_PI;
          continuous = true;
        }
      }
      found_joint = true;
    }
    link = urdf_->getLink(link->getParent()->name);
  }
  return found_joint;
}

bool KDLRobotModel::checkJointLimits(const std::vector<double> &angles)
{
  // std::vector<double> a = angles;
  // if(!sbpl::interp::NormalizeAnglesIntoRange(a, min_limits_, max_limits_))
  // {
  //   ROS_ERROR("Joint angles are out of bounds.");  
  //   return false;
  // }

  return true;
}

bool KDLRobotModel::computeFK(const std::vector<double> &angles, std::string name, KDL::Frame &f)
{
  for(size_t i = 0; i < angles.size(); ++i)
    jnt_pos_in_(i) = angles::normalize_angle(angles[i]);

  KDL::Frame f1;
  if(fk_solver_->JntToCart(jnt_pos_in_, f1, joint_map_[name]) < 0)
  {
    ROS_ERROR("JntToCart returned < 0.");
    return false;
  }

  //f = f1;
  f = T_kinematics_to_planning_ * f1;
  return true;
}

bool KDLRobotModel::computeFK(const std::vector<double> &angles, std::string name, std::vector<double> &pose)
{
  KDL::Frame f;
  pose.resize(6);
  if(computeFK(angles, name, f))
  {
    pose[0] = f.p[0];
    pose[1] = f.p[1];
    pose[2] = f.p[2];
    f.M.GetRPY(pose[3], pose[4], pose[5]);
    return true;
  }
  return false;
}

bool KDLRobotModel::computePlanningLinkFK(const std::vector<double> &angles, std::vector<double> &pose)
{
  KDL::Frame f, f1;
  pose.resize(6);
  for(size_t i = 0; i < angles.size(); ++i)
    jnt_pos_in_(i) = angles::normalize_angle(angles[i]);

  if(fk_solver_->JntToCart(jnt_pos_in_, f1, /*joint_map_[planning_link_]*/8) < 0)
  {
    ROS_ERROR("JntToCart returned < 0.");
    return false;
  }

  //f = f1;
  f = T_kinematics_to_planning_ * f1;

  pose[0] = f.p[0];
  pose[1] = f.p[1];
  pose[2] = f.p[2];
  f.M.GetRPY(pose[3], pose[4], pose[5]);
  return true;
}

bool KDLRobotModel::computeIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution)
{
  //return computeFastIK(pose, start, solution);

  //pose: {x,y,z,r,p,y} or {x,y,z,qx,qy,qz,qw}
  KDL::Frame frame_des;
  frame_des.p.x(pose[0]);
  frame_des.p.y(pose[1]);
  frame_des.p.z(pose[2]);

  // RPY
  if(pose.size() == 6)
    frame_des.M = KDL::Rotation::RPY(pose[3],pose[4],pose[5]);
  // quaternion
  else
    frame_des.M = KDL::Rotation::Quaternion(pose[3],pose[4],pose[5],pose[6]);

  // transform into kinematics frame
  frame_des = T_planning_to_kinematics_ * frame_des;

  // seed configuration
  for(size_t i = 0; i < start.size(); i++)
    jnt_pos_in_(i) = angles::normalize_angle(start[i]); // must be normalized for CartToJntSearch

  if(pr2_ik_solver_->CartToJnt(jnt_pos_in_, frame_des, jnt_pos_out_) < 0)
    return false;

  solution.resize(start.size());
  for(size_t i = 0; i < solution.size(); ++i)
    solution[i] = jnt_pos_out_(i);

  return true;
}

bool KDLRobotModel::computeFastIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution)
{
  //pose: {x,y,z,r,p,y} or {x,y,z,qx,qy,qz,qw}

  KDL::Frame frame_des;
  frame_des.p.x(pose[0]);
  frame_des.p.y(pose[1]);
  frame_des.p.z(pose[2]);

  // RPY
  if(pose.size() == 6)
    frame_des.M = KDL::Rotation::RPY(pose[3],pose[4],pose[5]);
  // quaternion
  else
    frame_des.M = KDL::Rotation::Quaternion(pose[3],pose[4],pose[5],pose[6]);

  // transform into kinematics frame
  frame_des = T_planning_to_kinematics_ * frame_des;

  // seed configuration
  for(size_t i = 0; i < start.size(); i++)
    jnt_pos_in_(i) = angles::normalize_angle(start[i]); // must be normalized for CartToJntSearch

  if(ik_solver_->CartToJnt(jnt_pos_in_, frame_des, jnt_pos_out_) < 0)
    return false;

  solution.resize(start.size());
  for(size_t i = 0; i < solution.size(); ++i)
    solution[i] = jnt_pos_out_(i);

  return true;
}

void KDLRobotModel::printKinematicModelInformation(std::string stream)
{
  ROS_ERROR("Function not filled in.");  
}

}
