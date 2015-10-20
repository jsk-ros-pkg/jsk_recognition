#ifndef JSK_PCL_ROS_SELF_MASK_URDF_ROBOT_
#define JSK_PCL_ROS_SELF_MASK_URDF_ROBOT_

#include <robot_self_filter/self_mask.h>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

namespace robot_self_filter
{
  class SelfMaskUrdfRobot : public SelfMask<pcl::PointXYZ>
  {
  public:
  SelfMaskUrdfRobot(tf::TransformListener &tfl,
                    tf::TransformBroadcaster &tfb,
                    const std::vector<LinkInfo> &links,
                    const urdf::Model &urdf_model,
                    std::string root_link_id = "BODY",
                    std::string world_frame_id = "map",
                    bool set_foot_pose = false)
    : SelfMask<pcl::PointXYZ>(tfl, links),
      tf_broadcaster_(tfb),
      urdf_model_(urdf_model),
      root_link_id_(root_link_id),
      world_frame_id_(world_frame_id)
    {
      ros::NodeHandle pnh("~");
      pnh.param("publish_tf", publish_tf_, false);
      initKdlConfigure();
    }

    bool initKdlConfigure()
    {
      // generate kdl tree
      if(!kdl_parser::treeFromUrdfModel(urdf_model_, tree_)) {
        ROS_FATAL("Failed to load robot_description");
        return false;
      }
      // generate kdl tree
      for(unsigned int i = 0 ; i < bodies_.size() ; i++) {
        std::string name = bodies_[i].name;
        tree_.getChain(root_link_id_, name, chain_map_[name]);
        for(size_t j = 0; j < chain_map_[name].getNrOfSegments(); j++) {
          ROS_DEBUG_STREAM("kdl_chain(" << j << ") "
                           << chain_map_[name].getSegment(j).getJoint().getName().c_str());
        }
      }
      return true;
    }

    void updateChain(std::map<std::string, double>& joint_angles,
                     KDL::Chain& chain,
                     tf::Pose& output_pose)
    {
      // solve forward kinematics
      KDL::JntArray jnt_pos(chain.getNrOfJoints());
      for (int i = 0, j = 0; i < chain.getNrOfSegments(); i++) {
        std::string joint_name = chain.getSegment(i).getJoint().getName();
        if (joint_angles.find(joint_name) != joint_angles.end()) {
          jnt_pos(j++) = joint_angles[joint_name];
        }
      }
      KDL::ChainFkSolverPos_recursive fk_solver(chain);
      KDL::Frame pose;
      if (fk_solver.JntToCart(jnt_pos, pose) < 0) {
        ROS_FATAL("Failed to compute FK");
      }
      tf::poseKDLToTF(pose, output_pose);
    }

    void updateRobotModel(std::map<std::string, double>& joint_angles, const tf::Pose& root_pose)
    {
      // for debug
      /* std::map<std::string, double> joint_angles; */
      /* for(std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = urdf_model_.joints_.begin(); it != urdf_model_.joints_.end(); ++it) { */
      /*   joint_angles[it->first] = 0.0; */
      /* } */

      for(std::map<std::string, double>::iterator it = joint_angles.begin(); it != joint_angles.end(); ++it) {
        ROS_DEBUG_STREAM("joint " << it->first << " : " << it->second);
      }

      for(unsigned int i = 0 ; i < bodies_.size() ; i++) {
        std::string name = bodies_[i].name;
        updateChain(joint_angles, chain_map_[name], pose_map_[name]);
        pose_map_[name] = root_pose * pose_map_[name];
      }

      // publish tf of all links
      if(publish_tf_) {
        std::vector<tf::StampedTransform> link_transforms;
        for(unsigned int i = 0 ; i < bodies_.size() ; i++) {
          std::string name = bodies_[i].name;
          tf::Pose link_trans = pose_map_[name];
          ros::Time stamp = ros::Time::now();
          link_transforms.push_back(tf::StampedTransform(link_trans, stamp, world_frame_id_, "collision_detector/"+name));
          ROS_DEBUG_STREAM("link trans [" << name <<
                           "] pos=( " << link_trans.getOrigin().getX() << ", " << link_trans.getOrigin().getY() << ", " << link_trans.getOrigin().getZ() <<
                           ")  rot=( " << link_trans.getRotation().getW() << ", " << link_trans.getRotation().getX() << ", " << link_trans.getRotation().getY() << ", " << link_trans.getRotation().getZ() << ")");
        }
        tf_broadcaster_.sendTransform(link_transforms);
      }
    }

    void assumeFrameFromJointAngle(const sensor_msgs::JointState& joint, const geometry_msgs::PoseStamped& pose)
    {
      std::map<std::string, double> joint_angles;
      for (size_t i = 0; i < joint.name.size(); i++) {
        joint_angles[joint.name[i]] = joint.position[i];
      }
      tf::Pose tf_pose;
      tf::poseMsgToTF(pose.pose, tf_pose);
      updateRobotModel(joint_angles, tf_pose);

      // update pose of all links
      for(int i = 0; i < bodies_.size(); i++) {
        std::string name = bodies_[i].name;
        // pose_map_ should be the transformation from world frame to link frame.
        bodies_[i].body->setPose(pose_map_[name] * bodies_[i].constTransf);
        bodies_[i].unscaledBody->setPose(pose_map_[name] * bodies_[i].constTransf);
      }

      computeBoundingSpheres();
    }

  protected:
    urdf::Model urdf_model_;
    KDL::Tree tree_;
    std::map<std::string, KDL::Chain> chain_map_;
    std::map<std::string, tf::Pose> pose_map_;
    tf::TransformBroadcaster &tf_broadcaster_;
    std::string world_frame_id_;
    std::string root_link_id_;
    bool publish_tf_;
  };

}

#endif
