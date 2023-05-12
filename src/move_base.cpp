//
// Created by yawara on 23-5-3.
//
#include "move_base.h"
using namespace BT;
namespace BehaviorTree {
    MoveBase::MoveBase(const std::string &name, const BT::NodeConfiguration &config, const ros::NodeHandle &root_nh,
                   const ros::NodeHandle &decision_nh)
            : BT::SyncActionNode(name, config){
        mbf_client_ =
                std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
        ros::NodeHandle patrol_nh(decision_nh, "patrol");
        decision_nh.getParam("/behavior/behavior_list", patrol_list);
    }

    void MoveBase::init(){
        if((DetectState == SUCCEEDED)||(rand_index == patrol_list[goal_list_index].size() - 1)) {
            goal_list_index++;
            rand_index = 0;
            DetectState = SUCCEEDED;
        }
        for (int i = 0; (i < patrol_list[goal_list_index].size())&&(DetectState == SUCCEEDED); ++i) {
            ROS_ASSERT(patrol_list[goal_list_index][i].getType() == XmlRpc::XmlRpcValue::TypeArray);
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "map";

            ROS_ASSERT(patrol_list[goal_list_index][i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble and
                       patrol_list[goal_list_index][i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble and
                       patrol_list[goal_list_index][i][2].getType() == XmlRpc::XmlRpcValue::TypeDouble);

            pose_stamped.pose.position.x = static_cast<double>(patrol_list[goal_list_index][i][0]);
            pose_stamped.pose.position.y = static_cast<double>(patrol_list[goal_list_index][i][1]);
            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, static_cast<double>(patrol_list[goal_list_index][i][2]));
            pose_stamped.pose.orientation = toMsg(quat);

            patrol_list_.push_back(pose_stamped);
        }
        DetectState = FAILURE;
    }
    void MoveBase::Move_Base(){
        if (actionlib::SimpleClientGoalState::ACTIVE == mbf_client_->getState().state_) {
            // do nothing
        } else {
            ros::Rate loop_rate(5.0/3.0);
            loop_rate.sleep();
            //rand() % (patrol_list_.size());
            move_base_msgs::MoveBaseGoal mbf_goal;
            mbf_goal.target_pose = patrol_list_[rand_index];
            mbf_client_->sendGoal(mbf_goal);
            if(rand_index < (patrol_list[goal_list_index].size() - 1))
                rand_index++;
        }
    }

    BT::PortsList MoveBase::providedPorts(){
        // This action has a single input port called "goal"
//    return { BT::InputPort<Position2D>("goal") };
        BT::PortsList ports_list;
        return ports_list;
    }

    BT::NodeStatus MoveBase::tick(){
        ROS_INFO("BehaviorsTree, Start!!!");
        Move_Base();
        return BT::NodeStatus::SUCCESS;
    }

} // BehaviorTree

