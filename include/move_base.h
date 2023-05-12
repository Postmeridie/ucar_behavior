//
// Created by yawara on 23-5-3.
//

#ifndef SRC_MOVE_BASE_H
#define SRC_MOVE_BASE_H

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace BehaviorTree {
    class MoveBase : public BT::SyncActionNode{
    public:
        enum DetectResult{
            SUCCEEDED,
            FAILURE
        };
        DetectResult DetectState = SUCCEEDED;
        MoveBase(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh,
                const ros::NodeHandle& decision_nh);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;


    private:
        int goal_list_index = -1;
        int rand_index = 0;
        std::vector<geometry_msgs::PoseStamped> patrol_list_;
        XmlRpc::XmlRpcValue patrol_list;
//    int i = 0;
        std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> mbf_client_;
        void init();
        void Move_Base();
    };
} // BehaviorTree


#endif //SRC_MOVE_BASE_H
