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
        MoveBase(const std::string& name, const BT::NodeConfiguration& config, const ros::NodeHandle& root_nh,
                const ros::NodeHandle& decision_nh);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;
    private:
//    double patrol_list[1][3];
//    std::vector<double> patrol_list;
        std::vector<geometry_msgs::PoseStamped> patrol_list_;
        XmlRpc::XmlRpcValue patrol_list;
//    int i = 0;
        std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> mbf_client_;
        void Move_Base();
    };
} // BehaviorTree


#endif //SRC_MOVE_BASE_H
