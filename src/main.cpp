//
// Created by yawara on 23-5-3.
//
#include "move_base.h"

static const char* xml_text = R"(
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <MovetoGoal />
        </Sequence>
    </BehaviorTree>
</root>
)";

//<MovetoFirstGoal  goal="4.0,5.0,-1.57" />
//
////            <MovetoSecondGoal goal="2.5,2.5,-2.356" />  goal="4.0,0.5,-1.57"
////            <MovetoThirdGoal  goal="1.7,1.5,-1.57" />
////            <MovetoForthGoal  goal="0.0,0.0,-3.0" />
//

int main(int argc, char** argv) {
    ros::init(argc, argv, "run_bt");
    ros::NodeHandle root_nh;
    ros::NodeHandle decision_nh("~");
    BT::BehaviorTreeFactory factory_;
    //std::string file_path = decision_nh.param("file_path", std::string(" "));

    BT::NodeBuilder builder_calculate = [&root_nh, &decision_nh](const std::string& name,
                                                                 const BT::NodeConfiguration& config) {
        return std::make_unique<BehaviorTree::MoveBase>(name, config, root_nh, decision_nh);
    };
    factory_.registerBuilder<BehaviorTree::MoveBase>("MovetoGoal", builder_calculate);
//    factory_.registerBuilder<BehaviorsTree::Patrol>("MovetoFirstGoal", builder_calculate);
//    factory_.registerBuilder<BehaviorsTree::Patrol>("MovetoSecondGoal", builder_calculate);
//    factory_.registerBuilder<BehaviorsTree::Patrol>("MovetoThirdGoal", builder_calculate);
//    factory_.registerBuilder<BehaviorsTree::Patrol>("MovetoForthGoal", builder_calculate);

    auto tree = factory_.createTreeFromText(xml_text);
    ros::Rate loop_rate(20);

    while(ros::ok()){
        tree.tickRoot();
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}