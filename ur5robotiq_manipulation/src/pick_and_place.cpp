#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "manipulator_base_link";

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;

    collision_objects[0].operation = collision_objects[0].ADD;

    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "manipulator_base_link";

    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;

    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.2;

    collision_objects[1].operation = collision_objects[1].ADD;

    collision_objects[2].header.frame_id = "manipulator_base_link";
    collision_objects[2].id = "object";

    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.035;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;

    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

void openGripper(moveit::planning_interface::MoveGroupInterface& move_group)
{
    moveit::core::RobotStatePtr gripper_current_state;
    std::vector<double> gripper_joint_positions;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success;
    
    const robot_state::JointModelGroup *gripper_joint_model_group = move_group.getCurrentState()->getJointModelGroup("gripper");
    gripper_current_state = move_group.getCurrentState();
    gripper_current_state->copyJointGroupPositions(gripper_joint_model_group, gripper_joint_positions);
    gripper_joint_positions[0] = 0;
    move_group.setJointValueTarget(gripper_joint_positions);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        ros::Duration(1.0).sleep();
        move_group.move();
    }
}

void closeGripper(moveit::planning_interface::MoveGroupInterface& move_group)
{
    moveit::core::RobotStatePtr gripper_current_state;
    std::vector<double> gripper_joint_positions;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success;

    const robot_state::JointModelGroup *gripper_joint_model_group = move_group.getCurrentState()->getJointModelGroup("gripper");
    gripper_current_state = move_group.getCurrentState();
    gripper_current_state->copyJointGroupPositions(gripper_joint_model_group, gripper_joint_positions);
    gripper_joint_positions[0] = 0.5;
    move_group.setJointValueTarget(gripper_joint_positions);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        ros::Duration(1.0).sleep();
        move_group.move();
    }
}

void moveInitialPosition(moveit::planning_interface::MoveGroupInterface& move_group)
{
    geometry_msgs::Pose target_pose;
    tf2::Quaternion orientation;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group.setStartState(*move_group.getCurrentState());
    orientation.setRPY(-M_PI / 2, -M_PI / 2, -M_PI / 2);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = 0.5;
    target_pose.position.y = 0.2425;
    target_pose.position.z = 0.55;
    move_group.setPoseTarget(target_pose);
    move_group.plan(my_plan);
    move_group.move();
}

void moveFinalPosition(moveit::planning_interface::MoveGroupInterface& move_group)
{
    geometry_msgs::Pose target_pose;
    tf2::Quaternion orientation;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group.setStartState(*move_group.getCurrentState());
    orientation.setRPY(-M_PI / 2, -M_PI / 2, -M_PI / 2);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = 0.475;
    target_pose.position.y = 0.2475;
    target_pose.position.z = 0.575;
    move_group.setPoseTarget(target_pose);
    move_group.plan(my_plan);
    move_group.move();

    move_group.setStartState(*move_group.getCurrentState());
    orientation.setRPY(-M_PI / 2, -M_PI / 2, M_PI / 2);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = 0;
    target_pose.position.y = 0.25;
    target_pose.position.z = 0.55;
    move_group.setPoseTarget(target_pose);
    move_group.plan(my_plan);
    move_group.move();
}

void pick(moveit::planning_interface::MoveGroupInterface& manipulator, moveit::planning_interface::MoveGroupInterface& gripper)
{
    gripper.setSupportSurfaceName("table1");
    manipulator.setSupportSurfaceName("table1");
    moveInitialPosition(manipulator);
    closeGripper(gripper);
    gripper.attachObject("object");
}

void place(moveit::planning_interface::MoveGroupInterface& manipulator, moveit::planning_interface::MoveGroupInterface& gripper)
{
    gripper.setSupportSurfaceName("table2");
    manipulator.setSupportSurfaceName("table2");
    moveFinalPosition(manipulator);
    gripper.detachObject("object");
    openGripper(gripper);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_to_pose");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface manipulator("manipulator");
    moveit::planning_interface::MoveGroupInterface gripper("gripper");
    manipulator.setPlanningTime(10.0);
    gripper.setPlanningTime(10.0);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    addCollisionObjects(planning_scene_interface);
    ros::WallDuration(1.0).sleep();
    
    pick(manipulator, gripper);
    place(manipulator, gripper);

    ros::shutdown();
    return 0;
}

