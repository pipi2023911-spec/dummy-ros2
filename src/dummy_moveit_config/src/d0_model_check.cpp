// SPDX-License-Identifier: GPL-3.0-only
// D0 diagnostic only. Never sends trajectories or hardware commands.
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <cmath>
#include <iostream>
#include <vector>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "d0_model_check",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  const auto model = loader.getModel();
  if (!model) {
    std::cerr << "FAIL: MoveIt RobotModel could not load\n";
    rclcpp::shutdown();
    return 1;
  }
  const auto* group = model->getJointModelGroup("dummy_arm");
  const std::vector<std::string> expected{
      "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
  if (!group || group->getVariableNames() != expected ||
      !model->hasLinkModel("link6_1_1")) {
    std::cerr << "FAIL: group variables or tip differ from audited model\n";
    rclcpp::shutdown();
    return 1;
  }
  // A model can load while a mesh fails to load. Never report collision PASS
  // unless all seven expected physical links have actual collision shapes.
  const std::vector<std::string> physical_links{
      "base_link", "link1_1_1", "link2_1_1", "link3_1_1",
      "link4_1_1", "link5_1_1", "link6_1_1"};
  for (const auto& name : physical_links) {
    const auto* link = model->getLinkModel(name);
    if (!link || link->getShapes().empty()) {
      std::cerr << "FAIL: missing collision geometry for " << name << '\n';
      rclcpp::shutdown();
      return 1;
    }
    for (const auto& shape : link->getShapes()) {
      if (!shape) {
        std::cerr << "FAIL: null collision shape for " << name << '\n';
        rclcpp::shutdown();
        return 1;
      }
    }
  }
  planning_scene::PlanningScene scene(model);
  auto& state = scene.getCurrentStateNonConst();
  bool valid = true;
  for (const std::string pose : {"home", "reset"}) {
    state.setToDefaultValues();
    if (!state.setToDefaultValues(group, pose)) {
      std::cerr << "FAIL: missing named state " << pose << '\n';
      valid = false;
      continue;
    }
    state.update();
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 100;
    collision_detection::CollisionResult result;
    scene.checkSelfCollision(request, result, state);
    const bool bounds = state.satisfiesBounds(group);
    std::cout << "pose=" << pose << " bounds=" << bounds
              << " self_collision=" << result.collision
              << " tip_xyz=" << state.getGlobalLinkTransform("link6_1_1")
                                    .translation().transpose() << '\n';
    for (const auto& contact : result.contacts)
      std::cout << "contact=" << contact.first.first << ","
                << contact.first.second << '\n';
    valid = valid && bounds && !result.collision;
  }
  // Check FK changes for all six variables, including orientation-only effects.
  state.setToDefaultValues();
  state.update();
  const Eigen::Matrix4d initial =
      state.getGlobalLinkTransform("link6_1_1").matrix();
  for (const auto& joint : expected) {
    state.setToDefaultValues();
    state.setVariablePosition(joint, 0.1);
    state.update();
    const double change =
        (state.getGlobalLinkTransform("link6_1_1").matrix() - initial).norm();
    std::cout << "joint=" << joint << " delta_rad=0.1 fk_matrix_delta="
              << change << '\n';
    valid = valid && std::isfinite(change) && change > 1e-8;
  }
  std::cout << "group=dummy_arm dof=" << group->getVariableCount()
            << " root=" << model->getRootLinkName()
            << " tip=link6_1_1 result=" << (valid ? "PASS" : "FAIL")
            << "\nOnly two poses checked with inherited SRDF exclusions; "
               "this is not a full self-collision matrix validation.\n";
  rclcpp::shutdown();
  return valid ? 0 : 2;
}
