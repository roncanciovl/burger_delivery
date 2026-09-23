// Copyright 2026 Henry Roncancio — Universidad Militar Nueva Granada
// SPDX-License-Identifier: Apache-2.0
//
// Pick & place de la caja de hamburguesa con MoveIt Task Constructor (TODO.md §2,
// burger_control). Adaptado del tutorial oficial de MTC al Kinova Gen3 de 6 GDL con pinza
// Robotiq 2F-85 (grupos "manipulator" y "gripper" del paquete
// kinova_gen3_6dof_robotiq_2f_85_moveit_config).
//
// Dónde está la caja:
//   * si existe el TF <planning_frame> -> <object_frame> (p. ej. target_burger_box_frame, que
//     publica el nodo Gemini, o un tag), se usa esa pose;
//   * si no aparece en object_tf_timeout_s, se usa el parámetro object_xyz.
//
// SEGURIDAD: execute:=false por defecto. El nodo sólo PLANIFICA y publica la solución para
// verla en RViz (panel "Motion Planning Tasks"). Para mover el robot hay que pedir
// execute:=true de forma explícita, con la velocidad escalada por velocity_scaling (0.1).

#include <Eigen/Geometry>

#include <cmath>

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mtc = moveit::task_constructor;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("burger_pick_place");
constexpr char kObjectId[] = "burger_box";
}  // namespace

class PickPlaceNode
{
public:
  explicit PickPlaceNode(const rclcpp::NodeOptions & options)
  : node_{std::make_shared<rclcpp::Node>("burger_pick_place", options)}
  {
    arm_group_ = param<std::string>("arm_group", "manipulator");
    hand_group_ = param<std::string>("hand_group", "gripper");
    hand_frame_ = param<std::string>("hand_frame", "end_effector_link");
    planning_frame_ = param<std::string>("planning_frame", "base_link");
    object_frame_ = param<std::string>("object_frame", "target_burger_box_frame");
    object_tf_timeout_s_ = param<double>("object_tf_timeout_s", 2.0);
    object_xyz_ = param<std::vector<double>>("object_xyz", {0.45, -0.20, 0.03});
    object_size_ = param<std::vector<double>>("object_size", {0.10, 0.10, 0.06});
    place_xyz_ = param<std::vector<double>>("place_xyz", {0.45, 0.20, 0.03});
    tcp_offset_ = param<double>("tcp_offset_m", 0.15);
    open_state_ = param<std::string>("hand_open_state", "Open");
    close_state_ = param<std::string>("hand_close_state", "Close");
    home_state_ = param<std::string>("arm_home_state", "Home");
    velocity_scaling_ = param<double>("velocity_scaling", 0.1);
    execute_ = param<bool>("execute", false);
    max_solutions_ = static_cast<int>(param<int64_t>("max_solutions", 5));
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  {
    return node_->get_node_base_interface();
  }

  // Añade la caja a la escena de planificación, en el TF observado o en object_xyz.
  void setupPlanningScene()
  {
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = object_xyz_.at(0);
    pose.position.y = object_xyz_.at(1);
    pose.position.z = object_xyz_.at(2);

    tf2_ros::Buffer buffer(node_->get_clock());
    tf2_ros::TransformListener listener(buffer);
    try {
      const auto tf = buffer.lookupTransform(
        planning_frame_, object_frame_, tf2::TimePointZero,
        tf2::durationFromSec(object_tf_timeout_s_));
      pose.position.x = tf.transform.translation.x;
      pose.position.y = tf.transform.translation.y;
      pose.position.z = tf.transform.translation.z;
      // Sólo el giro alrededor de z: la caja está apoyada en la mesa.
      const auto & q = tf.transform.rotation;
      const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      pose.orientation.z = std::sin(yaw / 2.0);
      pose.orientation.w = std::cos(yaw / 2.0);
      RCLCPP_INFO(LOGGER, "Caja tomada del TF %s -> %s: (%.3f, %.3f, %.3f) m",
        planning_frame_.c_str(), object_frame_.c_str(),
        pose.position.x, pose.position.y, pose.position.z);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(LOGGER, "Sin TF %s -> %s (%s); se usa object_xyz (%.3f, %.3f, %.3f) m",
        planning_frame_.c_str(), object_frame_.c_str(), e.what(),
        pose.position.x, pose.position.y, pose.position.z);
    }

    moveit_msgs::msg::CollisionObject object;
    object.id = kObjectId;
    object.header.frame_id = planning_frame_;
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {object_size_.at(0), object_size_.at(1), object_size_.at(2)};
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
  }

  void doTask()
  {
    task_ = createTask();
    try {
      task_.init();
    } catch (mtc::InitStageException & e) {
      RCLCPP_ERROR_STREAM(LOGGER, "Error al inicializar la tarea: " << e);
      return;
    }
    if (!task_.plan(max_solutions_)) {
      RCLCPP_ERROR(LOGGER, "La planificación falló: revisa las etapas en RViz");
      return;
    }
    task_.introspection().publishSolution(*task_.solutions().front());
    RCLCPP_INFO(LOGGER, "Plan listo (%zu soluciones). Costo de la mejor: %.3f",
      task_.solutions().size(), task_.solutions().front()->cost());

    if (!execute_) {
      RCLCPP_WARN(LOGGER, "execute:=false: el robot NO se mueve. Revisa el plan en RViz y "
        "relanza con execute:=true para ejecutarlo.");
      return;
    }
    const auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(LOGGER, "La ejecución falló (código %d)", result.val);
      return;
    }
    RCLCPP_INFO(LOGGER, "Pick & place completado");
  }

private:
  template<typename T>
  T param(const std::string & name, const T & default_value)
  {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_value);
    }
    return node_->get_parameter(name).get_value<T>();
  }

  mtc::Task createTask()
  {
    mtc::Task task;
    task.stages()->setName("burger pick & place");
    task.loadRobotModel(node_);
    task.setProperty("group", arm_group_);
    task.setProperty("eef", hand_group_);
    task.setProperty("ik_frame", hand_frame_);

    mtc::Stage * current_state_ptr = nullptr;
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    for (const auto & planner : std::vector<mtc::solvers::PlannerInterfacePtr>{
        sampling_planner, interpolation_planner, cartesian_planner})
    {
      planner->setMaxVelocityScalingFactor(velocity_scaling_);
      planner->setMaxAccelerationScalingFactor(velocity_scaling_);
    }
    cartesian_planner->setStepSize(.01);

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_);
      stage->setGoal(open_state_);
      task.add(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_, sampling_planner}});
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage));
    }

    const auto hand_links = task.getRobotModel()->getJointModelGroup(hand_group_)
      ->getLinkModelNamesWithCollisionGeometry();

    mtc::Stage * attach_object_stage = nullptr;
    {
      auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
      task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
      grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
      {
        // Aproximación a lo largo del eje z de la herramienta (hacia la caja).
        auto stage = std::make_unique<mtc::stages::MoveRelative>(
          "approach object", cartesian_planner);
        stage->properties().set("marker_ns", "approach_object");
        stage->properties().set("link", hand_frame_);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        stage->setMinMaxDistance(0.05, 0.12);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame_;
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }
      {
        auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "grasp_pose");
        stage->setPreGraspPose(open_state_);
        stage->setObject(kObjectId);
        stage->setAngleDelta(M_PI / 12);
        stage->setMonitoredStage(current_state_ptr);

        // Agarre desde arriba: el punto de agarre (TCP) está a tcp_offset_m sobre el eje z
        // de la herramienta, y ese eje apunta hacia abajo (-z de la caja).
        Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
        grasp_frame_transform.linear() =
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
        grasp_frame_transform.translation().z() = tcp_offset_;

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(8);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(grasp_frame_transform, hand_frame_);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        grasp->insert(std::move(wrapper));
      }
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "allow collision (hand,object)");
        stage->allowCollisions(kObjectId, hand_links, true);
        grasp->insert(std::move(stage));
      }
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
        stage->setGroup(hand_group_);
        stage->setGoal(close_state_);
        grasp->insert(std::move(stage));
      }
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
        stage->attachObject(kObjectId, hand_frame_);
        attach_object_stage = stage.get();
        grasp->insert(std::move(stage));
      }
      {
        auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        stage->setMinMaxDistance(0.05, 0.15);
        stage->setIKFrame(hand_frame_);
        stage->properties().set("marker_ns", "lift_object");
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = planning_frame_;
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        grasp->insert(std::move(stage));
      }
      task.add(std::move(grasp));
    }
    {
      auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_, sampling_planner},
          {hand_group_, interpolation_planner}});
      stage->setTimeout(5.0);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      task.add(std::move(stage));
    }
    {
      auto place = std::make_unique<mtc::SerialContainer>("place object");
      task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
      place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
      {
        auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
        stage->setObject(kObjectId);
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = planning_frame_;
        target.pose.position.x = place_xyz_.at(0);
        target.pose.position.y = place_xyz_.at(1);
        target.pose.position.z = place_xyz_.at(2);
        target.pose.orientation.w = 1.0;
        stage->setPose(target);
        stage->setMonitoredStage(attach_object_stage);

        auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(4);
        wrapper->setMinSolutionDistance(1.0);
        wrapper->setIKFrame(kObjectId);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
        place->insert(std::move(wrapper));
      }
      {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
        stage->setGroup(hand_group_);
        stage->setGoal(open_state_);
        place->insert(std::move(stage));
      }
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
          "forbid collision (hand,object)");
        stage->allowCollisions(kObjectId, hand_links, false);
        place->insert(std::move(stage));
      }
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
        stage->detachObject(kObjectId, hand_frame_);
        place->insert(std::move(stage));
      }
      {
        // Retirada hacia arriba, lejos de la caja recién depositada.
        auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
        stage->setMinMaxDistance(0.05, 0.15);
        stage->setIKFrame(hand_frame_);
        stage->properties().set("marker_ns", "retreat");
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = planning_frame_;
        vec.vector.z = 1.0;
        stage->setDirection(vec);
        place->insert(std::move(stage));
      }
      task.add(std::move(place));
    }
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setGoal(home_state_);
      task.add(std::move(stage));
    }
    return task;
  }

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  std::string arm_group_, hand_group_, hand_frame_, planning_frame_, object_frame_;
  std::string open_state_, close_state_, home_state_;
  std::vector<double> object_xyz_, object_size_, place_xyz_;
  double object_tf_timeout_s_{2.0}, tcp_offset_{0.15}, velocity_scaling_{0.1};
  bool execute_{false};
  int max_solutions_{5};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // moveit_configs llega como parámetros (robot_description, semantic, kinematics...).
  options.automatically_declare_parameters_from_overrides(true);

  auto pick_place = std::make_shared<PickPlaceNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &pick_place]() {
      executor.add_node(pick_place->getNodeBaseInterface());
      executor.spin();
      executor.remove_node(pick_place->getNodeBaseInterface());
    });

  pick_place->setupPlanningScene();
  pick_place->doTask();

  // Se deja el nodo vivo para inspeccionar la solución en RViz; Ctrl+C para salir.
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
