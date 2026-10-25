#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ignition/gazebo/Actor.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Actor.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/rendering/Events.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/Visual.hh>

namespace jackal_actor_controller
{
namespace gz = ignition::gazebo;
using Pose = ignition::math::Pose3d;
using Message = geometry_msgs::msg::PoseStamped;

// Callbacks run only in PreUpdate's spin_some: no ROS thread may touch the ECM.
class ActorController final : public gz::System,
  public gz::ISystemConfigure, public gz::ISystemPreUpdate
{
  struct Command
  {
    int64_t stamp;
    double x, y, yaw;
  };

  struct Binding
  {
    std::string id, name, topic, animation;
    double yawOffset{0};
    gz::Entity entity{gz::kNullEntity};
    Pose origin, initial, target;
    bool active{false}, moving{false};
    int64_t lastStamp{-1}, lastApplied{-1};
    std::chrono::steady_clock::duration animationTime{0};
    std::deque<Command> pending;
    std::vector<rclcpp::Subscription<Message>::SharedPtr> subscriptions;
    rclcpp::Publisher<Message>::SharedPtr groundTruth;
  };

  struct ObservationBinding
  {
    std::string name;
    rclcpp::Publisher<Message>::SharedPtr publisher;
    int64_t lastStamp{-1};
  };
  struct Observations
  {
    std::mutex mutex;
    bool enabled{true};
    std::string frame;
    std::vector<ObservationBinding> bindings;
  };

public:
  ~ActorController() override
  {
    this->renderConnection.reset();
    if (this->observations)
    {
      std::lock_guard<std::mutex> lock(this->observations->mutex);
      this->observations->enabled = false;
      this->observations->bindings.clear();
    }
    if (this->executor && this->node)
      this->executor->remove_node(this->node);
    this->bindings.clear();
    this->node.reset();
    this->executor.reset();
    if (this->context && this->context->is_valid())
      this->context->shutdown("ActorController unloaded");
  }

  void Configure(const gz::Entity &world, const std::shared_ptr<const sdf::Element> &sdf,
      gz::EntityComponentManager &, gz::EventManager &events) override
  {
    this->world = world;
    this->context = std::make_shared<rclcpp::Context>();
    this->context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(this->context).use_global_arguments(false);
    this->node = std::make_shared<rclcpp::Node>("human_actor_controller", options);
    rclcpp::ExecutorOptions executorOptions;
    executorOptions.context = this->context;
    this->executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executorOptions);
    this->executor->add_node(this->node);
    auto config = sdf->Clone();
    this->frame = config->Get<std::string>("frame_id", "world").first;
    this->timeout = config->Get<double>("animation_timeout", 0.3).first;
    if (this->frame.empty() || !std::isfinite(this->timeout) || this->timeout <= 0)
      throw std::runtime_error("Invalid frame_id or animation_timeout");
    std::set<std::string> names, ids, topics;
    if (!config->HasElement("mapping"))
      throw std::runtime_error("ActorController requires at least one <mapping>");
    for (auto element = config->GetElement("mapping"); element;
         element = element->GetNextElement("mapping"))
    {
      auto binding = std::make_shared<Binding>();
      binding->id = element->Get<std::string>("person_id");
      binding->name = element->Get<std::string>("actor_name");
      binding->topic = element->Get<std::string>("topic",
        "/humans/" + binding->id + "/target_pose").first;
      binding->animation = element->Get<std::string>("animation", "walk").first;
      binding->yawOffset = element->Get<double>("yaw_offset", 0.0).first;
      if (binding->id.empty() || binding->name.empty() || binding->animation.empty() ||
          !std::isfinite(binding->yawOffset) || !ids.insert(binding->id).second ||
          !names.insert(binding->name).second)
        throw std::runtime_error("Invalid or duplicate actor mapping");
      std::vector<std::string> inputs{binding->topic};
      if (element->HasElement("legacy_topic"))
        inputs.push_back(element->Get<std::string>("legacy_topic"));
      for (const auto &topic : inputs)
      {
        if (!topics.insert(topic).second)
          throw std::runtime_error("Duplicate input topic: " + topic);
        // weak_ptr avoids a binding -> subscription -> callback -> binding cycle.
        std::weak_ptr<Binding> weak = binding;
        binding->subscriptions.push_back(this->node->create_subscription<Message>(
          topic, rclcpp::QoS(100).reliable(), [this, weak](Message::ConstSharedPtr msg)
          {
            if (auto state = weak.lock())
              this->Receive(*state, *msg);
          }));
        RCLCPP_INFO(this->node->get_logger(), "%s -> %s (%s)",
          topic.c_str(), binding->name.c_str(), binding->id.c_str());
      }
      binding->groundTruth = this->node->create_publisher<Message>(
        "/humans/" + binding->id + "/ground_truth", rclcpp::QoS(100).reliable());
      this->bindings.push_back(binding);
    }
    this->observations = std::make_shared<Observations>();
    this->observations->frame = this->frame;
    for (const auto &binding : this->bindings)
      this->observations->bindings.push_back({binding->name, binding->groundTruth, -1});
    // SceneUpdate runs in the renderer after actor transforms are applied. It
    // does not force extra rendering like a PreRender/PostRender subscriber.
    // This observation state is independent of commands, queues and target poses.
    this->renderConnection = events.Connect<gz::events::SceneUpdate>(
      [state = this->observations]() { ObserveRenderedActors(state); });
  }

  void PreUpdate(const gz::UpdateInfo &info, gz::EntityComponentManager &ecm) override
  {
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    if (now < this->lastSimTime)
    {
      for (auto &binding : this->bindings)
      {
        binding->pending.clear();
        binding->lastStamp = binding->lastApplied = -1;
        binding->active = binding->moving = false;
        binding->target = binding->initial;
        binding->animationTime = std::chrono::steady_clock::duration::zero();
      }
      RCLCPP_WARN(this->node->get_logger(), "Simulation time reset; targets cleared. Restart trajectory publishers.");
    }
    this->lastSimTime = now;
    for (auto &binding : this->bindings)
    {
      if (binding->entity != gz::kNullEntity)
        continue;
      binding->entity = ecm.EntityByComponents(
        gz::components::Name(binding->name), gz::components::ParentEntity(this->world));
      if (binding->entity == gz::kNullEntity)
        continue;
      gz::Actor actor(binding->entity);
      if (!actor.Valid(ecm))
        throw std::runtime_error("Mapped entity is not an actor: " + binding->name);
      binding->origin = actor.Pose(ecm).value_or(Pose::Zero);
      binding->initial = binding->origin;
      auto actorSdf = ecm.Component<gz::components::Actor>(binding->entity);
      const auto &description = actorSdf->Data();
      if (description.TrajectoryCount() > 0)
      {
        auto trajectory = description.TrajectoryByIndex(0);
        if (trajectory->WaypointCount() > 0)
          binding->initial = binding->origin * trajectory->WaypointByIndex(0)->Pose();
      }
      bool animationFound = false;
      for (uint64_t index = 0; index < description.AnimationCount(); ++index)
        animationFound |= description.AnimationByIndex(index)->Name() == binding->animation;
      if (!animationFound)
        throw std::runtime_error("Missing actor animation: " + binding->animation);
      binding->target = binding->initial;
      RCLCPP_INFO(this->node->get_logger(), "Bound %s to entity %llu; preserved world z=%.3f",
        binding->id.c_str(), static_cast<unsigned long long>(binding->entity), binding->initial.Z());
    }
    this->executor->spin_some(std::chrono::milliseconds(1));
    for (auto &binding : this->bindings)
    {
      if (binding->entity == gz::kNullEntity)
        continue;
      if (!info.paused)
      {
        while (!binding->pending.empty() && binding->pending.front().stamp <= now)
        {
          const auto command = binding->pending.front();
          binding->pending.pop_front();
          const Pose target(command.x, command.y, binding->initial.Z(),
            binding->initial.Rot().Roll(), binding->initial.Rot().Pitch(),
            command.yaw + binding->yawOffset);
          binding->moving = binding->active &&
            (target.Pos() - binding->target.Pos()).Length() > 1e-6;
          if (!binding->active)
            RCLCPP_INFO(this->node->get_logger(),
              "External control active: %s -> %s; world target x=%.3f y=%.3f yaw=%.3f",
              binding->id.c_str(), binding->name.c_str(), command.x, command.y, command.yaw);
          binding->target = target;
          binding->active = true;
          binding->lastApplied = command.stamp;
        }
        if (binding->moving && now - binding->lastApplied <= this->timeout * 1e9 &&
            info.dt > std::chrono::steady_clock::duration::zero())
          binding->animationTime += info.dt;
      }
      gz::Actor actor(binding->entity);
      // Fortress's renderer composes worldPose = origin * trajectoryPose.
      // Keep origin immutable and convert world targets into its local frame.
      actor.SetTrajectoryPose(ecm, binding->origin.Inverse() * binding->target);
      actor.SetAnimationName(ecm, binding->animation);
      actor.SetAnimationTime(ecm, binding->animationTime);
      // Fortress's Actor setters write component Data() directly. They do not
      // mark subsequent writes as changed, so SceneBroadcaster otherwise sends
      // only the initial pose to the GUI and its actor appears frozen there.
      ecm.SetChanged(binding->entity, gz::components::TrajectoryPose::typeId,
        gz::ComponentState::PeriodicChange);
      ecm.SetChanged(binding->entity, gz::components::AnimationTime::typeId,
        gz::ComponentState::PeriodicChange);
    }
  }


private:
  static void ObserveRenderedActors(const std::shared_ptr<Observations> &state)
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    if (!state->enabled)
      return;
    for (const auto &name : ignition::rendering::loadedEngines())
    {
      auto engine = ignition::rendering::engine(name);
      if (!engine)
        continue;
      for (unsigned int index = 0; index < engine->SceneCount(); ++index)
      {
        auto scene = engine->SceneByIndex(index);
        const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(scene->Time()).count();
        if (now <= 0)
          continue;
        for (auto &binding : state->bindings)
        {
          if (binding.publisher->get_subscription_count() == 0)
            continue;
          if (now < binding.lastStamp)
            binding.lastStamp = -1;
          if (binding.lastStamp >= 0 && now - binding.lastStamp < 50000000)
            continue;
          auto actorVisual = scene->VisualByName(binding.name);
          if (!actorVisual || !actorVisual->HasUserData("gazebo-entity"))
            continue;
          // Actual renderer node observation: never read the target command,
          // trajectory origin, or compose a commanded TrajectoryPose as GT.
          const auto pose = actorVisual->WorldPose();
          if (!pose.IsFinite())
            continue;
          Message message;
          message.header.frame_id = state->frame;
          message.header.stamp.sec = static_cast<int32_t>(now / 1000000000LL);
          message.header.stamp.nanosec = static_cast<uint32_t>(now % 1000000000LL);
          message.pose.position.x = pose.X();
          message.pose.position.y = pose.Y();
          message.pose.position.z = pose.Z();
          message.pose.orientation.x = pose.Rot().X();
          message.pose.orientation.y = pose.Rot().Y();
          message.pose.orientation.z = pose.Rot().Z();
          message.pose.orientation.w = pose.Rot().W();
          binding.publisher->publish(message);
          binding.lastStamp = now;
        }
      }
    }
  }

  void Receive(Binding &binding, const Message &message)
  {
    const auto &p = message.pose.position;
    const auto &q = message.pose.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (message.header.frame_id != this->frame ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(norm) || norm < 1e-9 ||
        message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000u)
    {
      RCLCPP_WARN(this->node->get_logger(), "Rejected %s target: require frame %s, finite pose, valid quaternion and timestamp",
        binding.id.c_str(), this->frame.c_str());
      return;
    }
    int64_t stamp = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
      message.header.stamp.nanosec;
    if (stamp == 0)
      stamp = this->lastSimTime;  // Manual ros2 topic pub: apply on the next unpaused tick.
    if (stamp > this->lastSimTime + 60000000000LL)
    {
      RCLCPP_WARN(this->node->get_logger(), "Rejected %s target >60s in the future; use_sim_time must be true in the publisher",
        binding.id.c_str());
      return;
    }
    if (stamp < binding.lastStamp || binding.pending.size() >= 1000)
    {
      RCLCPP_WARN(this->node->get_logger(), "Rejected out-of-order target or full queue for %s", binding.id.c_str());
      return;
    }
    const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
    const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    binding.pending.push_back({stamp, p.x, p.y, yaw});
    binding.lastStamp = stamp;
  }

  gz::Entity world{gz::kNullEntity};
  int64_t lastSimTime{0};
  std::string frame;
  double timeout{0.3};
  rclcpp::Context::SharedPtr context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::vector<std::shared_ptr<Binding>> bindings;
  std::shared_ptr<Observations> observations;
  ignition::common::ConnectionPtr renderConnection;
};
}  // namespace jackal_actor_controller

IGNITION_ADD_PLUGIN(jackal_actor_controller::ActorController,
  ignition::gazebo::System,
  jackal_actor_controller::ActorController::ISystemConfigure,
  jackal_actor_controller::ActorController::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(jackal_actor_controller::ActorController,
  "jackal_actor_controller::ActorController")
