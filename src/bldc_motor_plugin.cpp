#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/JointVelocity.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mutex>
#include <memory>
#include <atomic>
#include <chrono>

using namespace gz;
using namespace sim;

class BLDCMotorPlugin : public System,
                        public ISystemConfigure,
                        public ISystemPreUpdate
{
public:
  // Called once when the plugin is attached to a model
  void Configure(const Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm,
                 EventManager &/*eventMgr*/) override
  {
    // Wrap the model entity
    this->model = Model(entity);
    if (!this->model.Valid(ecm))
    {
      gzerr << "[BLDCMotorPlugin] Invalid model.\n";
      return;
    }

    // Read motor and plugin parameters from SDF (with defaults)
    this->jointName  = sdf->Get<std::string>("joint_name", "rotor_joint").first;
    this->R          = sdf->Get<double>("resistance", 0.5).first; // Ohms
    this->Ke         = sdf->Get<double>("ke", 0.02).first;        // V·s/rad
    this->Kt         = sdf->Get<double>("kt", 0.02).first;        // N·m/A
    this->cmdTopic   = sdf->Get<std::string>("voltage_topic", "/bldc_motor/voltage").first;
    this->useRosCmd  = sdf->Get<bool>("use_ros_voltage_sub", false).first; // True = ROS2 command, False = GZ command
    this->publish_rate_hz = sdf->Get<double>("publish_rate_hz", 50.0).first;  // e.g., 1.0 for 1 Hz
    this->control_rate_hz = sdf->Get<double>("control_rate_hz", 0.0).first;  // 0.0 => run every step

    // Find the rotor joint in the model
    this->jointEntity = this->model.JointByName(ecm, this->jointName);
    if (!this->jointEntity)
    {
      gzerr << "[BLDCMotorPlugin] Joint not found: " << this->jointName << "\n";
      return;
    }

    // If using GZ transport for command input, subscribe to /bldc_motor/voltage (gz.msgs.Double)
    if (!this->useRosCmd)
    {
      if (!this->gzNode.Subscribe(this->cmdTopic, &BLDCMotorPlugin::OnVoltageMsgGZ, this))
        gzerr << "[BLDCMotorPlugin] Failed to subscribe (GZ): " << this->cmdTopic << "\n";
      else
        gzmsg << "[BLDCMotorPlugin] Subscribed (GZ) to " << this->cmdTopic << "\n";
    }

    // Initialize ROS 2 (only once, shared among all plugins)
    std::call_once(rosInitOnce, [](){
      int argc = 0; char **argv = nullptr;
      rclcpp::init(argc, argv);
    });

    // Create ROS 2 node
    this->rosNode = std::make_shared<rclcpp::Node>("bldc_motor_plugin");

    // Create ROS 2 publisher for motor state: geometry_msgs/Vector3
    //   x = omega (rad/s), y = current (A), z = torque (N·m)
    this->rosStatePub = this->rosNode->create_publisher<geometry_msgs::msg::Vector3>(
        "/bldc_motor/state", 10);

    // If using ROS 2 for command input, subscribe to /bldc_motor/voltage (std_msgs/Float64)
    if (this->useRosCmd)
    {
      this->rosCmdSub = this->rosNode->create_subscription<std_msgs::msg::Float64>(
        "/bldc_motor/voltage", 10,
        [this](const std_msgs::msg::Float64 &msg){
          this->voltageCmd.store(msg.data, std::memory_order_relaxed);
        });
      gzmsg << "[BLDCMotorPlugin] Subscribed (ROS2) to /bldc_motor/voltage\n";
    }

    // Create a simple executor to handle ROS 2 callbacks
    this->rosExec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->rosExec->add_node(this->rosNode);

    gzmsg << "[BLDCMotorPlugin] Configured. joint=" << this->jointName
          << " R=" << this->R << " Ke=" << this->Ke << " Kt=" << this->Kt
          << " useRosCmd=" << (this->useRosCmd ? "true" : "false") << "\n";

    gzmsg << "[BLDCMotorPlugin] Rates: publish=" << this->publish_rate_hz
          << " Hz, control=" << this->control_rate_hz << " Hz\n";
  }

  // Called every simulation iteration
  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override
  {
    if (info.paused || !this->jointEntity)
      return;

    if (this->rosExec) this->rosExec->spin_some();

    // Read current omega (rad/s)
    double omega = 0.0;
    if (auto vel = ecm.Component<components::JointVelocity>(this->jointEntity))
      if (!vel->Data().empty()) omega = vel->Data()[0];

    // === NEW: control throttling ===
    bool doControl = true;
    if (this->control_rate_hz > 0.0)
    {
      const auto ctrl_period = std::chrono::duration<double>(1.0 / this->control_rate_hz);
      if (info.simTime - this->last_ctrl_sim >= ctrl_period)
      {
        this->last_ctrl_sim = info.simTime;
        doControl = true;
      }
      else
      {
        doControl = false;
      }
    }

    // Compute/hold control
    double I = this->lastCurrent;
    double torque = this->lastTorque;

    if (doControl)
    {
      const double vcmd = this->voltageCmd.load(std::memory_order_relaxed);
      I = (vcmd - this->Ke * omega) / this->R;
      torque = this->Kt * I;

      // Cache for publishing at a slower rate
      this->lastOmega   = omega;
      this->lastCurrent = I;
      this->lastTorque  = torque;
    }

    // Apply torque (always)
    Joint joint(this->jointEntity);
    joint.SetForce(ecm, std::vector<double>{torque});

    // === NEW: publish throttling ===
    const auto pub_period = std::chrono::duration<double>(
        1.0 / std::max(1e-9, this->publish_rate_hz)); // prevent div-by-zero

    if (info.simTime - this->last_pub_sim >= pub_period)
    {
      this->last_pub_sim = info.simTime;

      if (this->rosStatePub)
      {
        geometry_msgs::msg::Vector3 v;
        v.x = this->lastOmega;    // rad/s
        v.y = this->lastCurrent;  // A
        v.z = this->lastTorque;   // N·m
        this->rosStatePub->publish(v);
      }
    }
  }


private:
  // Callback for receiving GZ transport voltage commands
  void OnVoltageMsgGZ(const msgs::Double &msg)
  {
    this->voltageCmd.store(msg.data(), std::memory_order_relaxed);
  }

private:
  // Gazebo model and joint
  Model   model{kNullEntity};
  Entity  jointEntity{kNullEntity};

  // Configurable parameters
  std::string jointName{"rotor_joint"};
  std::string cmdTopic{"/bldc_motor/voltage"};
  bool useRosCmd{false};

  // GZ transport node
  transport::Node gzNode;

  // ROS 2 integration
  static std::once_flag rosInitOnce;
  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rosStatePub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rosCmdSub;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr rosExec;

  // Motor electrical parameters
  double R{0.5};   // Ohms
  double Ke{0.02}; // V·s/rad
  double Kt{0.02}; // N·m/A
  double publish_rate_hz{50.0};   // SDF <publish_rate_hz>, default 50 Hz
  double control_rate_hz{0.0};    // SDF <control_rate_hz>, default 0 (= every step)

  std::chrono::steady_clock::duration last_pub_sim{std::chrono::steady_clock::duration::zero()};
  std::chrono::steady_clock::duration last_ctrl_sim{std::chrono::steady_clock::duration::zero()};

  // Latest commanded voltage
  std::atomic<double> voltageCmd{0.0};
  double lastOmega{0.0};
  double lastCurrent{0.0};
  double lastTorque{0.0};
};

std::once_flag BLDCMotorPlugin::rosInitOnce;

GZ_ADD_PLUGIN(BLDCMotorPlugin,
              System,
              BLDCMotorPlugin::ISystemConfigure,
              BLDCMotorPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(BLDCMotorPlugin, "BLDCMotorPlugin")
