#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointPosition.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <algorithm>
#include <cmath>  // round

using namespace gz;
using namespace sim;

// 3 ondalığa yuvarla
inline double round_to_3(double x) {
  return std::round(x * 1000.0) / 1000.0;
}

class BLDCMotorPlugin : public System,
                        public ISystemConfigure,
                        public ISystemPreUpdate
{
public:
  void Configure(const Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm,
                 EventManager &/*eventMgr*/) override
  {
    // Model
    this->model = Model(entity);
    if (!this->model.Valid(ecm))
    {
      gzerr << "[BLDCMotorPlugin] Invalid model.\n";
      return;
    }

    // ---- SDF params (defaults) ----
    this->jointName        = sdf->Get<std::string>("joint_name", "rotor_joint").first;
    this->R                = sdf->Get<double>("resistance", 0.5).first; // Ohm
    this->Ke               = sdf->Get<double>("ke", 0.02).first;        // V·s/rad
    this->Kt               = sdf->Get<double>("kt", 0.02).first;        // N·m/A
    this->cmdTopic         = sdf->Get<std::string>("voltage_topic", "/bldc_motor/voltage").first;
    this->stateTopic       = sdf->Get<std::string>("state_topic",   "state").first;
    this->useRosCmd        = sdf->Get<bool>("use_ros_voltage_sub", false).first;
    this->publish_rate_hz  = sdf->Get<double>("publish_rate_hz", 50.0).first;
    this->control_rate_hz  = sdf->Get<double>("control_rate_hz", 0.0).first;

    // ---- ROS namespace (<ros><namespace>...</namespace></ros>) ----
    std::string ros_ns;
    if (sdf && sdf->HasElement("ros"))
    {
      auto sdf_nc = std::const_pointer_cast<sdf::Element>(sdf);
      auto rosElem = sdf_nc->GetElement("ros");
      if (rosElem && rosElem->HasElement("namespace"))
        ros_ns = rosElem->Get<std::string>("namespace");
    }

    // Joint
    this->jointEntity = this->model.JointByName(ecm, this->jointName);
    if (!this->jointEntity)
    {
      gzerr << "[BLDCMotorPlugin] Joint not found: " << this->jointName << "\n";
      return;
    }

    // *** Fizikten veri gelmesi için state komponentlerini talep et ***
    if (!ecm.Component<components::JointVelocity>(this->jointEntity))
      ecm.CreateComponent(this->jointEntity, components::JointVelocity({0.0}));
    if (!ecm.Component<components::JointPosition>(this->jointEntity))
      ecm.CreateComponent(this->jointEntity, components::JointPosition({0.0}));

    // GZ transport command (ROS kapalıysa)
    if (!this->useRosCmd)
    {
      if (!this->gzNode.Subscribe(this->cmdTopic, &BLDCMotorPlugin::OnVoltageMsgGZ, this))
        gzerr << "[BLDCMotorPlugin] Failed to subscribe (GZ): " << this->cmdTopic << "\n";
      else
        gzmsg << "[BLDCMotorPlugin] Subscribed (GZ) to " << this->cmdTopic << "\n";
    }

    // ROS 2 init (bir kez)
    std::call_once(rosInitOnce, [](){
      int argc = 0; char **argv = nullptr;
      rclcpp::init(argc, argv);
    });

    // Benzersiz node adı
    const std::string node_name = "bldc_motor_plugin_" + this->jointName;

    rclcpp::NodeOptions opts;
    this->rosNode = std::make_shared<rclcpp::Node>(node_name, ros_ns, opts);

    // State publisher
    this->rosStatePub = this->rosNode->create_publisher<geometry_msgs::msg::Vector3>(
        this->stateTopic, 10);

    // ROS command subscriber
    if (this->useRosCmd)
    {
      this->rosCmdSub = this->rosNode->create_subscription<std_msgs::msg::Float64>(
        this->cmdTopic, 10,
        [this](const std_msgs::msg::Float64 &msg){
          this->voltageCmd.store(msg.data, std::memory_order_relaxed);
        });

      gzmsg << "[BLDCMotorPlugin] Subscribed (ROS2) to "
            << (ros_ns.empty() ? "" : ros_ns)
            << (ros_ns.empty() ? "" : "/")
            << this->cmdTopic << "\n";
    }

    // Executor
    this->rosExec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->rosExec->add_node(this->rosNode);

    // Fallback türev için ilk değerleri temizle
    this->lastPosValid = false;
    this->lastPos = 0.0;
    this->lastPosSim = std::chrono::steady_clock::duration::zero();

    gzmsg << "[BLDCMotorPlugin] Configured. joint=" << this->jointName
          << " R=" << this->R << " Ke=" << this->Ke << " Kt=" << this->Kt
          << " useRosCmd=" << (this->useRosCmd ? "true" : "false")
          << " cmdTopic=" << this->cmdTopic
          << " stateTopic=" << this->stateTopic
          << " ns=" << (ros_ns.empty() ? "/" : ros_ns) << "\n";

    gzmsg << "[BLDCMotorPlugin] Rates: publish=" << this->publish_rate_hz
          << " Hz, control=" << this->control_rate_hz << " Hz\n";

    gzmsg << "[BLDCMotorPlugin] ROS node: "
          << this->rosNode->get_fully_qualified_name() << "\n";
  }

  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override
  {
    if (info.paused || !this->jointEntity)
      return;

    if (this->rosExec) this->rosExec->spin_some();

    // --- Açısal hız (omega) okuma ---
    double omega = 0.0;

    // 1) Varsa doğrudan JointVelocity
    if (auto vel = ecm.Component<components::JointVelocity>(this->jointEntity))
    {
      if (!vel->Data().empty())
        omega = vel->Data()[0];
    }

    // 2) Yoksa (veya 0 kalmışsa): JointPosition'dan türev al (dθ/dt)
    if (std::abs(omega) < 1e-12)
    {
      if (auto pos = ecm.Component<components::JointPosition>(this->jointEntity))
      {
        if (!pos->Data().empty())
        {
          double theta = pos->Data()[0];
          if (this->lastPosValid)
          {
            const double dt = std::chrono::duration<double>(info.simTime - this->lastPosSim).count();
            if (dt > 1e-9)
              omega = (theta - this->lastPos) / dt;
          }
          this->lastPos = theta;
          this->lastPosSim = info.simTime;
          this->lastPosValid = true;
        }
      }
    }

    // --- Komut → akım → tork ---
    const double vcmd = this->voltageCmd.load(std::memory_order_relaxed);
    double I = (vcmd - this->Ke * omega) / this->R;
    double torque = this->Kt * I;

    // Kontrol throttling (isteğe bağlı)
    bool doControl = true;
    if (this->control_rate_hz > 0.0)
    {
      const auto ctrl_period = std::chrono::duration<double>(1.0 / this->control_rate_hz);
      if (info.simTime - this->last_ctrl_sim >= ctrl_period)
      {
        this->last_ctrl_sim = info.simTime;
      }
      else
      {
        doControl = false;
      }
    }

    if (doControl)
    {
      this->lastOmega   = omega;
      this->lastCurrent = I;
      this->lastTorque  = torque;
    }

    // Tork uygula
    Joint joint(this->jointEntity);
    joint.SetForce(ecm, std::vector<double>{this->lastTorque});

    // Publish throttling
    const auto pub_period =
        std::chrono::duration<double>(1.0 / std::max(1e-9, this->publish_rate_hz));

    if (info.simTime - this->last_pub_sim >= pub_period)
    {
      this->last_pub_sim = info.simTime;

      if (this->rosStatePub)
      {
        geometry_msgs::msg::Vector3 v;
        // <-- 3 basamaklı yuvarlama -->
        v.x = round_to_3(this->lastOmega);    // rad/s
        v.y = round_to_3(this->lastCurrent);  // A
        v.z = round_to_3(this->lastTorque);   // N·m
        this->rosStatePub->publish(v);
      }
    }
  }

private:
  // GZ transport voltage callback
  void OnVoltageMsgGZ(const msgs::Double &msg)
  {
    this->voltageCmd.store(msg.data(), std::memory_order_relaxed);
  }

private:
  // Model & joint
  Model   model{kNullEntity};
  Entity  jointEntity{kNullEntity};

  // Params
  std::string jointName{"rotor_joint"};
  std::string cmdTopic{"/bldc_motor/voltage"};
  std::string stateTopic{"state"};
  bool useRosCmd{false};

  // GZ transport
  transport::Node gzNode;

  // ROS 2
  static std::once_flag rosInitOnce;
  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rosStatePub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rosCmdSub;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr rosExec;

  // Motor model
  double R{0.5};
  double Ke{0.02};
  double Kt{0.02};
  double publish_rate_hz{50.0};
  double control_rate_hz{0.0};

  // Publish/Control zamanlayıcıları
  std::chrono::steady_clock::duration last_pub_sim{std::chrono::steady_clock::duration::zero()};
  std::chrono::steady_clock::duration last_ctrl_sim{std::chrono::steady_clock::duration::zero()};

  // Komut ve son durum cache
  std::atomic<double> voltageCmd{0.0};
  double lastOmega{0.0};
  double lastCurrent{0.0};
  double lastTorque{0.0};

  // Fallback için pozisyon geçmişi
  bool lastPosValid{false};
  double lastPos{0.0};
  std::chrono::steady_clock::duration lastPosSim{std::chrono::steady_clock::duration::zero()};
};

std::once_flag BLDCMotorPlugin::rosInitOnce;

GZ_ADD_PLUGIN(BLDCMotorPlugin,
              System,
              BLDCMotorPlugin::ISystemConfigure,
              BLDCMotorPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(BLDCMotorPlugin, "BLDCMotorPlugin")
