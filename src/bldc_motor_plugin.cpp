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
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <mutex>
#include <memory>
#include <atomic>
#include <chrono>
#include <thread>
#include <cmath>
#include <algorithm>

using namespace gz;
using namespace sim;

class BLDCMotorPlugin : public System,
                        public ISystemConfigure,
                        public ISystemPreUpdate
{
public:
  // --- Configure ---
  void Configure(const Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &ecm,
                 EventManager &/*eventMgr*/) override
  {
    this->model = Model(entity);
    if (!this->model.Valid(ecm))
    {
      gzerr << "[BLDCMotorPlugin] Invalid model.\n";
      return;
    }

    // Motor parametreleri
    this->R   = sdf->Get<double>("resistance", this->R).first;
    this->Ke  = sdf->Get<double>("ke", this->Ke).first;
    this->Kt  = sdf->Get<double>("kt", this->Kt).first;

    this->Vmax = sdf->Get<double>("voltage_limit", this->Vmax).first;
    this->Imax = sdf->Get<double>("current_limit", this->Imax).first;
    this->Tmax = sdf->Get<double>("torque_limit", this->Tmax).first;

    this->B  = sdf->Get<double>("viscous_friction", this->B).first;
    this->Tc = sdf->Get<double>("coulomb_friction", this->Tc).first;

    this->rosTopic = sdf->Get<std::string>("ros_voltage_topic", this->rosTopic).first;
    this->useRos   = sdf->Get<bool>("use_ros_voltage_sub", false).first;

    // Joint adı
    this->jointName = sdf->Get<std::string>("joint_name", this->jointName).first;
    this->jointEntity = this->model.JointByName(ecm, this->jointName);
    if (!this->jointEntity)
    {
      gzerr << "[BLDCMotorPlugin] Joint not found: " << this->jointName << "\n";
      return;
    }

    // GZ topic (alternatif kontrol için)
    if (!this->useRos)
    {
      if (!this->gzNode.Subscribe(this->cmdTopic, &BLDCMotorPlugin::OnVoltageMsgGZ, this))
        gzerr << "[BLDCMotorPlugin] Failed to subscribe (GZ): " << this->cmdTopic << "\n";
      else
        gzmsg << "[BLDCMotorPlugin] Subscribed (GZ) to " << this->cmdTopic << "\n";
    }

    // ROS2 entegrasyonu
    if (this->useRos)
    {
      if (!rclcpp::ok())
        rclcpp::init(0, nullptr);

      this->node = std::make_shared<rclcpp::Node>("bldc_motor_plugin");

      // Voltage subscriber
      this->subV = this->node->create_subscription<std_msgs::msg::Float64>(
          this->rosTopic, 10,
          [this](const std_msgs::msg::Float64 &msg)
          {
            this->voltageCmd.store(msg.data, std::memory_order_relaxed);
          });

      // State publisher
      this->pubState = this->node->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/bldc_motor/state", 10);

      // Spin thread
      this->spinThread = std::thread([&]{ rclcpp::spin(this->node); });
    }
  }

  // --- PreUpdate ---
  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override
  {
    if (info.paused || !this->jointEntity)
      return;

    // Mevcut hız (rad/s)
    double omega = 0.0;
    if (auto vel = ecm.Component<components::JointVelocity>(this->jointEntity))
      if (!vel->Data().empty()) omega = vel->Data()[0];

    // Motor denklemleri
    double emf = this->Ke * omega;    // back-EMF
    double vcmd = std::clamp(this->voltageCmd.load(), -this->Vmax, this->Vmax);
    double i = std::clamp((vcmd - emf) / this->R, -this->Imax, this->Imax);
    double tau_e = this->Kt * i;      // elektromanyetik tork

    // Sürtünmeler
    double tau_f = this->B * omega + (omega == 0.0 ? 0.0 : this->Tc * std::copysign(1.0, omega));

    // Tork komutu
    double tau_cmd = std::clamp(tau_e - tau_f, -this->Tmax, this->Tmax);

    // Joint'e uygula
    this->SetJointTorque(ecm, tau_cmd);

    // State publish (ROS varsa)
    auto now = std::chrono::steady_clock::now();
    if (this->useRos && this->pubState)
    {
      auto dt = std::chrono::duration<double>(now - this->lastPub).count();
      if (dt >= 1.0 / this->publish_rate_hz)
      {
        std_msgs::msg::Float64MultiArray m;
        m.data = {omega, i, vcmd, tau_cmd};
        this->pubState->publish(m);
        this->lastPub = now;
      }
    }

    // son değerleri sakla
    this->lastOmega = omega;
    this->lastCurrent = i;
    this->lastTorque = tau_cmd;
  }

private:
  // GZ transport callback
  void OnVoltageMsgGZ(const msgs::Double &msg)
  {
    this->voltageCmd.store(msg.data(), std::memory_order_relaxed);
  }

  // Torque uygulama
  void SetJointTorque(EntityComponentManager &ecm, double tau)
  {
    Joint j(this->jointEntity);
    j.SetForce(ecm, std::vector<double>{tau});
  }


private:
  // Gazebo
  Model   model{kNullEntity};
  Entity  jointEntity{kNullEntity};
  std::string jointName{"rotor_joint"};

  // Motor parametreleri
  double R{0.5};
  double Ke{0.02};
  double Kt{0.02};
  double Vmax{48.0};
  double Imax{40.0};
  double Tmax{4.2};
  double B{0.0};
  double Tc{0.0};

  // ROS2
  bool useRos{false};
  std::string rosTopic{"/bldc_motor/voltage_cmd"};
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subV;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubState;
  std::thread spinThread;

  // GZ transport
  transport::Node gzNode;
  std::string cmdTopic{"/bldc_motor/voltage"};

  // Motor durumu
  std::atomic<double> voltageCmd{0.0};
  double lastOmega{0.0};
  double lastCurrent{0.0};
  double lastTorque{0.0};

  // Publish rate
  double publish_rate_hz{50.0};
  std::chrono::steady_clock::time_point lastPub{std::chrono::steady_clock::now()};
};

// Plugin kaydı
GZ_ADD_PLUGIN(BLDCMotorPlugin,
              System,
              BLDCMotorPlugin::ISystemConfigure,
              BLDCMotorPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(BLDCMotorPlugin, "BLDCMotorPlugin")
