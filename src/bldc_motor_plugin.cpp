#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>                 // <-- this one
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/msgs/double_v.pb.h>

using namespace gz;
using namespace sim;

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
    this->model = Model(entity);
    if (!this->model.Valid(ecm))
    {
      gzerr << "[BLDCMotorPlugin] Invalid model.\n";
      return;
    }

    // Parameters (with SDF overrides)
    this->jointName = sdf->Get<std::string>("joint_name", "rotor_joint").first;
    this->R  = sdf->Get<double>("resistance", 0.5).first;   // Ohm
    this->Ke = sdf->Get<double>("ke",         0.02).first;  // V·s/rad
    this->Kt = sdf->Get<double>("kt",         0.02).first;  // N·m/A
    this->topic = sdf->Get<std::string>("voltage_topic", "/bldc_motor/voltage").first;

    this->jointEntity = this->model.JointByName(ecm, this->jointName);
    if (!this->jointEntity)
    {
      gzerr << "[BLDCMotorPlugin] Joint not found: " << this->jointName << "\n";
      return;
    }

    // GZ Transport subscriber (gz.msgs.Double). ROS2 can bridge to this.
    if (!this->node.Subscribe(this->topic, &BLDCMotorPlugin::OnVoltageMsg, this))
    {
      gzerr << "[BLDCMotorPlugin] Failed to subscribe to " << this->topic << "\n";
    }
    else
    {
      gzmsg << "[BLDCMotorPlugin] Subscribed to " << this->topic << "\n";
    }

    gzmsg << "[BLDCMotorPlugin] Configured. joint=" << this->jointName
          << " R=" << this->R << " Ke=" << this->Ke << " Kt=" << this->Kt << "\n";


    this->statePub = this->node.Advertise<msgs::Double_V>("/bldc_motor/state");
    if (!this->statePub)
    {
        gzerr << "[BLDCMotorPlugin] Failed to advertise state topic\n";
    }
    else
    {
        gzmsg << "[BLDCMotorPlugin] Publishing state on /bldc_motor/state\n";
    }

  }

    void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override
    {
    if (info.paused || !this->jointEntity)
        return;

    // Current angular velocity (rad/s)
    double omega = 0.0;
    auto vel = ecm.Component<components::JointVelocity>(this->jointEntity);
    if (vel && !vel->Data().empty())
        omega = vel->Data()[0];

    // Electrical + mechanical model
    const double I = (this->voltageCmd - this->Ke * omega) / this->R;
    const double torque = this->Kt * I;

    // Apply torque
    Joint joint(this->jointEntity);
    joint.SetForce(ecm, std::vector<double>{torque});

    // --- Publish state ---
    if (this->statePub)
    {
        msgs::Double_V stateMsg;
        stateMsg.add_data(omega);  // rad/s
        stateMsg.add_data(I);      // A
        stateMsg.add_data(torque); // N·m
        this->statePub.Publish(stateMsg);
    }
    }


private:
  void OnVoltageMsg(const msgs::Double &msg)
  {
    this->voltageCmd = msg.data();
  }

private:
  Model   model{kNullEntity};
  Entity  jointEntity{kNullEntity};
  std::string jointName{"rotor_joint"};
  std::string topic{"/bldc_motor/voltage"};
  transport::Node node;
  transport::Node::Publisher statePub;

  // Motor params
  double R{0.5};
  double Ke{0.02};
  double Kt{0.02};

  // Command
  double voltageCmd{0.0};
};

GZ_ADD_PLUGIN(BLDCMotorPlugin,
              System,
              BLDCMotorPlugin::ISystemConfigure,
              BLDCMotorPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(BLDCMotorPlugin, "BLDCMotorPlugin")
