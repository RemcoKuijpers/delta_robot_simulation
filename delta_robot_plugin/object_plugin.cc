#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ObjectPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _model;

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ObjectPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      this->model->SetLinearVel(ignition::math::Vector3d(0, 0.3, 0));
    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(ObjectPlugin)
}