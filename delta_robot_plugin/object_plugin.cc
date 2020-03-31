#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ObjectPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      this->model = _model;

      if (_sdf->HasElement("speed")){
        this->speed = _sdf->Get<float>("speed");
      }

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ObjectPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      this->model->SetLinearVel(ignition::math::Vector3d(0, this->speed, 0));
    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: std::float_t speed;
  };

  GZ_REGISTER_MODEL_PLUGIN(ObjectPlugin)
}