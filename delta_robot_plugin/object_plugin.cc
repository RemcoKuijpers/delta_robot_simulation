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
      if (_sdf->HasElement("distance")){
        this->distance = _sdf->Get<float>("distance");
      }

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ObjectPlugin::OnUpdate, this, std::placeholders::_1));

    }

    public: void OnUpdate(const common::UpdateInfo &_info)
    {
      //this->model->SetLinearVel(ignition::math::Vector3d(0, this->speed*(_info.realTime.Float()/_info.simTime.Float()), 0));
      
      this->position = this->model->WorldPose().Pos().Y();
      if (this->position > this->distance){
        this->model->Fini();
      }
    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: std::float_t speed;
    private: std::float_t distance;
    private: std::double_t position;
  };

  GZ_REGISTER_MODEL_PLUGIN(ObjectPlugin)
}