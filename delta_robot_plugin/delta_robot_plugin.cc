#ifndef _DELTA_ROBOT_PLUGIN_HH_
#define _DELTA_ROBOT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Vector3.h"

namespace gazebo
{
  class DeltaRobotPlugin : public ModelPlugin
  {
    public: DeltaRobotPlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
    if (_model->GetJointCount() == 0)
    {
      std::cerr << "Invalid joint count, delta_robot plugin not loaded\n";
      return;
    }

    this->model = _model;
    this->joint1 = _model->GetJoint("motor1");
    this->joint2 = _model->GetJoint("motor2");
    this->joint3 = _model->GetJoint("motor3");

    int p, i, d;
    if (_sdf->HasElement("p"))
      p = _sdf->Get<int>("p");
    if (_sdf->HasElement("i"))
      i = _sdf->Get<int>("i");
    if (_sdf->HasElement("d"))
      d = _sdf->Get<int>("d");

    this->pid = common::PID(p, i, d);
    this->model->GetJointController()->SetPositionPID(
      this->joint1->GetScopedName(), this->pid);
    this->model->GetJointController()->SetPositionPID(
      this->joint2->GetScopedName(), this->pid);
    this->model->GetJointController()->SetPositionPID(
      this->joint3->GetScopedName(), this->pid);
    this->SetPositions(0.5, 0.5, 0.5);

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());
    std::string topicName = "~/" + this->model->GetName() + "/pos_cmd";
    this->sub = this->node->Subscribe(topicName,
      &DeltaRobotPlugin::OnMsg, this);


    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client",
          ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Vector3>(
          "/" + this->model->GetName() + "/pos_cmd",
          1,
          boost::bind(&DeltaRobotPlugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);

    this->rosSub = this->rosNode->subscribe(so);

    this->rosQueueThread =
      std::thread(std::bind(&DeltaRobotPlugin::QueueThread, this));
    }

    public: void SetPositions(const double &_a1, const double &_a2, const double &_a3)
    {
      this->model->GetJointController()->SetPositionTarget(
        this->joint1->GetScopedName(), _a1);
      this->model->GetJointController()->SetPositionTarget(
        this->joint2->GetScopedName(), _a2);
      this->model->GetJointController()->SetPositionTarget(
        this->joint3->GetScopedName(), _a3);
    }

    private: void OnMsg(ConstVector3dPtr &_msg)
    {
        this->SetPositions(_msg->x(), _msg->y(), _msg->z());
    }

    public: void OnRosMsg(const geometry_msgs::Vector3ConstPtr _msg)
    {

      this->SetPositions(_msg->x, _msg->y, _msg->z);
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    private: physics::ModelPtr model;

    private: physics::JointPtr joint1;
    private: physics::JointPtr joint2;
    private: physics::JointPtr joint3;

    private: common::PID pid;

    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;
    };

  GZ_REGISTER_MODEL_PLUGIN(DeltaRobotPlugin)
}
#endif
