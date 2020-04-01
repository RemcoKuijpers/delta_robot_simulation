#ifndef _DELTA_ROBOT_PLUGIN_HH_
#define _DELTA_ROBOT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"

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
    this->ee_joint = _model->GetJoint("ee_joint");
    this->ee = _model->GetLink("low_base");

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
    this->model->GetJointController()->SetPositionPID(
      this->ee_joint->GetScopedName(), this->pid);
    this->SetPositions(0, 0, 0);

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

    ros::SubscribeOptions so_ee =
      ros::SubscribeOptions::create<std_msgs::Float32>(
          "/" + this->model->GetName() + "/ee_cmd",
          1,
          boost::bind(&DeltaRobotPlugin::OnRosMsgEE, this, _1),
          ros::VoidPtr(), &this->rosQueue);

    this->rosSub = this->rosNode->subscribe(so);
    this->rosSub_ee = this->rosNode->subscribe(so_ee);
    this->rosMotorStatePub = this->rosNode->advertise<geometry_msgs::Vector3Stamped>("/" + this->model->GetName() + "/motor_angles", 10);
    this->rosPosePub = this->rosNode->advertise<geometry_msgs::PoseStamped>("/" + this->model->GetName() + "/ee_pose", 10);

    this->rosQueueThread =
      std::thread(std::bind(&DeltaRobotPlugin::QueueThread, this));
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&DeltaRobotPlugin::OnUpdate, this));
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

    public: void OnRosMsg(const geometry_msgs::Vector3ConstPtr _msg)
    {
      this->SetPositions(_msg->x, _msg->y, _msg->z);
    }

    public: void OnRosMsgEE(const std_msgs::Float32ConstPtr _msg)
    {
      this->ee_joint->SetPosition(0, _msg->data);
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    
    private: void OnUpdate()
    {
      geometry_msgs::Vector3Stamped msg;
      geometry_msgs::PoseStamped msg2;

      msg.header.stamp = ros::Time::now();
      msg.vector.x = this->joint1->Position();
      msg.vector.y = this->joint2->Position();
      msg.vector.z = this->joint3->Position();

      msg2.header.stamp = ros::Time::now();
      msg2.pose.position.x = this->ee->WorldCoGPose().Pos().X();
      msg2.pose.position.y = this->ee->WorldCoGPose().Pos().Y();
      msg2.pose.position.z = this->ee->WorldCoGPose().Pos().Z();
      msg2.pose.orientation.w = this->ee->WorldCoGPose().Rot().W();
      msg2.pose.orientation.x = this->ee->WorldCoGPose().Rot().X();
      msg2.pose.orientation.y = this->ee->WorldCoGPose().Rot().Y();
      msg2.pose.orientation.z = this->ee->WorldCoGPose().Rot().Z();

      this->rosMotorStatePub.publish(msg);
      this->rosPosePub.publish(msg2);
    }

    private: physics::ModelPtr model;

    private: physics::JointPtr joint1;
    private: physics::JointPtr joint2;
    private: physics::JointPtr joint3;
    private: physics::JointPtr ee_joint;
    private: physics::LinkPtr ee;

    private: common::PID pid;

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;

    private: ros::Subscriber rosSub_ee;

    private: ros::Publisher rosMotorStatePub;
    private: ros::Publisher rosPosePub;
    private: event::ConnectionPtr updateConnection;
    };

  GZ_REGISTER_MODEL_PLUGIN(DeltaRobotPlugin)
}
#endif
