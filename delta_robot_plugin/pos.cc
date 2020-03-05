#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>


int main(int _argc, char *_argv[])
{
  gazebo::client::setup(_argc, _argv);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Vector3d>("~/delta_robot/pos_cmd");

  pub->WaitForConnection();

  gazebo::msgs::Vector3d msg;

  gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(_argv[1]),std::atof(_argv[2]),std::atof(_argv[3])));
  pub->Publish(msg);

  gazebo::client::shutdown();
}