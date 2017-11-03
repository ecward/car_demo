#pragma once

#include <ignition/math/Vector3.hh>
#include <ignition/msgs/cmd_vel2d.pb.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>

///@todo Include relevant message types to send control commands
///replace this include...
//#include <prius_msgs/Control.h>

namespace gazebo
{
// Forward declaration
class RCVPluginPrivate;

/// \brief A model plugin for the RCV
/// Note that this class inherits from gazebo::ModelPlugin
/// This parent class defines a set of methods and members, which
/// are used to interface with the simulator.
///
/// Note that the functions:
/// Load,Init and Reset are virtual in gazebo::ModelPlugin, so that we can change
/// the implementation of them.
///
/// It might be worthwhile to look at:
/// http://gazebosim.org/tutorials/?tut=plugins_hello_world
class RCVPlugin : public ModelPlugin
{
public:
  /// \brief Constructor.
  RCVPlugin();

  /// \brief Destructor.
  virtual ~RCVPlugin();

  // Documentation Inherited
  virtual void Reset();

  /// \brief Load the controller.
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief ROS subscriber callback
private:
  ///@todo implement a method that handles current control inputs.
  /// look at this method in PriusHybridPlugin.hh
  //void OnPriusCommand(const prius_msgs::Control::ConstPtr &msg);

  /// \brief Callback each time a key message is received.
  /// \param[in] _msg Keypress message.
  void OnKeyPress(ConstAnyPtr &_msg);

  /// \brief Callback each time a key message is received.
  /// \param[in] _msg Keypress message.
  void OnKeyPressIgn(const ignition::msgs::Any &_msg);

  /// \brief Key control
  /// \param[in] _key key value
  void KeyControl(const int _key);

  /// \brief Key control type A
  /// \param[in] _key key value
  void KeyControlTypeA(const int _key);

  /// \brief Key control type B
  /// \param[in] _key key value
  void KeyControlTypeB(const int _key);


  /// \brief Command to reset the world
  /// \param[in] _msg Int32 message data. Not used
  void OnReset(const ignition::msgs::Any &_msg);

  /// \brief Command to stop the simulation
  /// \param[in] _msg Int32 message data. Not used
  void OnStop(const ignition::msgs::Any &_msg);

  /// \brief Update on every time step
  /// @note This function is used as a callback inside gazebo.
  /// In the Load function we will tell gazebo that this is the function
  /// to use for each "update cycle"
  void Update();

  /// \brief Update steering wheel to front left/right wheel ratio
  void UpdateHandWheelRatio();

  /// \brief Get the radius of a collision
  double CollisionRadius(physics::CollisionPtr _collision);

  /// \brief Get the multiplier that is determined based on the direction
  /// state of the vehicle.
  /// \return 1.0 if FORWARD, -1.0 if REVERSE, 0.0 otherwise
  double GasTorqueMultiplier();

  /// \brief Private data
  std::unique_ptr<RCVPluginPrivate> dataPtr;
};
}
