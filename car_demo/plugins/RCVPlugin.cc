#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/AdvertiseOptions.hh>

#include "RCVPlugin.hh"
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>

#include <ros/ros.h>

namespace gazebo {
/**
 * @brief The RCVPluginPrivate class
 * Here we store all the paramters and state we need
 * to calculate forces that are sent to the simulator.
 *
 * We also store objects which allow us to communicate
 * with gazebo and with ROS.
 */
class RCVPluginPrivate
{
public:
  /// \enum DirectionType
  /// \brief Direction selector switch type.
  enum DirectionType {
    /// \brief Reverse
    REVERSE = -1,
    /// \brief Neutral
    NEUTRAL = 0,
    /// \brief Forward
    FORWARD = 1
  };

  /// \brief this registers the class object to a "node" in
  /// the ROS system, so that it can communicate. We use nh
  /// to set up new connections
  ros::NodeHandle nh;

  /// \brief this object represents a connection to an input
  /// "topic" in the ros system
  ros::Subscriber controlSub;

  /// \brief Pointer to the world
  physics::WorldPtr world;

  /// \brief Pointer to the parent model
  physics::ModelPtr model;

  /// \brief Transport node
  transport::NodePtr gznode;

  /// \brief Ignition transport node
  ignition::transport::Node node;

  /// \brief Physics update event connection
  event::ConnectionPtr updateConnection;

  /* Links are rigid body entities of the model and joints are
   * relationships between them, e.g. the wheel axes
   * for each wheel are connected to the steering wheel axes.
   *
   * How the links go together are defined in the .urdf file
   * "rcv.urdf". You might need to measure some distances
   * and angles on the RCV to get it right.
   *
   * Basically how the simulator should interpret the geometry
   * of the model and it's properties (e.g. mass) is listed
   * in "rcv.urdf"
   *
   * If we follow the coordinate conventions of the prius
   * the chassis is rotated +pi/2 around the z axis so it's x-axis points to
   * the left and the y axis points backwards.
   *
   * In the simulator we will make the car move by applying
   * torques to the different joints. We will make the car move
   * forward and backwards by applying torqe to fl,fr,bl,brWheelJoint
   * and make it turn by applying torque to fl,fr,bl,brWheelSteeringJoint
   *
   * Look at axis in the urdf.
   * Eg. for front_left_wheel_joint, it's xyz="1 0 0"
   * so we will apply torque to an axis that is perpendicular to
   * the direction of travel.
   * For front_left_steer_joint axis xyz="0 0 1" so we will apply torque
   * to an axis pointing up.
   *
   * We connect these pointers (pointing to objects used by the simulator)
   * in the "Load" function.
   *
   */

  /// \brief Chassis link
  physics::LinkPtr chassisLink;

  /// \brief Front left wheel joint
  physics::JointPtr flWheelJoint;

  /// \brief Front right wheel joint
  physics::JointPtr frWheelJoint;

  /// \brief Rear left wheel joint
  physics::JointPtr blWheelJoint;

  /// \brief Rear right wheel joint
  physics::JointPtr brWheelJoint;

  /// \brief Front left wheel steering joint
  physics::JointPtr flWheelSteeringJoint;

  /// \brief Front right wheel steering joint
  physics::JointPtr frWheelSteeringJoint;

  ///@todo add pointers to bl,brSteeringJoint

  /// \brief Steering wheel joint
  /// Just here to make the wheel turn, which looks cool (we don't
  /// model the linkage between steering wheel and wheels)
  physics::JointPtr handWheelJoint;

  /*
   * We have a bunch of PID controllers so
   * that we can control the joints to achive
   * a desired angle.
   *
   * So the refernce is the steering angle we want according
   * to our model, the measured value the actual angle and
   * the control signal the torque.
   */

  /// \brief PID control for the front left wheel steering joint
  common::PID flWheelSteeringPID;

  /// \brief PID control for the front right wheel steering joint
  common::PID frWheelSteeringPID;

  ///@todo add PIDs for rear left/right wheel steering joints

  /// \brief PID control for steering wheel joint
  common::PID handWheelPID;

  /// \brief Last pose msg time
  common::Time lastMsgTime;

  /// \brief Last sim time received
  common::Time lastSimTime;

  /// \brief Last sim time when a pedal command is received
  common::Time lastPedalCmdTime;

  /// \brief Last sim time when a steering command is received
  common::Time lastSteeringCmdTime;

  /// \brief Last sim time when a EV mode command is received
  common::Time lastModeCmdTime;

  /// \brief Current direction of the vehicle: FORWARD, NEUTRAL, REVERSE.
  DirectionType directionState;

  /// \brief Chassis aerodynamic drag force coefficient,
  /// with units of [N / (m/s)^2]
  double chassisAeroForceGain = 0;

  /// \brief Max torque that can be applied to the front wheels
  double frontTorque = 0;

  /// \brief Max torque that can be applied to the back wheels
  double backTorque = 0;

  /// \brief Max speed (m/s) of the car
  double maxSpeed = 0;

  /// \brief Max steering angle
  double maxSteer = 0;

  /// \brief Max torque that can be applied to the front brakes
  double frontBrakeTorque = 0;

  /// \brief Max torque that can be applied to the rear brakes
  double backBrakeTorque = 0;

  /// \brief Angle ratio between the steering wheel and the front wheels
  double steeringRatio = 0;

  /// \brief Max range of hand steering wheel
  double handWheelHigh = 0;

  /// \brief Min range of hand steering wheel
  double handWheelLow = 0;

  /// \brief Front left wheel desired steering angle (radians)
  double flWheelSteeringCmd = 0;

  /// \brief Front right wheel desired steering angle (radians)
  double frWheelSteeringCmd = 0;

  /// \brief Steering wheel desired angle (radians)
  double handWheelCmd = 0;

  /// \brief Front left wheel radius
  double flWheelRadius = 0;

  /// \brief Front right wheel radius
  double frWheelRadius = 0;

  /// \brief Rear left wheel radius
  double blWheelRadius = 0;

  /// \brief Rear right wheel radius
  double brWheelRadius = 0;

  /// \brief Front left joint friction
  double flJointFriction = 0;

  /// \brief Front right joint friction
  double frJointFriction = 0;

  /// \brief Rear left joint friction
  double blJointFriction = 0;

  /// \brief Rear right joint friction
  double brJointFriction = 0;

  /// \brief Distance distance between front and rear axles
  double wheelbaseLength = 0;

  /// \brief Distance distance between front left and right wheels
  double frontTrackWidth = 0;

  /// \brief Distance distance between rear left and right wheels
  double backTrackWidth = 0;

  /// \brief Gas pedal position in percentage. 1.0 = Fully accelerated.
  double gasPedalPercent = 0;


  /// \brief Threshold delimiting the gas pedal (throttle) low and medium
  /// ranges.
  const double kGasPedalLowMedium = 0.25;

  /// \brief Threshold delimiting the gas pedal (throttle) medium and high
  /// ranges.
  const double kGasPedalMediumHigh = 0.5;

  ///@todo change below to propper SI units! (Spacecraft have crashed
  /// because of this nonsense)

  /// \brief Threshold delimiting the speed (throttle) low and medium
  /// ranges in miles/h.
  const double speedLowMedium = 26.0;

  /// \brief Threshold delimiting the speed (throttle) medium and high
  /// ranges in miles/h.
  const double speedMediumHigh = 46.0;

  /// \brief Brake pedal position in percentage. 1.0 =
  double brakePedalPercent = 0;

  /// \brief Hand brake position in percentage.
  double handbrakePercent = 1.0;

  /// \brief Angle of steering wheel at last update (radians)
  double handWheelAngle = 0;

  /// \brief Steering angle of front left wheel at last update (radians)
  double flSteeringAngle = 0;

  /// \brief Steering angle of front right wheel at last update (radians)
  double frSteeringAngle = 0;

  /// \brief Linear velocity of chassis c.g. in world frame at last update (m/s)
  ignition::math::Vector3d chassisLinearVelocity;

  /// \brief Angular velocity of front left wheel at last update (rad/s)
  double flWheelAngularVelocity = 0;

  /// \brief Angular velocity of front right wheel at last update (rad/s)
  double frWheelAngularVelocity = 0;

  /// \brief Angular velocity of back left wheel at last update (rad/s)
  double blWheelAngularVelocity = 0;

  /// \brief Angular velocity of back right wheel at last update (rad/s)
  double brWheelAngularVelocity = 0;

  /// \brief Subscriber to the keyboard topic
  transport::SubscriberPtr keyboardSub;

  /// \brief Mutex to protect updates
  std::mutex mutex;

  /// \brief Odometer
  double odom = 0.0;

  /// \brief Keyboard control type
  int keyControl = 1;

  /// \brief Publisher for the world_control topic.
  transport::PublisherPtr worldControlPub;
};
}

using namespace gazebo;

/////////////////////////////////////////////////
RCVPlugin::RCVPlugin()
  : dataPtr(new RCVPluginPrivate)
{
  int argc = 0;
  char *argv = nullptr;
  //here we tell ros that we are node in the system
  ros::init(argc, &argv, "RCVPlugin");
  ros::NodeHandle nh;
  ///@todo subsribe to control input messages
  /// simlar to this.
  //this->dataPtr->controlSub = nh.subscribe("prius", 10, &PriusHybridPlugin::OnPriusCommand, this);

  //Set some default values for parameters
  this->dataPtr->directionState = RCVPluginPrivate::FORWARD;
  this->dataPtr->flWheelRadius = 0.3;
  this->dataPtr->frWheelRadius = 0.3;
  this->dataPtr->blWheelRadius = 0.3;
  this->dataPtr->brWheelRadius = 0.3;
}

/////////////////////////////////////////////////
RCVPlugin::~RCVPlugin()
{
  this->dataPtr->updateConnection.reset();
}

/////////////////////////////////////////////////
void RCVPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzwarn << "RCVPlugin loading params" << std::endl;
  // shortcut to this->dataPtr
  RCVPluginPrivate *dPtr = this->dataPtr.get();

  //pointer to phisics model inside the simulator
  this->dataPtr->model = _model;
  this->dataPtr->world = this->dataPtr->model->GetWorld();
  auto physicsEngine = this->dataPtr->world->Physics();
  physicsEngine->SetParam("friction_model", std::string("cone_model"));

  this->dataPtr->gznode = transport::NodePtr(new transport::Node());
  this->dataPtr->gznode->Init();

  //Set callbacks for what happens when simulation
  //is reset, or stopped. This uses the "ignition" library
  //which provides simular functionality to ROS (e.g. message passing)
  //but is what is used in gazebo.
  this->dataPtr->node.Subscribe("/rcv/reset",
                                &RCVPlugin::OnReset, this);
  this->dataPtr->node.Subscribe("/rcv/stop",
                                &RCVPlugin::OnStop, this);

  //Set up the links (that we later apply forces on)
  //We query for names that originally come from the .urdf file to
  //find the names used inside gazebo (if you click the model inside
  //the gazebo GUI you will see these names, e.g. "rcv::chassis")

  std::string chassisLinkName = dPtr->model->GetName() + "::"
      + _sdf->Get<std::string>("chassis");
  dPtr->chassisLink = dPtr->model->GetLink(chassisLinkName);
  if (!dPtr->chassisLink)
  {
    std::cerr << "could not find chassis link" << std::endl;
    return;
  }

  std::string handWheelJointName = this->dataPtr->model->GetName() + "::"
      + _sdf->Get<std::string>("steering_wheel");
  this->dataPtr->handWheelJoint =
      this->dataPtr->model->GetJoint(handWheelJointName);
  if (!this->dataPtr->handWheelJoint)
  {
    std::cerr << "could not find steering wheel joint" <<std::endl;
    return;
  }

  std::string flWheelJointName = this->dataPtr->model->GetName() + "::"
      + _sdf->Get<std::string>("front_left_wheel");
  this->dataPtr->flWheelJoint =
      this->dataPtr->model->GetJoint(flWheelJointName);
  if (!this->dataPtr->flWheelJoint)
  {
    std::cerr << "could not find front left wheel joint" <<std::endl;
    return;
  }

  std::string frWheelJointName = this->dataPtr->model->GetName() + "::"
      + _sdf->Get<std::string>("front_right_wheel");
  this->dataPtr->frWheelJoint =
      this->dataPtr->model->GetJoint(frWheelJointName);
  if (!this->dataPtr->frWheelJoint)
  {
    std::cerr << "could not find front right wheel joint" <<std::endl;
    return;
  }

  std::string blWheelJointName = this->dataPtr->model->GetName() + "::"
      + _sdf->Get<std::string>("back_left_wheel");
  this->dataPtr->blWheelJoint =
      this->dataPtr->model->GetJoint(blWheelJointName);
  if (!this->dataPtr->blWheelJoint)
  {
    std::cerr << "could not find back left wheel joint" <<std::endl;
    return;
  }

  std::string brWheelJointName = this->dataPtr->model->GetName() + "::"
      + _sdf->Get<std::string>("back_right_wheel");
  this->dataPtr->brWheelJoint =
      this->dataPtr->model->GetJoint(brWheelJointName);
  if (!this->dataPtr->brWheelJoint)
  {
    std::cerr << "could not find back right wheel joint" <<std::endl;
    return;
  }

  std::string flWheelSteeringJointName = this->dataPtr->model->GetName() + "::"
      + _sdf->Get<std::string>("front_left_wheel_steering");
  this->dataPtr->flWheelSteeringJoint =
      this->dataPtr->model->GetJoint(flWheelSteeringJointName);
  if (!this->dataPtr->flWheelSteeringJoint)
  {
    std::cerr << "could not find front left steering joint" <<std::endl;
    return;
  }

  std::string frWheelSteeringJointName = this->dataPtr->model->GetName() + "::"
      + _sdf->Get<std::string>("front_right_wheel_steering");
  this->dataPtr->frWheelSteeringJoint =
      this->dataPtr->model->GetJoint(frWheelSteeringJointName);
  if (!this->dataPtr->frWheelSteeringJoint)
  {
    std::cerr << "could not find front right steering joint" <<std::endl;
    return;
  }

  ///@todo also set up blWheelSteeringJoint and beWheelSteeringJoint

  //Load parameters from urdf file, defined for this plugin, that
  //is attached to the Model.
  std::string paramName;
  double paramDefault;

  paramName = "chassis_aero_force_gain";
  paramDefault = 1;
  if (_sdf->HasElement(paramName))
    this->dataPtr->chassisAeroForceGain = _sdf->Get<double>(paramName);
  else
    this->dataPtr->chassisAeroForceGain = paramDefault;

  paramName = "front_torque";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frontTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->frontTorque = paramDefault;

  paramName = "back_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->backTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->backTorque = paramDefault;

  paramName = "front_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frontBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->frontBrakeTorque = paramDefault;

  paramName = "back_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->backBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->backBrakeTorque = paramDefault;

  paramName = "max_speed";
  paramDefault = 10;
  if (_sdf->HasElement(paramName))
    this->dataPtr->maxSpeed = _sdf->Get<double>(paramName);
  else
    this->dataPtr->maxSpeed = paramDefault;

  paramName = "max_steer";
  paramDefault = 0.6;
  if (_sdf->HasElement(paramName))
    this->dataPtr->maxSteer = _sdf->Get<double>(paramName);
  else
    this->dataPtr->maxSteer = paramDefault;

  paramName = "flwheel_steering_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetPGain(paramDefault);

  paramName = "frwheel_steering_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetPGain(paramDefault);

  paramName = "flwheel_steering_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetIGain(paramDefault);

  paramName = "frwheel_steering_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetIGain(paramDefault);

  paramName = "flwheel_steering_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetDGain(paramDefault);

  paramName = "frwheel_steering_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetDGain(paramDefault);

  ///@todo also set gains for blWheelSteering and brWheelSteering

  this->UpdateHandWheelRatio();

  // Update wheel radius for each wheel from SDF collision objects
  //  assumes that wheel link is child of joint (and not parent of joint)
  //  assumes that wheel link has only one collision
  unsigned int id = 0;
  this->dataPtr->flWheelRadius = this->CollisionRadius(
        this->dataPtr->flWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->frWheelRadius = this->CollisionRadius(
        this->dataPtr->frWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->blWheelRadius = this->CollisionRadius(
        this->dataPtr->blWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->brWheelRadius = this->CollisionRadius(
        this->dataPtr->brWheelJoint->GetChild()->GetCollision(id));

  // Get initial joint friction and add it to braking friction
  dPtr->flJointFriction = dPtr->flWheelJoint->GetParam("friction", 0);
  dPtr->frJointFriction = dPtr->frWheelJoint->GetParam("friction", 0);
  dPtr->blJointFriction = dPtr->blWheelJoint->GetParam("friction", 0);
  dPtr->brJointFriction = dPtr->brWheelJoint->GetParam("friction", 0);

  // Compute wheelbase, frontTrackWidth, and rearTrackWidth
  //  first compute the positions of the 4 wheel centers
  //  again assumes wheel link is child of joint and has only one collision
  ignition::math::Vector3d flCenterPos =
      this->dataPtr->flWheelJoint->GetChild()->GetCollision(id)
      ->WorldPose().Pos();
  ignition::math::Vector3d frCenterPos =
      this->dataPtr->frWheelJoint->GetChild()->GetCollision(id)
      ->WorldPose().Pos();
  ignition::math::Vector3d blCenterPos =
      this->dataPtr->blWheelJoint->GetChild()->GetCollision(id)
      ->WorldPose().Pos();
  ignition::math::Vector3d brCenterPos =
      this->dataPtr->brWheelJoint->GetChild()->GetCollision(id)
      ->WorldPose().Pos();

  // track widths are computed first
  ignition::math::Vector3d vec3 = flCenterPos - frCenterPos;
  this->dataPtr->frontTrackWidth = vec3.Length();
  vec3 = flCenterPos - frCenterPos;
  this->dataPtr->backTrackWidth = vec3.Length();
  // to compute wheelbase, first position of axle centers are computed
  ignition::math::Vector3d frontAxlePos = (flCenterPos + frCenterPos) / 2;
  ignition::math::Vector3d backAxlePos = (blCenterPos + brCenterPos) / 2;
  // then the wheelbase is the distance between the axle centers
  vec3 = frontAxlePos - backAxlePos;
  this->dataPtr->wheelbaseLength = vec3.Length();

  // gzerr << "wheel base length and track width: "
  //   << this->dataPtr->wheelbaseLength << " "
  //   << this->dataPtr->frontTrackWidth
  //   << " " << this->dataPtr->backTrackWidth << std::endl;

  // Max force that can be applied to hand steering wheel
  double handWheelForce = 10;
  this->dataPtr->handWheelPID.Init(100, 0, 10, 0, 0,
                                   handWheelForce, -handWheelForce);

  // Max force that can be applied to wheel steering joints
  double kMaxSteeringForceMagnitude = 5000;

  this->dataPtr->flWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  this->dataPtr->flWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  this->dataPtr->frWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  this->dataPtr->frWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  //Here we tell the simulator that the Update function should be called
  //for the simulation cycles
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RCVPlugin::Update, this));

  //we can also control the car with keyboard
  this->dataPtr->keyboardSub =
      this->dataPtr->gznode->Subscribe("~/keyboard/keypress",
                                       &RCVPlugin::OnKeyPress, this, true);

  this->dataPtr->worldControlPub =
      this->dataPtr->gznode->Advertise<msgs::WorldControl>("~/world_control");

  this->dataPtr->node.Subscribe("/keypress", &RCVPlugin::OnKeyPressIgn,
                                this);
}

/////////////////////////////////////////////////
void RCVPlugin::KeyControlTypeA(const int _key)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  switch (_key)
  {
    // e - gas pedal
    case 69:
    case 101:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      this->dataPtr->gasPedalPercent += 0.1;
      this->dataPtr->gasPedalPercent =
          std::min(this->dataPtr->gasPedalPercent, 1.0);
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // w - release pedals
    case 87:
    case 119:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // q - brake
    case 113:
    {
      this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->brakePedalPercent += 0.1;
      this->dataPtr->brakePedalPercent =
          std::min(this->dataPtr->brakePedalPercent, 1.0);
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // a - steer left
    case 65:
    case 97:
    {
      this->dataPtr->handWheelCmd += 0.25;
      this->dataPtr->handWheelCmd = std::min(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelHigh);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // d - steer right
    case 68:
    case 100:
    {
      this->dataPtr->handWheelCmd -= 0.25;
      this->dataPtr->handWheelCmd = std::max(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelLow);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // s - center steering
    case 83:
    case 115:
    {
      this->dataPtr->handWheelCmd = 0;
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // z reverse
    case 90:
    case 122:
    {
      this->dataPtr->directionState = RCVPluginPrivate::REVERSE;
      break;
    }
    // x neutral
    case 88:
    case 120:
    {
      this->dataPtr->directionState = RCVPluginPrivate::NEUTRAL;
      break;
    }
    // c forward
    case 67:
    case 99:
    {
      this->dataPtr->directionState = RCVPluginPrivate::FORWARD;
      break;
    }

    default:
    {
      this->dataPtr->brakePedalPercent = 0;
      this->dataPtr->gasPedalPercent = 0;
      break;
    }
  }
}

/////////////////////////////////////////////////
void RCVPlugin::KeyControlTypeB(const int _key)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  switch (_key)
  {
    // w - accelerate forward
    case 87:
    case 119:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      this->dataPtr->gasPedalPercent += 0.1;
      this->dataPtr->gasPedalPercent =
          std::min(this->dataPtr->gasPedalPercent, 1.0);
      this->dataPtr->directionState = RCVPluginPrivate::FORWARD;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // a - steer left
    case 65:
    case 97:
    {
      this->dataPtr->handWheelCmd += 0.25;
      this->dataPtr->handWheelCmd = std::min(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelHigh);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // s - reverse
    case 83:
    case 115:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      if (this->dataPtr->directionState != RCVPluginPrivate::REVERSE)
        this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->gasPedalPercent += 0.1;
      this->dataPtr->gasPedalPercent =
          std::min(this->dataPtr->gasPedalPercent, 1.0);
      this->dataPtr->directionState = RCVPluginPrivate::REVERSE;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // d - steer right
    case 68:
    case 100:
    {
      this->dataPtr->handWheelCmd -= 0.25;
      this->dataPtr->handWheelCmd = std::max(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelLow);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // e brake
    case 69:
    case 101:
    {
      this->dataPtr->brakePedalPercent = 1.0;
      this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // x neutral
    case 88:
    case 120:
    {
      this->dataPtr->directionState = RCVPluginPrivate::NEUTRAL;
      break;
    }
    // q
    case 81:
    case 113:
    default:
    {
      break;
    }
  }
}

/////////////////////////////////////////////////
void RCVPlugin::KeyControl(const int _key)
{
  if (this->dataPtr->keyControl == 0)
    this->KeyControlTypeA(_key);
  else if (this->dataPtr->keyControl == 1)
    this->KeyControlTypeB(_key);
}

/////////////////////////////////////////////////
void RCVPlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  this->KeyControl(_msg->int_value());
}

/////////////////////////////////////////////////
void RCVPlugin::OnKeyPressIgn(const ignition::msgs::Any &_msg)
{
  this->KeyControl(_msg.int_value());
}

/////////////////////////////////////////////////
void RCVPlugin::OnReset(const ignition::msgs::Any & /*_msg*/)
{
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);

  this->dataPtr->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void RCVPlugin::OnStop(const ignition::msgs::Any & /*_msg*/)
{
  ignition::msgs::StringMsg req;
  ignition::msgs::StringMsg rep;
  bool result = false;
  unsigned int timeout = 5000;
  bool executed = this->dataPtr->node.Request("/priuscup/upload",
      req, timeout, rep, result);
  if (executed)
  {
    std::cerr << "Result: " << result << std::endl;
    std::cerr << rep.data() << std::endl;
  }
  else
  {
    std::cerr << "Service call timed out" << std::endl;
  }
}

/////////////////////////////////////////////////
void RCVPlugin::Reset()
{
  this->dataPtr->odom = 0;
  this->dataPtr->flWheelSteeringPID.Reset();
  this->dataPtr->frWheelSteeringPID.Reset();
  this->dataPtr->handWheelPID.Reset();
  this->dataPtr->lastMsgTime = 0;
  this->dataPtr->lastSimTime = 0;
  this->dataPtr->lastModeCmdTime = 0;
  this->dataPtr->lastPedalCmdTime = 0;
  this->dataPtr->lastSteeringCmdTime = 0;
  this->dataPtr->directionState = RCVPluginPrivate::FORWARD;
  this->dataPtr->flWheelSteeringCmd = 0;
  this->dataPtr->frWheelSteeringCmd = 0;
  this->dataPtr->handWheelCmd = 0;
  this->dataPtr->gasPedalPercent = 0;
  this->dataPtr->brakePedalPercent = 0;
  this->dataPtr->handbrakePercent = 1.0;
  this->dataPtr->handWheelAngle  = 0;
  this->dataPtr->flSteeringAngle = 0;
  this->dataPtr->frSteeringAngle = 0;
  this->dataPtr->flWheelAngularVelocity  = 0;
  this->dataPtr->frWheelAngularVelocity = 0;
  this->dataPtr->blWheelAngularVelocity = 0;
  this->dataPtr->brWheelAngularVelocity  = 0;
}

/////////////////////////////////////////////////
void RCVPlugin::Update()
{
  // shortcut to this->dataPtr
  RCVPluginPrivate *dPtr = this->dataPtr.get();

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  common::Time curTime = this->dataPtr->world->SimTime();
  double dt = (curTime - this->dataPtr->lastSimTime).Double();
  if (dt < 0)
  {
    this->Reset();
    return;
  }
  else if (ignition::math::equal(dt, 0.0))
  {
    return;
  }

  //Get data from simulation, and store for later computations
  dPtr->handWheelAngle = dPtr->handWheelJoint->Position();
  dPtr->flSteeringAngle = dPtr->flWheelSteeringJoint->Position();
  dPtr->frSteeringAngle = dPtr->frWheelSteeringJoint->Position();

  dPtr->flWheelAngularVelocity = dPtr->flWheelJoint->GetVelocity(0);
  dPtr->frWheelAngularVelocity = dPtr->frWheelJoint->GetVelocity(0);
  dPtr->blWheelAngularVelocity = dPtr->blWheelJoint->GetVelocity(0);
  dPtr->brWheelAngularVelocity = dPtr->brWheelJoint->GetVelocity(0);

  dPtr->chassisLinearVelocity = dPtr->chassisLink->WorldCoGLinearVel();

  ///@todo get rid of non SI units!

  // Convert meter/sec to miles/hour
  double linearVel = dPtr->chassisLinearVelocity.Length() * 2.23694;

  // Distance traveled in miles.
  this->dataPtr->odom += (fabs(linearVel) * dt/3600.0);

  bool neutral = dPtr->directionState == RCVPluginPrivate::NEUTRAL;

  this->dataPtr->lastSimTime = curTime;

  // Aero-dynamic drag on chassis
  // F: force in world frame, applied at center of mass
  // V: velocity in world frame of chassis center of mass
  // C: drag coefficient based on straight-ahead driving [N / (m/s)^2]
  // |V|: speed
  // V_hat: velocity unit vector
  // F = -C |V|^2 V_hat
  auto dragForce = -dPtr->chassisAeroForceGain *
        dPtr->chassisLinearVelocity.SquaredLength() *
        dPtr->chassisLinearVelocity.Normalized();
  dPtr->chassisLink->AddForce(dragForce);

  // PID (position) steering
  this->dataPtr->handWheelCmd =
    ignition::math::clamp(this->dataPtr->handWheelCmd,
        -this->dataPtr->maxSteer / this->dataPtr->steeringRatio,
        this->dataPtr->maxSteer / this->dataPtr->steeringRatio);
  double steerError =
      this->dataPtr->handWheelAngle - this->dataPtr->handWheelCmd;
  double steerCmd = this->dataPtr->handWheelPID.Update(steerError, dt);
  this->dataPtr->handWheelJoint->SetForce(0, steerCmd);

  ///@todo replace with the model for the RCV!!!
  /// and compute steering angles for all wheels!
  // PID (position) steering joints based on steering position
  // Ackermann steering geometry here
  //  \TODO provide documentation for these equations
  double tanSteer =
      tan(this->dataPtr->handWheelCmd * this->dataPtr->steeringRatio);
  this->dataPtr->flWheelSteeringCmd = atan2(tanSteer,
      1 - this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength *
      tanSteer);
  this->dataPtr->frWheelSteeringCmd = atan2(tanSteer,
      1 + this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength *
      tanSteer);
  // this->flWheelSteeringCmd = this->handWheelAngle * this->steeringRatio;
  // this->frWheelSteeringCmd = this->handWheelAngle * this->steeringRatio;

  double flwsError =
      this->dataPtr->flSteeringAngle - this->dataPtr->flWheelSteeringCmd;
  double flwsCmd = this->dataPtr->flWheelSteeringPID.Update(flwsError, dt);
  this->dataPtr->flWheelSteeringJoint->SetForce(0, flwsCmd);


  double frwsError =
      this->dataPtr->frSteeringAngle - this->dataPtr->frWheelSteeringCmd;
  double frwsCmd = this->dataPtr->frWheelSteeringPID.Update(frwsError, dt);
  this->dataPtr->frWheelSteeringJoint->SetForce(0, frwsCmd);

  ///@todo this longitudinal model might not be the best for the
  /// RCV, replace it.
  // Model low-speed creep and high-speed regen braking
  // with term added to gas/brake
  // Cross-over speed is 7 miles/hour
  // 10% throttle at 0 speed
  // max 2.5% braking at higher speeds
  double creepPercent;
  if (std::abs(linearVel) <= 7)
  {
    creepPercent = 0.1 * (1 - std::abs(linearVel) / 7);
  }
  else
  {
    creepPercent = 0.025 * (7 - std::abs(linearVel));
  }
  creepPercent = ignition::math::clamp(creepPercent, -0.025, 0.1);

  // Gas pedal torque.
  // Map gas torques to individual wheels.
  // Cut off gas torque at a given wheel if max speed is exceeded.
  // Use directionState to determine direction of that can be applied torque.
  // Note that definition of DirectionType allows multiplication to determine
  // torque direction.
  // also, make sure gas pedal is at least as large as the creepPercent.
  double gasPercent = std::max(this->dataPtr->gasPedalPercent, creepPercent);
  double gasMultiplier = this->GasTorqueMultiplier();
  double flGasTorque = 0, frGasTorque = 0, blGasTorque = 0, brGasTorque = 0;
  // Apply equal torque at left and right wheels, which is an implicit model
  // of the differential.
  if (fabs(dPtr->flWheelAngularVelocity * dPtr->flWheelRadius) < dPtr->maxSpeed &&
      fabs(dPtr->frWheelAngularVelocity * dPtr->frWheelRadius) < dPtr->maxSpeed)
  {
    flGasTorque = gasPercent*dPtr->frontTorque * gasMultiplier;
    frGasTorque = gasPercent*dPtr->frontTorque * gasMultiplier;
  }
  if (fabs(dPtr->blWheelAngularVelocity * dPtr->blWheelRadius) < dPtr->maxSpeed &&
      fabs(dPtr->brWheelAngularVelocity * dPtr->brWheelRadius) < dPtr->maxSpeed)
  {
    blGasTorque = gasPercent * dPtr->backTorque * gasMultiplier;
    brGasTorque = gasPercent * dPtr->backTorque * gasMultiplier;
  }

  // auto release handbrake as soon as the gas pedal is depressed
  if (this->dataPtr->gasPedalPercent > 0)
    this->dataPtr->handbrakePercent = 0.0;

  double brakePercent = this->dataPtr->brakePedalPercent
      + this->dataPtr->handbrakePercent;
  // use creep braking if not in Neutral
  if (!neutral)
  {
    brakePercent = std::max(brakePercent,
        -creepPercent - this->dataPtr->gasPedalPercent);
  }

  brakePercent = ignition::math::clamp(brakePercent, 0.0, 1.0);
  dPtr->flWheelJoint->SetParam("friction", 0,
      dPtr->flJointFriction + brakePercent * dPtr->frontBrakeTorque);
  dPtr->frWheelJoint->SetParam("friction", 0,
      dPtr->frJointFriction + brakePercent * dPtr->frontBrakeTorque);
  dPtr->blWheelJoint->SetParam("friction", 0,
      dPtr->blJointFriction + brakePercent * dPtr->backBrakeTorque);
  dPtr->brWheelJoint->SetParam("friction", 0,
      dPtr->brJointFriction + brakePercent * dPtr->backBrakeTorque);

  this->dataPtr->flWheelJoint->SetForce(0, flGasTorque);
  this->dataPtr->frWheelJoint->SetForce(0, frGasTorque);
  this->dataPtr->blWheelJoint->SetForce(0, blGasTorque);
  this->dataPtr->brWheelJoint->SetForce(0, brGasTorque);


  ///@todo it might be cool to output some data, e.g. current
  /// speed, gear etc (look at lines 1249-1300 in PriusHybridPlugin)
  /// I suggest that you use ROS topics instead of ignition topics for
  /// this.

  // reset if last command is more than x sec ago
  if ((curTime - this->dataPtr->lastPedalCmdTime).Double() > 0.3)
  {
    this->dataPtr->gasPedalPercent = 0.0;
    this->dataPtr->brakePedalPercent = 0.0;
  }

  if ((curTime - this->dataPtr->lastSteeringCmdTime).Double() > 0.3)
  {
    this->dataPtr->handWheelCmd = 0;
  }
}

/////////////////////////////////////////////////
void RCVPlugin::UpdateHandWheelRatio()
{
  // The total range the steering wheel can rotate
  this->dataPtr->handWheelHigh = 7.85;
  this->dataPtr->handWheelLow = -7.85;
  double handWheelRange =
      this->dataPtr->handWheelHigh - this->dataPtr->handWheelLow;
  double high = 0.8727;
  high = std::min(high, this->dataPtr->maxSteer);
  double low = -0.8727;
  low = std::max(low, -this->dataPtr->maxSteer);
  double tireAngleRange = high - low;

  // Compute the angle ratio between the steering wheel and the tires
  this->dataPtr->steeringRatio = tireAngleRange / handWheelRange;
}

/////////////////////////////////////////////////
// function that extracts the radius of a cylinder or sphere collision shape
// the function returns zero otherwise
double RCVPlugin::CollisionRadius(physics::CollisionPtr _coll)
{
  if (!_coll || !(_coll->GetShape()))
    return 0;
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
  {
    physics::CylinderShape *cyl =
        static_cast<physics::CylinderShape*>(_coll->GetShape().get());
    return cyl->GetRadius();
  }
  else if (_coll->GetShape()->HasType(physics::Base::SPHERE_SHAPE))
  {
    physics::SphereShape *sph =
        static_cast<physics::SphereShape*>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

/////////////////////////////////////////////////
double RCVPlugin::GasTorqueMultiplier()
{
  // if (this->dataPtr->keyState == ON)
  {
    if (this->dataPtr->directionState == RCVPluginPrivate::FORWARD)
      return 1.0;
    else if (this->dataPtr->directionState == RCVPluginPrivate::REVERSE)
      return -1.0;
  }
  return 0;
}

GZ_REGISTER_MODEL_PLUGIN(RCVPlugin)
