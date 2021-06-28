#ifndef THRUSTERS_PLUGIN_HH
#define THRUSTERS_PLUGIN_HH

#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <utils/CommandLong.h>
#include <utils/ThrustCommand.h>
#include <utils/KeyboardCommand.h>
#include <gazebo/physics/physics.hh>
#include "utils/motor_stamped.h"

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <complex>

#include "math.h"
using namespace utils;
namespace gazebo
{
  // Foward declaration of AUVThrust class
  class AUVThrust;

  /// \brief Thruster class
  class Thruster
  {
    /// \brief Constructor
    /// \param[in] _parent Pointer to an SDF element to parse.
    public: explicit Thruster(AUVThrust *_parent);

    
    /// \brief Max forward force in Newtons.
    public: double maxForceFwd;

    /// \brief Max reverse force in Newtons.
    public: double maxForceRev;

    /// \brief Parameters of GLF pwm-force characteristic
    public: complex<double> Af, Bf, Cf, Mf, Kf, vf;
    public: complex<double> Ar, Br, Cr, Mr, Kr, vr;
    public: double pwmDf, pwmDr;

    /// \brief Link where thrust force is applied.
    public: physics::LinkPtr link;

    /// \brief Thruster mapping (0=linear; 1=GLF, nonlinear).
    public: int mappingType;

    /// \brief Current, most recent command.
    public: double currCmd;

    /// \brief Joint controlling the propeller.
    public: physics::JointPtr propJoint;

    /// \brief Plugin parent pointer - for accessing world, etc.
    protected: AUVThrust *plugin;
  };

  class AUVThrust : public ModelPlugin
  {
    /// \brief Constructor.
    public: AUVThrust() = default;

    /// \brief Destructor.
    public: virtual ~AUVThrust() = default;

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _parent,
                              sdf::ElementPtr _sdf);
    private: physics::LinkPtr link;
    /// \brief Callback executed at every physics update.
    protected: virtual void Update();

    /// \brief Convenience function for getting SDF parameters.
    /// \param[in] _sdfPtr Pointer to an SDF element to parse.
    /// \param[in] _paramName The name of the element to parse.
    /// \param[in] _defaultVal The default value returned if the element
    /// does not exist.
    /// \return The value parsed.
    private: double SdfParamDouble(sdf::ElementPtr _sdfPtr,
                                   const std::string &_paramName,
                                   const double _defaultVal) const;

    /// \brief Get GLF Parameters for each thruster
    private: void parseGLFParams(sdf::ElementPtr _sdfPtr, Thruster& thruster);

    /// \brief Takes ROS Drive commands and scales them by max thrust.
    /// \param[in] _cmd ROS drive command.
    /// \return Value scaled and saturated.
    private: double ScaleThrustCmd(const double _cmd,
                                   const double _max_pos,
                                   const double _max_neg) const;

    /// \brief Generalized logistic function (GLF) used for non-linear
    /// thruster model.
    /// \param[in] _x Independent variable (input) of GLF.
    /// \param[in] _A Lower asymptote.
    /// \param[in] _K Upper asymptote.
    /// \param[in] _B Growth rate
    /// \param[in] _v Affects near which asymptote max. growth occurs.
    /// \param[in] _C Typically near 1.0.
    /// \param[in] _M Offset to input.
    /// \return
    private: double Glf(const double _x,
                        const complex<double>  _A,
                        const complex<double>  _K,
                        const complex<double>  _B,
                        const complex<double>  _v,
                        const complex<double>  _C,
                        const complex<double>  _M) const;

    /// \brief Uses GLF function to map thrust command to thruster force
    /// in Newtons.
    /// \param[in] _cmd Thrust command {-1.0,1.0}.
    /// \param[in] _i Index of thruster to apply force
    /// \return Thrust force [N].
    private: double GlfThrustCmd(const double _cmd, size_t _i) const;

    /// \brief Spin a propeller based on its current command
    /// \param[in] _i Index of thruster whose propeller will be spun
    private: void SpinPropeller(size_t _i);

    /// \brief Callback for services from GCS.
    /// \param[in] _req The request service from client.
    /// \param[out] _res The response service from server.


    


    public: bool onSetArmingCallBack(utils::CommandLongRequest &_req,
                                     utils::CommandLongResponse &_res);

    /// \brief Callback for new thrust commands.
    /// \param[in] _msg The thrust message to process.
    public: void OnThrustCmdCallBack(const utils::motor_stamped::ConstPtr &_msg);

    /// \brief Callback for new thrust commands from keyboard.
    /// \param[in] _msg The thrust message to process.
    public: void OnThrustCmdKeyboardCallBack(const utils::KeyboardCommand::ConstPtr &_msg);


    public: double convertSpeedToForce(float speed);
    /// \brief A mutex to protect member variables accessed during
    /// OnThustCmd() and Update().
    public: std::mutex mutex;

    /// \brief The ROS node handler used for communications.
    private: std::unique_ptr<ros::NodeHandle> rosnode;

    /// \brief Pointer to the Gazebo world, retrieved when the model is loaded.
    public: physics::WorldPtr world;

    /// \brief Pointer to Gazebo parent model, retrieved when the model is
    /// loaded.
    private: physics::ModelPtr model;

    /// \brief Timeout for receiving Drive commands [s].
    private: double cmdTimeout;

    /// \brief Topic name for incoming ROS thruster commands.
    public: std::string cmdTopic;
    public: std::string pubTopic;

    /// \brief Subscription to thruster commands.
    public: ros::Subscriber cmdSub;
    public: ros::Publisher pubSpeed;
    /// \brief Topic name for incoming ROS thruster commands from keyboard.
    public: std::string cmdKeyboardTopic;

    /// \brief Subscription to thruster commands from keyboard.
    public: ros::Subscriber cmdKeyboardSub;

    /// \brief Last time received a command via ROS topic.
    public: common::Time lastCmdTime;

    /// \brief Service name for incoming GCS commands.
    public: std::string setArmingService;

    /// \brief Subscription to GCS commands.
    public: ros::ServiceServer resSetArming;

    
   public: int cnt ;
    /// \brief Propeller-locking status from GCS.
    public: bool propLocked = true;

    /// \brief Keyboard control status from keyboard.
    public: bool keyboardEnabled = true;

    /// \brief Vector of thruster instances
    private: std::vector<Thruster> thrusters;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif // THRUSTERS_PLUGIN_HH
