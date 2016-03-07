#ifndef RTT_GAZEBO_LWR_SIMULATION_HPP
#define RTT_GAZEBO_LWR_SIMULATION_HPP

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>
#include <rci/dto/JointImpedance.h>
#include <rci/dto/CartesianImpedance.h>
#include <rci/dto/JointAngles.h>
#include <rci/dto/JointTorques.h>
#include <rci/dto/JointVelocities.h>
#include <rci/dto/JointAccelerations.h>
#include <rci/dto/CartesianVelocity.h>
#include <rci/dto/CartesianPose.h>
#include <nemo/Vector.h>
#include <nemo/Mapping.h>
#include <nemo/Matrix.h>
#include <kdl/tree.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <boost/scoped_ptr.hpp>

#include <Eigen/Core>

#define TORAD 3.141592653589793/180.0

#define DEFAULT_ROOT_LINK "lwr_arm_base_link"
#define DEFAULT_TIP_LINK "lwr_arm_7_link"
#define DEFAULT_NR_JOINTS_LWR 7

inline float clamp(const float& x, const float& a, const float& b) {
	return x < a ? a : (x > b ? b : x);
}

class RTTGazeboLWRSimulation: public RTT::TaskContext {
public:
	RTTGazeboLWRSimulation(std::string const& name);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();

protected:
	struct JointState {
		std::vector<std::string> names;
		Eigen::VectorXd positions;
		Eigen::VectorXd velocities;
		Eigen::VectorXd efforts; // TODO ? what does effort mean?
	};

	enum FRI_RobotCtrlMode {
		FRI_CTRL_JNT_IMP,
		FRI_CTRL_POSITION,
		FRI_CTRL_CART_IMP,
		FRI_CTRL_DIRECT_TORQUE,
		CountEntries_RCM
	};

	void setJointImpedanceMode();
	void setCartesianImpedanceMode();
	void resetJointImpedanceGains();
	void setInitialJointPositionOutside(double j0, double j1, double j2, double j3, double j4, double j5, double j6);

	void resetCartesianImpedanceGains();
	void initJointStateFromKDLCHain(const KDL::Chain& kdl_chain,
			JointState& joint_state);
	bool parseURDFforKDL(std::string urdfString);

	bool setGravityMode();
	bool setJointImpedance(const rci::JointStiffness& stiffness,
			const rci::JointDamping& damping);
//    bool setCartesianImpedance(const rci::CartesianImpedance& cart_stiffness, const Eigen::Matrix<double,6,1> & cart_damping);  TODO ??

	bool safetyCheck(const nemo::RealVector& v, const nemo::RealVector& limits,
			const std::string& name);
	bool safetyChecks(rci::JointAnglesPtr position,
			rci::JointVelocitiesPtr velocity, rci::JointTorquesPtr torque);
	void initSafetyConstraints();

	void convertRealVectorToEigenVectorXd(const nemo::RealVector& realV,
			Eigen::VectorXd& vXd);

	void convertPoseKDL2PoseRCI(const KDL::Frame& frame, rci::PosePtr pose);

	void chooseControllerBasedOnJointCtrlModes(
			const std::vector<FRI_RobotCtrlMode>& modes_);

	void callControllerForCtrlMode(const FRI_RobotCtrlMode& mode_);

	void executeJointPositionController();
	void executeJointImpedanceController();
	void tunnelDirectTorque();

	void createGazeboTrqCommandAccordingToCtrlModes(
			const std::vector<FRI_RobotCtrlMode>& modes_,
			const std::vector<rci::JointTorquesPtr>& pack,
			rci::JointTorquesPtr outTrqCmd);

	void clampImpedance(rci::JointImpedancePtr imp,
			rci::JointImpedancePtr limits);

	void setInitialJointPosition(rci::JointAnglesPtr j_init);

	bool setControlMode(int index, std::string mode);

	/** Input-Ports (data from Controller) */

	RTT::FlowStatus jnt_pos_cmd_fs;
	RTT::FlowStatus jnt_vel_cmd_fs;
	RTT::FlowStatus jnt_trq_cmd_fs;

	RTT::InputPort<rci::JointImpedancePtr> port_JointImpedanceCommand;
	RTT::InputPort<rci::JointAnglesPtr> port_JointPositionCommand;
	RTT::InputPort<rci::JointTorquesPtr> port_JointTorqueCommand;
	RTT::InputPort<rci::JointVelocitiesPtr> port_JointVelocityCommand;
	// KRL stuff
//	RTT::InputPort<tFriKrlData> port_ToKRL;

	/** Output-Ports (data to Controller) */

//	RTT::OutputPort<geometry_msgs::Wrench > port_CartesianWrench;
	RTT::OutputPort<rci::JointVelocitiesPtr> port_JointVelocity;
//	RTT::OutputPort<rci::CartesianVelocityPtr> port_CartesianVelocity;
	RTT::OutputPort<rci::PosePtr> port_CartesianPosition;
//	RTT::OutputPort<nemo::RealMatrix> port_MassMatrix;
//	RTT::OutputPort<KDL::Jacobian > port_Jacobian;
	RTT::OutputPort<rci::JointTorquesPtr> port_JointTorque, port_GravityTorque;
	RTT::OutputPort<rci::JointAnglesPtr> port_JointPosition;

	/** Input-Ports (data from Gazebo) */

	RTT::InputPort<rci::JointAnglesPtr> port_JointPositionGazebo;
	RTT::FlowStatus fs_p;
	RTT::InputPort<rci::JointTorquesPtr> port_JointTorqueGazebo;
	RTT::FlowStatus fs_v;
	RTT::InputPort<rci::JointVelocitiesPtr> port_JointVelocityGazebo;
	RTT::FlowStatus fs_g;

	/** Output-Ports (data to Gazebo) */

	// only used to set the inital joint position
	RTT::OutputPort<rci::JointAnglesPtr> port_JointPositionGazeboCommand;
	RTT::OutputPort<rci::JointTorquesPtr> port_JointTorqueGazeboCommand;
	RTT::OutputPort<rci::JointVelocitiesPtr> port_JointVelocityGazeboCommand; // TODO not yet used!

	rci::JointTorquesPtr joint_torque_gazebo_cmd;
	// holds output for different joint based on their ctrl mode
	std::vector<rci::JointTorquesPtr> joint_torque_gazebo_cmd_tmp;

//	RTT::FlowStatus jnt_pos_cmd_fs, jnt_trq_cmd_fs;

	/** Different KDL Solver */
	boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;
	boost::scoped_ptr<KDL::ChainDynParam> id_dyn_solver;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
	boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_rne_solver;

	/** Robot Joint State */
	std::vector<FRI_RobotCtrlMode> joint_control_modes_;

	// KDL fields
	KDL::Vector gravity_vector;
	KDL::Chain kdl_chain_;
	KDL::Tree kdl_tree_;

	KDL::Wrenches f_ext;
	KDL::JntArray G, qdot, qddot, jnt_trq_kdl_, jnt_trq_coriolis_kdl_;
	KDL::Wrench cart_wrench_kdl_;
	KDL::JntArrayVel jntArrVelConfig_q;
	KDL::JntSpaceInertiaMatrix H;
	Eigen::MatrixXd mass_;
	KDL::Jacobian jac_;
	KDL::FrameVel ee_framevel_kdl_;

//	KDL::Twist ee_twist_kdl_; // dlw leave twist out
	KDL::Frame ee_frame_kdl_;

	// for conversion to use KDL
	Eigen::VectorXd jnt_pos_, jnt_vel_, jnt_trq_;
	Eigen::VectorXd grav_trq_;

	/** ####### Storage fields ####### */
	JointState joint_state_, joint_state_cmd_, joint_state_gravity_,
			joint_state_dyn_;

	// feedback from Gazebo
	rci::JointAnglesPtr joint_position_gazebo;
	rci::JointVelocitiesPtr joint_velocity_gazebo;
	rci::JointTorquesPtr joint_torque_gazebo;

	// send gravity to controller-side
	rci::JointTorquesPtr joint_torque_gravity;


	// incoming cmds from the outside
	rci::JointAnglesPtr joint_pos_cmd_;
	Eigen::VectorXd jnt_pos_cmd_;

	rci::JointVelocitiesPtr joint_vel_cmd_;
	Eigen::VectorXd jnt_vel_cmd_;

	rci::JointTorquesPtr joint_trq_cmd_;
	Eigen::VectorXd jnt_trq_cmd_;

	rci::JointImpedancePtr joint_imp_cmd_;
//	Eigen::VectorXd jnt_imp_cmd_;

	// feedback to the outside
	rci::PosePtr cart_pos_;

	// joint limits / safety checks
	nemo::RealVector pos_limits_, vel_limits_, trq_limits_;
	rci::JointImpedancePtr impedance_limits_;

	/** ####### STORAGE FIELDS FOR CONTROLLERS ####### */
	Eigen::VectorXd jnt_trq_gazebo_cmd_;

	// Control gains
	Eigen::VectorXd kp_, kd_, kg_;

	/** ####### MISC ####### */

	// container for URDF string
	std::string urdfContainer;
	// name of the root link for the KDL chain
	std::string chain_root_link_name;
	// name of the tip link for the KDL chain
	std::string chain_tip_link_name;
	// period of the simulation
	double period_sim_;
	double read_start, write_start, read_duration;

	bool once;

};
#endif
