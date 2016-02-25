#include "rtt-gazebo-lwr-simulation.hpp"
#include <rtt/Component.hpp>
#include <iostream>

#include "parsertools/KDLParser.hpp"

using namespace std;
using namespace RTT;
using namespace Orocos;

#define l(lvl) log(lvl) << "[" << this->getName() << "] "

RTTGazeboLWRSimulation::RTTGazeboLWRSimulation(std::string const& name) :
		TaskContext(name), chain_root_link_name(DEFAULT_ROOT_LINK), chain_tip_link_name(
		DEFAULT_TIP_LINK), gravity_vector(0.,0.,-9.81289) {

	this->ports()->addPort("JointPositionGazeboCommand",
			port_JointPositionGazeboCommand).doc(
			"Output for JointPosition-cmds from Orocos to Gazebo world.");
	this->ports()->addPort("JointVelocityGazeboCommand",
			port_JointVelocityGazeboCommand).doc(
			"Output for JointVelocity-cmds from Orocos to Gazebo world.");
	this->ports()->addPort("JointTorqueGazeboCommand",
			port_JointTorqueGazeboCommand).doc(
			"Output for JointTorque-cmds from Orocos to Gazebo world.");

	this->ports()->addPort("JointPositionGazebo", port_JointPositionGazebo).doc(
			"Input for JointPosition-fbs from Gazebo to Orocos world.");
	this->ports()->addPort("JointVelocityGazebo", port_JointVelocityGazebo).doc(
			"Input for JointVelocity-fbs from Gazebo to Orocos world.");
	this->ports()->addPort("JointTorqueGazebo", port_JointTorqueGazebo).doc(
			"Input for JointTorque-fbs from Gazebo to Orocos world.");

	// input ports to the simulation from the controller-side
	this->ports()->addPort("JointImpedanceCommand", port_JointImpedanceCommand).doc(
			"");
	this->ports()->addPort("JointPositionCommand", port_JointPositionCommand).doc(
			"");
	this->ports()->addPort("JointTorqueCommand", port_JointTorqueCommand).doc(
			"");

	this->addOperation("parseURDFforKDL",
			&RTTGazeboLWRSimulation::parseURDFforKDL, this, OwnThread).doc(
			"Parses a URDF string to create a KDL::Tree.").arg("urdfString",
			"URDF string to parse.");

	this->addOperation("setControlMode",
			&RTTGazeboLWRSimulation::setControlMode, this, OwnThread).doc(
			"Sets the control mode for a single joint").arg("index",
			"joint index").arg("mode", "Control Mode");

//    this->ports()->addPort("toKRL",port_ToKRL).doc("");
//    this->ports()->addPort("fromKRL",port_FromKRL).doc("");

//    this->ports()->addPort("RobotState", port_RobotState).doc("");
//    this->ports()->addPort("FRIState", port_FRIState).doc("");
	this->ports()->addPort("JointVelocity", port_JointVelocity).doc(
			"Output for JointVelocity-fbs from Gazebo to Orocos world.");
//	this->ports()->addPort("CartesianVelocity", port_CartesianVelocity).doc("");
	this->ports()->addPort("CartesianPosition", port_CartesianPosition).doc(
			"Output for Cartesian-Endeff-Position-fbs from Gazebo to Orocos world.");
//	this->ports()->addPort("MassMatrix", port_MassMatrix).doc("");
//   this->ports()->addPort("Jacobian", port_Jacobian).doc("");
	this->ports()->addPort("JointTorque", port_JointTorque).doc(
			"Output for JointTorque-fbs from Gazebo to Orocos world.");
	this->ports()->addPort("GravityTorque", port_GravityTorque).doc("");
	this->ports()->addPort("JointPosition", port_JointPosition).doc(
			"Output for JointPosition-fbs from Gazebo to Orocos world.");

//    this->ports()->addPort("JointState",port_JointState).doc(""); // ? TODO
//    this->ports()->addPort("JointStateCommand",port_JointStateCommand).doc(""); // ? TODO
//    this->ports()->addPort("JointStateGravity",port_JointStateGravity).doc(""); // ? TODO

	l(Info) << "constructed !" << endlog();

//	KDLParser p;
//	boost::shared_ptr<urdf::ModelInterface> m = p.parseURDFFile(
//			"/homes/dwigand/code/cogimon/lwr-robot-description/lwr-robot.urdf");
//	KDL::Tree t;
//	p.treeFromUrdfModel(m, t);
//	l(Warning) << "NrOfJoints: " << t.getNrOfJoints()
//			<< ", NrOfSegments: " << t.getNrOfSegments() << endlog();
//
//	SegmentMap::const_iterator it;
//	for (it = t.getSegments().begin(); it != t.getSegments().end(); it++) {
//		l(Error) << "it: " << it->first << endlog();
//	}

}

bool RTTGazeboLWRSimulation::configureHook() {
	if ((kdl_tree_.getNrOfJoints() < DEFAULT_NR_JOINTS_LWR)
			|| (kdl_chain_.getNrOfJoints() < DEFAULT_NR_JOINTS_LWR)) {
		l(Error)
				<< "kdl_tree and kdl_chain not initialized, because number of joints not matching -> skipping configuration."
				<< endlog();
		l(Error) << "Number of joint found: " << kdl_tree_.getNrOfJoints()
				<< endlog();
		return false;
	}

	// initialize solvers
	id_dyn_solver.reset(new ChainDynParam(kdl_chain_, gravity_vector));
	id_rne_solver.reset(new ChainIdSolver_RNE(kdl_chain_, gravity_vector));
	fk_vel_solver.reset(new ChainFkSolverVel_recursive(kdl_chain_));
	jnt_to_jac_solver.reset(new ChainJntToJacSolver(kdl_chain_));

	// initialize output-ports and storage variables
	port_JointVelocity.setDataSample(
			rci::JointVelocities::create(DEFAULT_NR_JOINTS_LWR, 0.0));
//	port_CartesianVelocity.setDataSample(rci::CartesianVelocity()); // TODO ??
	port_CartesianPosition.setDataSample(
			rci::Pose::fromMetersAndRadians(0, 0, 0, 1, 0, 0, 0));
	RTT::OutputPort<nemo::RealMatrix> port_MassMatrix; // TODO ??
	port_JointTorque.setDataSample(
			rci::JointTorques::create(DEFAULT_NR_JOINTS_LWR, 0.0));
	port_GravityTorque.setDataSample(
			rci::JointTorques::create(DEFAULT_NR_JOINTS_LWR, 0.0));
	port_JointPosition.setDataSample(
			rci::JointAngles::create(DEFAULT_NR_JOINTS_LWR, 0.0));

	// initialize joint data storages // TODO not needed!
	initJointStateFromKDLCHain(kdl_chain_, joint_state_);
	initJointStateFromKDLCHain(kdl_chain_, joint_state_cmd_);
	initJointStateFromKDLCHain(kdl_chain_, joint_state_gravity_);
	initJointStateFromKDLCHain(kdl_chain_, joint_state_dyn_);

	setInitialJointPosition(rci::JointAngles::create(7, 0.0));

	joint_pos_cmd_ = rci::JointAngles::create(DEFAULT_NR_JOINTS_LWR, 0.0);
	jnt_pos_cmd_.resize(joint_pos_cmd_->getDimension());

	joint_vel_cmd_ = rci::JointVelocities::create(DEFAULT_NR_JOINTS_LWR, 0.0);
	jnt_vel_cmd_.resize(joint_vel_cmd_->getDimension());

	joint_trq_cmd_ = rci::JointTorques::create(DEFAULT_NR_JOINTS_LWR, 0.0);
	jnt_trq_cmd_.resize(joint_trq_cmd_->getDimension());

	joint_torque_gravity = rci::JointTorques::create(DEFAULT_NR_JOINTS_LWR,
			0.0);

	joint_torque_gazebo = rci::JointTorques::create(DEFAULT_NR_JOINTS_LWR,
			0.0);
	joint_position_gazebo = rci::JointAngles::create(DEFAULT_NR_JOINTS_LWR, 0.0);
	joint_velocity_gazebo = rci::JointVelocities::create(DEFAULT_NR_JOINTS_LWR, 0.0);

	// TODO check default impedance params.
	impedance_limits_ = rci::JointImpedance::create(
			nemo::RealVector(nemo::dim(DEFAULT_NR_JOINTS_LWR * 2), 0.0));
	impedance_limits_->setValue(0, 450.0);
	impedance_limits_->setValue(2, 450.0);
	impedance_limits_->setValue(4, 200.0);
	impedance_limits_->setValue(6, 200.0);
	impedance_limits_->setValue(8, 200.0);
	impedance_limits_->setValue(10, 20.0);
	impedance_limits_->setValue(12, 10.0);

	impedance_limits_->setValue(1, 1.0);
	impedance_limits_->setValue(3, 1.0);
	impedance_limits_->setValue(5, 0.7);
	impedance_limits_->setValue(7, 0.7);
	impedance_limits_->setValue(9, 0.7);
	impedance_limits_->setValue(11, 0.1);
	impedance_limits_->setValue(13, 0.0);

	joint_imp_cmd_ = rci::JointImpedance::create(
			nemo::RealVector(nemo::dim(DEFAULT_NR_JOINTS_LWR * 2), 0.0));

	cart_pos_ = rci::Pose::fromMetersAndRadians(0, 0, 0, 1, 0, 0, 0);

	joint_torque_gazebo_cmd = rci::JointTorques::create(DEFAULT_NR_JOINTS_LWR,
			0.0);

	for (int i = 0; i < (int) CountEntries_RCM; i++)
		joint_torque_gazebo_cmd_tmp.push_back(
				rci::JointTorques::create(
						joint_torque_gazebo_cmd->getDimension(), 0.0));

	// initialize KDL fields
	jntArrVelConfig_q.resize(DEFAULT_NR_JOINTS_LWR);
	f_ext.resize(kdl_chain_.getNrOfSegments());
	G.resize(DEFAULT_NR_JOINTS_LWR);
	qdot.resize(DEFAULT_NR_JOINTS_LWR);
	qddot.resize(DEFAULT_NR_JOINTS_LWR);
	jnt_trq_kdl_.resize(DEFAULT_NR_JOINTS_LWR);

	// initialize safety
	initSafetyConstraints();

	// resize FAKE robot fields
	jac_.resize(DEFAULT_NR_JOINTS_LWR);
	jac_.data.setZero();
	mass_.resize(DEFAULT_NR_JOINTS_LWR, DEFAULT_NR_JOINTS_LWR);
	mass_.setZero();
	H.resize(DEFAULT_NR_JOINTS_LWR);
	jnt_pos_.resize(DEFAULT_NR_JOINTS_LWR);
	jnt_pos_.setZero();
	jnt_vel_.resize(DEFAULT_NR_JOINTS_LWR);
	jnt_vel_.setZero();
	jnt_trq_coriolis_kdl_.resize(DEFAULT_NR_JOINTS_LWR);
	jnt_trq_.resize(DEFAULT_NR_JOINTS_LWR);
	jnt_trq_.setZero();
	grav_trq_.resize(DEFAULT_NR_JOINTS_LWR);
	grav_trq_.setZero();

	// initialize controller storage fields
	jnt_trq_gazebo_cmd_.resize(DEFAULT_NR_JOINTS_LWR);
	jnt_trq_gazebo_cmd_.setZero();

	kp_.resize(DEFAULT_NR_JOINTS_LWR);
	kd_.resize(DEFAULT_NR_JOINTS_LWR);
	kg_.resize(DEFAULT_NR_JOINTS_LWR);
	kg_.setConstant(1.0);

	resetJointImpedanceGains();

	// initialize control modes for each joint
	for (int i = 0; i < DEFAULT_NR_JOINTS_LWR; i++)
		joint_control_modes_.push_back(FRI_CTRL_POSITION);

	// do some other important stuff here:

	l(Info) << "configured !" << endlog();
	return true;
}

void RTTGazeboLWRSimulation::setInitialJointPosition(
		rci::JointAnglesPtr j_init) {
	if (j_init->getDimension() != DEFAULT_NR_JOINTS_LWR) {
		l(Error)
				<< "Invalid size of JointAngles for initial joint position (should be "
				<< DEFAULT_NR_JOINTS_LWR << ")" << endlog();
		return;
	}

	for (int i = 0; i < j_init->getDimension(); i++) {
		joint_state_cmd_.positions[i] = j_init->rad(i);
	}

	l(Info) << "Setting joint Position to " << j_init->print()
			<< endlog();
	port_JointPositionGazeboCommand.write(j_init);
}

bool RTTGazeboLWRSimulation::setControlMode(int index, std::string mode) {
	// TODO perhaps a mutex needs to be used...
	if (index >= (int) joint_control_modes_.size()) {
		l(Error) << "index out of bounds: " << index
				<< "! Needs to be < than " << (int) joint_control_modes_.size()
				<< "." << endlog();
		return false;
	}

	if (mode == "FRI_CTRL_JNT_IMP") {
		joint_control_modes_[index] = FRI_CTRL_JNT_IMP;
	} else if (mode == "FRI_CTRL_POSITION") {
		joint_control_modes_[index] = FRI_CTRL_POSITION;
	} else if (mode == "FRI_CTRL_CART_IMP") {
		joint_control_modes_[index] = FRI_CTRL_CART_IMP;
	} else if (mode == "FRI_CTRL_DIRECT_TORQUE") {
		joint_control_modes_[index] = FRI_CTRL_DIRECT_TORQUE;
	} else {
		l(Error) << "ControlMode: " << mode << " not available."
				<< endlog();
		return false;
	}
	l(Info) << "Set ControlMode for joint " << index << " to " << mode
			<< endlog();
	return true;
}

void RTTGazeboLWRSimulation::initSafetyConstraints() {
	pos_limits_ = nemo::RealVector(nemo::dim(DEFAULT_NR_JOINTS_LWR), 0.0);
	pos_limits_[0] = 170 * TORAD;
	pos_limits_[1] = 120 * TORAD;
	pos_limits_[2] = 170 * TORAD;
	pos_limits_[3] = 120 * TORAD;
	pos_limits_[4] = 170 * TORAD;
	pos_limits_[5] = 120 * TORAD;
	pos_limits_[6] = 170 * TORAD;

	vel_limits_ = nemo::RealVector(nemo::dim(DEFAULT_NR_JOINTS_LWR), 0.0);
	vel_limits_[0] = 112.5 * TORAD;
	vel_limits_[1] = 112.5 * TORAD;
	vel_limits_[2] = 112.5 * TORAD;
	vel_limits_[3] = 112.5 * TORAD;
	vel_limits_[4] = 180 * TORAD;
	vel_limits_[5] = 112.5 * TORAD;
	vel_limits_[6] = 112.5 * TORAD;

	trq_limits_ = nemo::RealVector(nemo::dim(DEFAULT_NR_JOINTS_LWR), 0.0);
	trq_limits_[0] = 200;
	trq_limits_[1] = 200;
	trq_limits_[2] = 100;
	trq_limits_[3] = 100;
	trq_limits_[4] = 100;
	trq_limits_[5] = 30;
	trq_limits_[6] = 30;

	l(Info) << "pos_limits_: " << pos_limits_ << endlog();
	l(Info) << "vel_limits_: " << vel_limits_ << endlog();
	l(Info) << "trq_limits_: " << trq_limits_ << endlog();
}

void RTTGazeboLWRSimulation::initJointStateFromKDLCHain(const Chain& kdl_chain,
		JointState& joint_state) {
	// initialize joint data storages
	int countRealJoints = 0;
	for (unsigned int i = 0; i < kdl_chain.getNrOfSegments(); ++i) {
		if (kdl_chain.getSegment(i).getJoint().getType() != KDL::Joint::None) {
			countRealJoints++;
			const std::string name =
					kdl_chain.getSegment(i).getJoint().getName();
			joint_state.names.push_back(name);
		}
	}
	joint_state.positions.resize(countRealJoints);
	joint_state.velocities.resize(countRealJoints);
	joint_state.efforts.resize(countRealJoints);

	joint_state.positions.setZero();
	joint_state.velocities.setZero();
	joint_state.efforts.setZero();
}

bool RTTGazeboLWRSimulation::parseURDFforKDL(string urdfString) {
	urdfContainer = urdfString; // perhaps a mutex here TODO
	KDLParser p;

//	std::string xml_str;
//	if (!p.loadURDFFileIntoString(
//			"/homes/dwigand/code/cogimon/lwr-robot-description/lwr-robot.urdf",
//			xml_str)) {
//		l(Error) << "Could not load URDF from file" << endlog();
//		return false;
//	}

	p.initTreeAndChainFromURDFString(urdfString, chain_root_link_name,
			chain_tip_link_name, kdl_tree_, kdl_chain_);

	if (kdl_tree_.getNrOfJoints() > 0) {
		l(Info) << "URDF parsed !" << endlog();

		l(Info) << "NrOfJoints: " << kdl_tree_.getNrOfJoints()
				<< ", NrOfSegments: " << kdl_tree_.getNrOfSegments()
				<< endlog();

		SegmentMap::const_iterator it;
		for (it = kdl_tree_.getSegments().begin();
				it != kdl_tree_.getSegments().end(); it++) {
			l(Info) << "Parsed Joints in KDL-Tree " << it->first
					<< endlog();
		}

		return true;
	} else {
		l(Error) << "URDF could not be parsed !" << endlog();
		return false;
	}
}

bool RTTGazeboLWRSimulation::startHook() {
	l(Info) << "started !" << endlog();
	return true;

}

void RTTGazeboLWRSimulation::resetJointImpedanceGains() {
	joint_imp_cmd_ = impedance_limits_;
	for (int i = 0; i < joint_imp_cmd_->getDimension() / 2; i++) {
		kp_[i] = joint_imp_cmd_->asDouble(i * 2);
		kd_[i] = joint_imp_cmd_->asDouble(i * 2 + 1);
	}
}

void RTTGazeboLWRSimulation::updateHook() {
	// do some timing stuff for debugging
	static double last_update_time_sim;
	double rtt_time_ = 1E-9
			* os::TimeService::ticks2nsecs(
					os::TimeService::Instance()->getTicks());
	period_sim_ = rtt_time_ - last_update_time_sim;
	last_update_time_sim = rtt_time_;
	read_start = os::TimeService::Instance()->getNSecs();

//
/////* ####### read robot feedback from Gazebo ####### */
	if (port_JointPositionGazebo.connected()
			&& port_JointVelocityGazebo.connected()
			&& port_JointTorqueGazebo.connected()) {

		fs_p = port_JointPositionGazebo.read(joint_position_gazebo);
		fs_v = port_JointVelocityGazebo.read(joint_velocity_gazebo);
		fs_g = port_JointTorqueGazebo.read(joint_torque_gazebo);

		if (fs_g == NoData || fs_p == NoData || fs_v == NoData) {
			return;
		}
	} else {
		return;
	}

//
/////* ####### read incoming commands from controller-side ####### */
	jnt_pos_cmd_fs = port_JointPositionCommand.read(joint_pos_cmd_);
	jnt_vel_cmd_fs = port_JointVelocityCommand.read(joint_vel_cmd_); // TODO wird noch nicht benutzt.
	jnt_trq_cmd_fs = port_JointTorqueCommand.read(joint_trq_cmd_);

	//FlowStatus fs_fri_to_krl = port_ToKRL.read(fri_to_krl);
	FlowStatus fs_jnt_imp_cmd = port_JointImpedanceCommand.read(joint_imp_cmd_);
	if (fs_jnt_imp_cmd == RTT::NewData) {
		clampImpedance(joint_imp_cmd_, impedance_limits_);
	}

//
/////* ### Handle and process incoming commands */

	// convert data to VectorXd
	if (jnt_pos_cmd_fs == NewData) {
		for (int i = 0; i < joint_pos_cmd_->getDimension(); i++)
			jnt_pos_cmd_[i] = joint_pos_cmd_->rad(i);
	}

	// TODO do this with position and velocity also???????????
	if (jnt_trq_cmd_fs != NewData)
		joint_trq_cmd_.reset();

	// read some FRI stuff?? NEEDED??
	// TODO setting the CONTROL_MODE (in FRI)

	/* ### Do safety checks on the commands */
	//safetyChecks(jnt_pos_cmd_, jnt_vel_cmd_, jnt_trq_cmd_); // TODO handle properly
	read_duration = (os::TimeService::Instance()->getNSecs() - read_start);

//
/////* ####### calculate outgoing commands for Gazebo ####### */
	/* ### Convert Data to be used with KDL solvers */
	convertRealVectorToEigenVectorXd(joint_position_gazebo->asDoubleVector(),
			jnt_pos_);
	convertRealVectorToEigenVectorXd(joint_velocity_gazebo->deg_sVector(),
			jnt_vel_);
	convertRealVectorToEigenVectorXd(joint_torque_gazebo->NmVector(), jnt_trq_);

	/* ### initialize strange stuff for solvers */
	jntArrVelConfig_q.q.data = jnt_pos_;
	jntArrVelConfig_q.qdot.data = jnt_vel_;
	qdot.data = jnt_vel_; // what's this?
	qddot.data.setConstant(0.0); // what's this?

	/* ### execute solvers for inv.Dynamics */
	// calculate matrices H (inertia),C(coriolis) and G(gravitation)
	id_dyn_solver->JntToMass(jntArrVelConfig_q.q, H);
	mass_ = H.data; // resize this in configHook

//	// getting the current torques based on the assumption that f_ext is zero TODO
//	std::fill(f_ext.begin(), f_ext.end(), Wrench::Zero());
//	id_rne_solver->CartToJnt(jntArrVelConfig_q.q,qdot,qddot,f_ext,jnt_trq_kdl_);

	id_dyn_solver->JntToGravity(jntArrVelConfig_q.q, G);
	id_dyn_solver->JntToCoriolis(jntArrVelConfig_q.q, jntArrVelConfig_q.qdot,
			jnt_trq_coriolis_kdl_);

	joint_state_dyn_.positions = G.data;
	joint_state_dyn_.velocities = jnt_trq_coriolis_kdl_.data;
	joint_state_dyn_.efforts = jnt_trq_kdl_.data;

	grav_trq_ = G.data; // not needed! TODO
	for (int i = 0; i < joint_torque_gravity->getDimension(); i++) {
		joint_torque_gravity->setFromNm(i, G.data.data()[i]);
	}

	/* ### execute solvers for Jacobian and Forward-Kinematics based on velocities */
	jnt_to_jac_solver->JntToJac(jntArrVelConfig_q.q, jac_,
			kdl_chain_.getNrOfSegments());
	fk_vel_solver->JntToCart(jntArrVelConfig_q, ee_framevel_kdl_,
			kdl_chain_.getNrOfSegments());
	//ee_twist_kdl_ = ee_framevel_kdl_.GetTwist(); // dlw leave twist out
	//tf::twistKDLToMsg(ee_twist_kdl_, cart_twist_); // dlw leave twist out
	ee_frame_kdl_ = ee_framevel_kdl_.GetFrame();
	convertPoseKDL2PoseRCI(ee_frame_kdl_, cart_pos_);

	//controller execution
	chooseControllerBasedOnJointCtrlModes(joint_control_modes_);

/////* ####### send outgoing commands for Gazebo ####### */
	createGazeboTrqCommandAccordingToCtrlModes(joint_control_modes_,
			joint_torque_gazebo_cmd_tmp, joint_torque_gazebo_cmd);

	port_JointTorqueGazeboCommand.write(joint_torque_gazebo_cmd);

/////* ####### send outgoing feedback to controller-side ####### */

	port_JointPosition.write(joint_position_gazebo);
	port_JointVelocity.write(joint_velocity_gazebo);
	port_JointTorque.write(joint_torque_gazebo);
	port_GravityTorque.write(joint_torque_gravity);
	port_CartesianPosition.write(cart_pos_);

}

void RTTGazeboLWRSimulation::chooseControllerBasedOnJointCtrlModes(
		const std::vector<FRI_RobotCtrlMode>& modes_) {
	for (int fooInt = FRI_CTRL_JNT_IMP; fooInt != CountEntries_RCM; fooInt++) {
		FRI_RobotCtrlMode enumMode_it = static_cast<FRI_RobotCtrlMode>(fooInt);
		if (std::find(modes_.begin(), modes_.end(), enumMode_it)
				!= modes_.end()) {
			// call controller for this mode
			callControllerForCtrlMode(enumMode_it);
		}
	}
}

void RTTGazeboLWRSimulation::callControllerForCtrlMode(
		const FRI_RobotCtrlMode& mode_) {
	switch (mode_) {
	case FRI_CTRL_JNT_IMP:
		executeJointImpedanceController();
		break;
	case FRI_CTRL_POSITION:
		executeJointPositionController();
		break;
	case FRI_CTRL_CART_IMP:
		break;
	case FRI_CTRL_DIRECT_TORQUE:
		tunnelDirectTorque();
		break;
	default:
		break;
	}
}

void RTTGazeboLWRSimulation::executeJointPositionController() {
	jnt_trq_gazebo_cmd_ = kg_.asDiagonal() * G.data;

	if (jnt_pos_cmd_fs == NewData) {
		jnt_trq_gazebo_cmd_ += kp_.asDiagonal() * (jnt_pos_cmd_ - jnt_pos_)
				- kd_.asDiagonal() * jnt_vel_;
	}

	// convert back to rci
	for (int i = 0; i < joint_torque_gazebo_cmd->getDimension(); i++) {
		joint_torque_gazebo_cmd_tmp[FRI_CTRL_POSITION]->setFromNm(i,
				jnt_trq_gazebo_cmd_.data()[i]);
	}
}

void RTTGazeboLWRSimulation::executeJointImpedanceController() {
	// Joint Impedance Control
	jnt_trq_gazebo_cmd_ = kg_.asDiagonal() * G.data;

	// Joint Impedance part
	if (jnt_pos_cmd_fs == NewData) {
		jnt_trq_gazebo_cmd_ += kp_.asDiagonal() * (jnt_pos_cmd_ - jnt_pos_)
				- kd_.asDiagonal() * jnt_vel_;
	}
	// Additional torque
	if (jnt_trq_cmd_fs != NoData)
		jnt_trq_gazebo_cmd_ += jnt_trq_cmd_;

	// convert back to rci
	for (int i = 0; i < joint_torque_gazebo_cmd->getDimension(); i++) {
		joint_torque_gazebo_cmd_tmp[FRI_CTRL_JNT_IMP]->setFromNm(i,
				jnt_trq_gazebo_cmd_.data()[i]);
	}
}

void RTTGazeboLWRSimulation::createGazeboTrqCommandAccordingToCtrlModes(
		const std::vector<FRI_RobotCtrlMode>& modes_,
		const std::vector<rci::JointTorquesPtr>& pack,
		rci::JointTorquesPtr outTrqCmd) {

	for (int i = 0; i < outTrqCmd->getDimension(); i++) {
		outTrqCmd->setFromNm(i, pack[modes_[i]]->Nm(i));
	}

}

void RTTGazeboLWRSimulation::tunnelDirectTorque() {
	// not sending all the time, because force is going to be accumulated.
	if (jnt_trq_cmd_fs == NewData) {
		joint_torque_gazebo_cmd_tmp[FRI_CTRL_DIRECT_TORQUE] = joint_trq_cmd_;
	}
}

void RTTGazeboLWRSimulation::convertPoseKDL2PoseRCI(const KDL::Frame& frame,
		rci::PosePtr pose) {

	// TODO check if this is working properly
	pose->setValue(0, frame.p.data[0]);
	pose->setValue(1, frame.p.data[1]);
	pose->setValue(2, frame.p.data[2]);

	double qx, qy, qz, qw;
	frame.M.GetQuaternion(qx, qy, qz, qw);
	pose->setValue(3, qw);
	pose->setValue(4, qx);
	pose->setValue(5, qy);
	pose->setValue(6, qz);
}

void RTTGazeboLWRSimulation::convertRealVectorToEigenVectorXd(
		const nemo::RealVector& realV, Eigen::VectorXd& vXd) {
	for (int i = 0; i < realV.dimension().get(); i++) {
		vXd[i] = realV[i];
	}
}

void RTTGazeboLWRSimulation::stopHook() {
	l(Info) << "executes stopping !" << endlog();
}

void RTTGazeboLWRSimulation::cleanupHook() {
	l(Info) << "cleaning up !" << endlog();
}

bool RTTGazeboLWRSimulation::safetyChecks(rci::JointAnglesPtr position,
		rci::JointVelocitiesPtr velocity, rci::JointTorquesPtr torque) {
	return safetyCheck(position->asDoubleVector(), pos_limits_, "Position")
			&& safetyCheck(velocity->deg_sVector(), vel_limits_, "Velocity")
			&& safetyCheck(torque->NmVector(), trq_limits_, "Torque");
}

void RTTGazeboLWRSimulation::clampImpedance(rci::JointImpedancePtr imp,
		rci::JointImpedancePtr limits) {
	if (imp->getDimension() != limits->getDimension()) {
		l(Warning) << "Dimension of JointImpedance not fitting: "
				<< imp->getDimension() << endlog();
		return;
	}


	for (int i = 0; i < limits->getDimension(); i++) {
//		l(Error) << "Imp: " << imp->asDouble(i * 2) << ", " << imp->asDouble(i * 2 + 1) << endlog();
		kp_[i] = clamp(imp->asDouble(i * 2), 0.0, limits->asDouble(i * 2));
		kd_[i] = clamp(imp->asDouble(i * 2 + 1), 0.0,
				limits->asDouble(i * 2 + 1));

//		l(Error) << "Imp Clamped: " << kp_[i] << ", " << kd_[i] << endlog();
	}
}

bool RTTGazeboLWRSimulation::safetyCheck(const nemo::RealVector& v,
		const nemo::RealVector& limits, const std::string& name) {
	if (v.dimension().get() != DEFAULT_NR_JOINTS_LWR) {
		log(Error) << name << " vector size error " << v.dimension().get()
				<< "!=" << DEFAULT_NR_JOINTS_LWR << endlog();
		return false;
	}

	bool ret = true;
	for (unsigned i = 0; i < v.dimension().get(); i++) {
		if (std::abs(v[i]) > limits[i]) {
			log(Error) << name << " limit exceeded at J" << i << " : " << v[i]
					<< " / limit " << limits[i] << endlog();
			ret = false;
		}
	}
	return ret;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(RTTController)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(RTTGazeboLWRSimulation)
