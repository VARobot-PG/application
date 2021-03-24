/*
 * dobotSim.h
 *
 *  Created on: Oct 15, 2019
 *      Author: philipf
 */

#ifndef SRC_DOBOT_DOBOT_DESCRIPTION_INCLUDE_DOBOTSIM_H_
#define SRC_DOBOT_DOBOT_DESCRIPTION_INCLUDE_DOBOTSIM_H_

#include <functional>
#include <memory>
#include <queue>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <std_msgs/Bool.h>

#include <dobot_msgs/SetPTPWithLCmd.h>
#include <dobot_msgs/GetPose.h>
#include <dobot_msgs/GetPoseL.h>
#include "dobot_msgs/SetCmdTimeout.h"
#include "dobot_msgs/SetQueuedCmdClear.h"
#include "dobot_msgs/SetQueuedCmdStartExec.h"
#include "dobot_msgs/SetQueuedCmdForceStopExec.h"
#include "dobot_msgs/GetDeviceVersion.h"
#include "dobot_msgs/SetEndEffectorParams.h"
#include "dobot_msgs/SetPTPJointParams.h"
#include "dobot_msgs/SetPTPCoordinateParams.h"
#include "dobot_msgs/SetPTPJumpParams.h"
#include "dobot_msgs/SetPTPCommonParams.h"
#include "dobot_msgs/SetPTPCmd.h"
#include "dobot_msgs/SetPTPWithLCmd.h"
#include "dobot_msgs/GetPoseL.h"
#include "dobot_msgs/SetEndEffectorSuctionCup.h"

#include <marker_helper/markerHelper.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class DobotSim : public gazebo::ModelPlugin
{
	//A Path multiple waypoints in a row
	struct Command
	{
		tf2::Vector3 goal;
		double rotation;
		double railPosition;
		uint8_t mode;
	};

	//Used to store the arm angles
	struct ArmJointAngles
	{
		double baseRotation;
		double shoulderRotation;
		double elbowRotation;
		double toolRotation;

		ArmJointAngles(double baseRotation, double shoulderRotation, double elbowRotation, double toolRotation)
		{
			this->baseRotation = baseRotation;
			this->shoulderRotation = shoulderRotation;
			this->elbowRotation = elbowRotation;
			this->toolRotation = toolRotation;
		}
		ArmJointAngles()
		{
			baseRotation = shoulderRotation = elbowRotation = toolRotation = 0.0;
		}
		bool setFromTrajectoryPoint(trajectory_msgs::JointTrajectory::_points_type::value_type p)
		{
			if(p.positions.size() < 4) return false;
			baseRotation = p.positions.at(0);
			shoulderRotation = p.positions.at(1);
			elbowRotation = p.positions.at(2);
			toolRotation = p.positions.at(3);
			return true;
		}
	};

public:
	//Functions called by Gazebo:
	//Called when the simulation is loaded
	void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);
	//Called when the simulation is initialized
	void Init();
	//Called after the physik engine updated
	void OnUpdate();
	//Called it on destroy
	void FiniChild();

protected:
	/*
	 * Checks if a new command can be executed
	 * */
	void OnUpdateCommandQueue();

	/*
	 * Calculates the arm angles required to reach the goal position and puts them into a trajectory point
	 * @param deltaTime is the time it will need to move there
	 * Returns false if point can not be reached*/
	bool createTrajectoryPoint(tf2::Vector3 goal, double rotation, ArmJointAngles currentArmAngles, double linearRailStart, double linearRailGoal,
			trajectory_msgs::JointTrajectory::_points_type::value_type &point, ArmJointAngles &newArmAngles, double &deltaTime);

	/*
	 * Creates a trajectory point with the current arm angles, which can be used e.g. as a start for the next trajectory
	 * */
	void createStartTrajectoryPoint(trajectory_msgs::JointTrajectory::_points_type::value_type &point);

	/*
	 * Calculates the angles neccecary to reach the goal with the arm
	 */
	bool calcArmAngles(tf2::Vector3 goal, double rotation, ArmJointAngles &angles);

	/*
	 * Compares all angles and returnes the longest path
	 * */
	double calcPathLength(ArmJointAngles start, ArmJointAngles goal);

	/*
	 * Simulates the vacuum gripper. Checks for collisions and attaches the object to the arm.
	 * */
	void handleSuction();

	/*
	 * Generates waypoints based on the given command and executes them
	 * */
	bool executeCommand(tf2::Vector3 goal, double rotation, double railGoal, uint8_t mode);

	/*
	 * Adds a commad to the command queue
	 * */
	void addCommand(tf2::Vector3 goal, double rotation, double railGoal, uint8_t mode);

	/*
	 * Calculates the current state and publishes it
	 */
	void sendDobotState();

	//Ros callbacks will be executed outside the simulator thread
    ros::CallbackQueue queue;
    boost::thread callbackQueueThread;
    void QueueThread();

    /*
     * Callback to command the robot arm
     * */
    bool setPtpCmdCB(dobot_msgs::SetPTPCmd::Request &req, dobot_msgs::SetPTPCmd::Response &res);
    bool setPtpWithLCmdCB(dobot_msgs::SetPTPWithLCmd::Request &req, dobot_msgs::SetPTPWithLCmd::Response &res);
    /*
     * Callback to enable the vacuum gripper
     */
    bool setEndeffectorSuctionCB(dobot_msgs::SetEndEffectorSuctionCup::Request &req, dobot_msgs::SetEndEffectorSuctionCup::Response &res);

    /*
     * Called by the joint trajectory action, when the execution has finished
     * */
    void goalReachedCB(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr &result);

    /* Creates linear waypoints from the start to the gaol
     * */
    bool createLinearTrajectoryPath(tf2::Vector3 start, double startRotation, tf2::Vector3 goal, double goalRotation,
    		double linearRailStart, double linearRailGoal, double startTime, trajectory_msgs::JointTrajectory& msg, double& finishTime);

private:
	gazebo::physics::ModelPtr model;
	gazebo::physics::WorldPtr world;
	gazebo::physics::LinkPtr suctionLink;
	gazebo::physics::LinkPtr toolLink;
	gazebo::physics::LinkPtr handLink;
	gazebo::physics::LinkPtr baseLink;

	gazebo::physics::JointPtr baseJoint;
	gazebo::physics::JointPtr shoulderJoint;
	gazebo::physics::JointPtr elbowJoint;
	gazebo::physics::JointPtr handJoint;
	gazebo::physics::JointPtr toolJoint;
	gazebo::physics::JointPtr gripperJoint;
	gazebo::physics::JointPtr railJoint;

	gazebo::physics::ContactManager *contactManager;
	static const unsigned int SUCTION_CATEGORY = 0x10000000;

	std::shared_ptr<TrajectoryClient> trajectoryClient;
	std::string lastState = "";

	tf2_ros::TransformBroadcaster tfBroadcaster;

	std::string DobotName = "NO_NAME";
	double jumpHeight = 0.04;
	double maxSpeed = M_PI_2 * 0.5;
	double linearRailSpeed = 0.03;
	bool suctionEnabled = false;
	gazebo::physics::LinkPtr graspedObject;
	gazebo::physics::Inertial graspedObjectInertial;
	gazebo::physics::JointPtr fixedJoint;

	//Counts the recieved commands
	unsigned long commandQueueCounter = 0;
	//Counts the joint trajectory msgs
	unsigned long commandCounter = 0;
	bool hasLinearRail = false;

	tf2::Vector3 toolOffset = tf2::Vector3(0.069,0,-0.06);
	tf2::Transform armPose = tf2::Transform(tf2::Quaternion(0,0,0,1),tf2::Vector3(0,0,0));
	tf2::Vector3 prevPrintedArmPosition = tf2::Vector3(0,0,0);
	size_t trajCounter = 0;

	// Pointer to the update event connection
	gazebo::event::ConnectionPtr updateConnection;

	ros::Publisher dobotStatePub;
	ros::Publisher idlePub;
	ros::Publisher dobotStateLPub;
	ros::ServiceServer setPtpCmdServer;
	ros::ServiceServer setEndEffectorSuctionCupServer;
	ros::ServiceServer setPtpWithLCmdServer;
	std::shared_ptr<ros::NodeHandle> nh;
	std::shared_ptr<MarkerHelper> marker;

	std::queue<Command> commandQueue;
	bool alive;

	gazebo::common::Time lastUpdateTime;
	gazebo::common::Time lastPubTime;
	double effort = -5000.0;
	double maxEffort = 100.0;
};

#endif /* SRC_DOBOT_DOBOT_DESCRIPTION_INCLUDE_DOBOTSIM_H_ */
