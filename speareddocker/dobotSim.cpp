/*
 * dobotSim.cpp
 *
 *  Created on: Oct 15, 2019
 *      Author: philipf
 */

#include "../include/dobotSim.h"
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/physics.hh>
#include "geometry_msgs/Pose.h"

void DobotSim::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
	// Store the pointer to the model
	this->model = parent;
	this->world = parent->GetWorld();
	for(size_t i = 0; i < world->ModelCount(); i++)
	{
		if( world->ModelByIndex(i)->GetName().find("Truck") != std::string::npos)
		{
			tf2::Vector3 modelPosition = tf2::Vector3(parent->WorldPose().Pos().X(),parent->WorldPose().Pos().Y(),parent->WorldPose().Pos().Z());
			tf2::Vector3 otherPosition = tf2::Vector3(world->ModelByIndex(i)->WorldPose().Pos().X(),world->ModelByIndex(i)->WorldPose().Pos().Y(),world->ModelByIndex(i)->WorldPose().Pos().Z());
			double distance = modelPosition.distance(otherPosition);
		}
	}

	DobotName = sdf->Get<std::string>("robotNamespace");
	std::cout << "RobotName: " << DobotName << std::endl;

	gazebo::physics::LinkPtr suctionLink = model->GetLink(DobotName + "_gripper_link");
	if (!suctionLink) {
		ROS_ERROR_NAMED("DobotSim", "No link named '%s_gripper_link' has been found!",DobotName.c_str());
		return;
	}
	this->suctionLink = suctionLink;

	gazebo::physics::LinkPtr toolLink = model->GetLink(DobotName + "_tool_link");
	if (!toolLink) {
		ROS_ERROR_NAMED("DobotSim", "No link named '%s_tool_link' has been found!",DobotName.c_str());
		return;
	}
	this->toolLink = toolLink;

	gazebo::physics::LinkPtr baseLink = model->GetLink(DobotName + "_base_link");
	if (!baseLink) {
		ROS_ERROR_NAMED("DobotSim", "No link named '%s_base_link' has been found!",DobotName.c_str());
		return;
	}
	this->baseLink = baseLink;

	gazebo::physics::JointPtr railJoint = model->GetJoint(DobotName + "_rail_joint");
	if(!railJoint)
	{
		hasLinearRail = false;
	}else{
		hasLinearRail = true;
		std::cout << "DobotSim has found linear rail for " << DobotName << std::endl;
		this->railJoint = railJoint;
	}

	gazebo::physics::LinkPtr handLink = model->GetLink(DobotName + "_hand_link");
	if (!handLink) {
		ROS_ERROR_NAMED("DobotSim", "No link named '%s_hand_link' has been found!",DobotName.c_str());
		return;
	}
	this->handLink = handLink;

	gazebo::physics::JointPtr baseJoint = model->GetJoint(DobotName + "_base_joint");
	if (!baseJoint) {
		ROS_ERROR_NAMED("DobotSim", "No link named '%s_base_joint' has been found!",DobotName.c_str());
		return;
	}
	this->baseJoint = baseJoint;

	gazebo::physics::JointPtr shoulderJoint = model->GetJoint(DobotName + "_shoulder_joint");
	if (!shoulderJoint) {
		ROS_ERROR_NAMED("DobotSim", "No link named '%s_shoulder_joint' has been found!",DobotName.c_str());
		return;
	}
	this->shoulderJoint = shoulderJoint;

	gazebo::physics::JointPtr elbowJoint = model->GetJoint(DobotName + "_elbow_joint");
	if (!elbowJoint) {
		ROS_ERROR_NAMED("DobotSim", "No link named '%s_elbow_joint' has been found!",DobotName.c_str());
		return;
	}
	this->elbowJoint = elbowJoint;

	gazebo::physics::JointPtr handJoint = model->GetJoint(DobotName + "_hand_joint");
	if (!handJoint) {
		ROS_ERROR_NAMED("DobotSim", "No link named '%s_hand_joint' has been found!",DobotName.c_str());
		return;
	}
	this->handJoint = handJoint;

	gazebo::physics::JointPtr toolJoint = model->GetJoint(DobotName + "_tool_joint");
	if (!toolJoint) {
		ROS_ERROR_NAMED("DobotSim", "No link named '%s_tool_joint' has been found!",DobotName.c_str());
		return;
	}
	this->toolJoint = toolJoint;

	gazebo::physics::JointPtr gripperJoint = model->GetJoint(DobotName + "_gripper_joint");
	if (!gripperJoint) {
		ROS_ERROR_NAMED("DobotSim", "No link named '%s_gripper_joint' has been found!",DobotName.c_str());
		return;
	}
	this->gripperJoint = gripperJoint;

	// Listen to the update event. This event is broadcast every simulation iteration.
	this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
			std::bind(&DobotSim::OnUpdate, this));

	alive = true;

	if (!ros::isInitialized())
	{
	  ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
		<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
	  return;
	}

	nh = std::make_shared<ros::NodeHandle>();
	marker = std::make_shared<MarkerHelper>(nh,DobotName + "_path","goal");
	dobotStatePub = nh->advertise<geometry_msgs::Pose>(DobotName + "/arm/Pose", 2);
	idlePub = nh->advertise<std_msgs::Bool>(DobotName + "/idle", 2);
	suctionEnabledPub = nh->advertise<std_msgs::Bool>(DobotName + "/suctionStatus", 2);

	ros::AdvertiseServiceOptions setPtpCmdOptions = ros::AdvertiseServiceOptions::create<dobot_msgs::SetPTPCmd>(DobotName + "/SetPTPCmd",
					boost::bind(&DobotSim::setPtpCmdCB, this, _1, _2),ros::VoidPtr(), &this->queue);
	this->setPtpCmdServer = this->nh->advertiseService(setPtpCmdOptions);

	ros::AdvertiseServiceOptions setEndEffectorSuctionCupOptions = ros::AdvertiseServiceOptions::create<dobot_msgs::SetEndEffectorSuctionCup>(DobotName + "/SetEndEffectorSuctionCup",
					boost::bind(&DobotSim::setEndeffectorSuctionCB, this, _1, _2),ros::VoidPtr(), &this->queue);
	this->setEndEffectorSuctionCupServer = this->nh->advertiseService(setEndEffectorSuctionCupOptions);

	if(hasLinearRail)
	{
		ros::AdvertiseServiceOptions setPtpWithLCmdOptions = ros::AdvertiseServiceOptions::create<dobot_msgs::SetPTPWithLCmd>(DobotName + "/SetPTPWithLCmd",
					boost::bind(&DobotSim::setPtpWithLCmdCB, this, _1, _2),ros::VoidPtr(), &this->queue);
		this->setPtpWithLCmdServer = this->nh->advertiseService(setPtpWithLCmdOptions);
	}

	// start custom queue
	this->callbackQueueThread = boost::thread(boost::bind(&DobotSim::QueueThread, this));
	lastUpdateTime = this->world->SimTime();
	//nh->setParam("/effort",effort);
	//nh->setParam("/maxEffort",maxEffort);
	nh->setParam("linearRailSpeed",linearRailSpeed);
}

void DobotSim::Init()
{
	//Register for collisions, to be able to simulation the suctioncup
	this->contactManager = this->model->GetWorld()->Physics()->GetContactManager();
	this->contactManager->SetNeverDropContacts(true);

	for(auto &c : suctionLink->GetCollisions())
	{
		c->SetCategoryBits(SUCTION_CATEGORY);
		c->SetCollideBits(GZ_FIXED_COLLIDE);
	}

	gazebo::physics::PhysicsEnginePtr physics = model->GetWorld()->Physics();
	this->fixedJoint = physics->CreateJoint("revolute");
}

void DobotSim::OnUpdateCommandQueue()
{
	if(commandQueue.size() > 0 && trajectoryClient->getState().isDone()) //If there are commands left, execute them!
	{
		ROS_INFO("Executing command: [%f,%f,%f,%f] of %i",commandQueue.front().goal.x(),commandQueue.front().goal.y(),commandQueue.front().goal.z(),commandQueue.front().rotation, (int)commandQueue.size());
		executeCommand(commandQueue.front().goal,commandQueue.front().rotation,commandQueue.front().railPosition,commandQueue.front().mode);
		commandQueue.pop();
	}
}

void DobotSim::handleSuction()
{
	if(!suctionEnabled && graspedObject) //Should we detach something?
	{
		fixedJoint->Detach();
		graspedObject->SetCollideMode("all");
		*graspedObject->GetInertial() = graspedObjectInertial;
		graspedObject = nullptr;
		return;
	}

	if(graspedObject || !suctionEnabled) return;
	if(this->contactManager->GetContactCount() == 0) return;

	for(auto &contact : contactManager->GetContacts())//Is there a contact with a relevant object for grasping? If yes, attach!
	{
		if(!contact->collision1->GetLink()->GetEnabled() || !contact->collision2->GetLink()->GetEnabled()) continue;
		if(contact->collision1->GetSurface()->collideWithoutContact || contact->collision2->GetSurface()->collideWithoutContact) continue;
		if(contact->collision1->IsStatic() || contact->collision2->IsStatic()) continue;

		std::cout << "Attatching 1: " << contact->collision1->GetLink()->GetName();
		std::cout << " 2: " << contact->collision2->GetLink()->GetName() << "\n" << std::endl;

		graspedObject = contact->collision1->GetLink();
		graspedObject->SetCollideMode("none");
		gazebo::physics::InertialPtr inertial = graspedObject->GetInertial();
		graspedObjectInertial = *inertial;
		inertial->SetMass(0.0001);
		inertial->SetIXX(0.0001);
		inertial->SetIYY(0.0001);
		inertial->SetIZZ(0.0001);


		fixedJoint->Load(suctionLink,graspedObject,suctionLink->WorldPose());
		fixedJoint->Init();
		fixedJoint->SetDamping(0,1.0);
		fixedJoint->SetVelocityLimit(0,0.00000001);
		fixedJoint->SetEffortLimit(0,0.00000001);
		//fixedJoint->SetStiffness(0,999999);

		fixedJoint->SetUpperLimit(0,fixedJoint->Position(0));
		fixedJoint->SetLowerLimit(0,fixedJoint->Position(0));
		return;
	}
}

// Called by the world update start event
void DobotSim::OnUpdate()
{
	gazebo::common::Time currentTime = this->world->SimTime();
	double deltaTime = (this->world->SimTime() - lastUpdateTime).Double();
	lastUpdateTime = currentTime;

	if(trajectoryClient)//Only used for debugging, to print the current state of the joint trajectory controller
	{
		std::string currentState = trajectoryClient->getState().toString();
		if(currentState != lastState)
		{
			std::cout << currentState << std::endl;
			lastState = currentState;
		}
	}

	//The hand joint is set manually so that the joint is always behaving like the true dobot magician arm.
	//Otherwise there would be a chance that the arm is not perfectly straight and this does not look good.
	double handRotation = -(shoulderJoint->Position(0) + elbowJoint->Position(0));
	handJoint->SetPosition(0,handRotation);

	//Simulates the spring on the gripper
	double f = gripperJoint->Position(0)*effort;
	f = std::min(maxEffort,std::max(-maxEffort,f));
	gripperJoint->SetForce(0,f);

	if ((currentTime - lastPubTime).Double() > (1.0 / 50.0))
	{
		if(!trajectoryClient)//On initialization, create a new client. This is not possible to do in the Load function.
		{
			std::cout << "Creating new trajectory follower action client.\n";
			trajectoryClient = std::make_shared<TrajectoryClient>(*nh,DobotName + "/joint_trajectory_controller/follow_joint_trajectory",false);
		}
		//Calculate the arm endeffector pose and get rotation and location
		ignition::math::Pose3d armPoseI = (baseLink->WorldPose().Inverse() * handLink->WorldPose());
		tf2::Vector3 armPosition = tf2::Vector3(armPoseI.Pos().X(),armPoseI.Pos().Y(),armPoseI.Pos().Z());
		armPose.setOrigin(armPosition + toolOffset.x() * tf2::Vector3(armPosition.x(),armPosition.y(),0).normalize() + tf2::Vector3(0,0,toolOffset.z()));

		armPoseI = (baseLink->WorldPose().Inverse() * toolLink->WorldPose());
		tf2::Quaternion armRotation = tf2::Quaternion(armPoseI.Rot().X(),armPoseI.Rot().Y(),armPoseI.Rot().Z(),armPoseI.Rot().W());
		armPose.setRotation(armRotation);

		sendDobotState();
		OnUpdateCommandQueue();

		//Used for visualization in rviz, to inspect the robot path
		if(prevPrintedArmPosition.distance(armPose.getOrigin()) > 0.005)
		{
			prevPrintedArmPosition = armPose.getOrigin();
			marker->publishMarker(prevPrintedArmPosition,"Dobot_Loader/Dobot_Loader",trajCounter,125,125,125,"Traj",0);
			trajCounter++;
		}
		handleSuction();

		lastPubTime = currentTime;
		//nh->getParam("/effort",effort);
		//nh->getParam("/maxEffort",maxEffort);
		nh->getParam("linearRailSpeed",linearRailSpeed);
	}
}

// Finalize the controller
void DobotSim::FiniChild()
{
	alive = false;
	queue.clear();
	queue.disable();
	nh->shutdown();
	callbackQueueThread.join();
}

bool DobotSim::setPtpCmdCB(dobot_msgs::SetPTPCmd::Request &req, dobot_msgs::SetPTPCmd::Response &res)
{
	if(!req.isQueued)
	{
		while(commandQueue.size() > 0) commandQueue.pop();
	}
	addCommand(tf2::Vector3(req.x*0.001,req.y*0.001,req.z*0.001),req.r,std::numeric_limits<double>::quiet_NaN(),(uint8_t)req.ptpMode);
	res.result = true;
	res.queuedCmdIndex = commandQueueCounter;
	commandQueueCounter++;
	return true;
}

bool DobotSim::setPtpWithLCmdCB(dobot_msgs::SetPTPWithLCmd::Request &req, dobot_msgs::SetPTPWithLCmd::Response &res)
{
	if(!req.isQueued)
	{
		while(commandQueue.size() > 0) commandQueue.pop();
	}
	addCommand(tf2::Vector3(req.x*0.001,req.y*0.001,req.z*0.001),req.r,-double(req.l)/1000.0,(uint8_t)req.ptpMode);
	res.result = true;
	res.queuedCmdIndex = commandQueueCounter;
	commandQueueCounter++;
	return true;
}

bool DobotSim::setEndeffectorSuctionCB(dobot_msgs::SetEndEffectorSuctionCup::Request &req, dobot_msgs::SetEndEffectorSuctionCup::Response &res)
{
	suctionEnabled = (req.enableCtrl && req.suck);
	res.result = true;
	res.queuedCmdIndex = commandQueueCounter;
	commandQueueCounter++;
	return true;
}

void DobotSim::goalReachedCB(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr &result)
{
	std::cout << "State: " << state.getText() << std::endl;
	std::cout << "Error code: " << result->error_code << " error text: " << result->error_string << std::endl;
}

void DobotSim::addCommand(tf2::Vector3 goal, double rotation, double railGoal, uint8_t mode)
{
	Command c;
	c.mode = mode;
	c.goal = goal;
	c.rotation = rotation;
	c.railPosition = railGoal;
	commandQueue.push(c);
}

void DobotSim::sendDobotState()
{
	geometry_msgs::Pose poseMsg;
	poseMsg.position.x = armPose.getOrigin().x()*1000;
	poseMsg.position.y = armPose.getOrigin().y()*1000;
	poseMsg.position.z = armPose.getOrigin().z()*1000;
	poseMsg.orientation.x = armPose.getRotation().x();
	poseMsg.orientation.y = armPose.getRotation().y();
	poseMsg.orientation.z = armPose.getRotation().z();
	poseMsg.orientation.w = armPose.getRotation().w();
	dobotStatePub.publish(poseMsg);


	std_msgs::Bool isIdle;
	std_msgs::Bool isSuctionEnabled;
	isSuctionEnabled.data = suctionEnabled;
	isIdle.data = trajectoryClient->getState().isDone();
	idlePub.publish(isIdle);
	suctionEnabledPub.publish(isSuctionEnabled);

	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = DobotName + "/" + DobotName + "_elbow_link";
	transformStamped.child_frame_id = DobotName + "/" + DobotName + "_hand_link";

	transformStamped.transform.translation.x = 0.147532; //Set in the urdf model
	transformStamped.transform.translation.y = 0.0;
	transformStamped.transform.translation.z = 0.0;

	tf2::Matrix3x3 rotMat;
	rotMat.setRPY(0,handJoint->Position(0),0);
	tf2::Quaternion rot;
	rotMat.getRotation(rot);

	transformStamped.transform.rotation.x = rot.x();
	transformStamped.transform.rotation.y = rot.y();
	transformStamped.transform.rotation.z = rot.z();
	transformStamped.transform.rotation.w = rot.w();
	tfBroadcaster.sendTransform(transformStamped);

	geometry_msgs::TransformStamped transformStamped2;
	transformStamped2.header.stamp = ros::Time::now();
	transformStamped2.header.frame_id = "world";
	transformStamped2.child_frame_id = DobotName + "/world";
	transformStamped2.transform.translation.x = 0.0;
	transformStamped2.transform.translation.y = 0.0;
	transformStamped2.transform.translation.z = 0.0;
	transformStamped2.transform.rotation.x = 0.0;
	transformStamped2.transform.rotation.y = 0.0;
	transformStamped2.transform.rotation.z = 0.0;
	transformStamped2.transform.rotation.w = 1.0;
	tfBroadcaster.sendTransform(transformStamped2);

	transformStamped2.header.frame_id = DobotName + "/" + DobotName + "_tool_link";
	transformStamped2.child_frame_id = DobotName + "/" + DobotName + "_gripper_link";
	tfBroadcaster.sendTransform(transformStamped2);
}

void DobotSim::createStartTrajectoryPoint(trajectory_msgs::JointTrajectory::_points_type::value_type &point)
{
	point.positions.clear();
	point.positions.push_back(baseJoint->Position(0));
	point.positions.push_back(shoulderJoint->Position(0));
	point.positions.push_back(elbowJoint->Position(0));
	point.positions.push_back(toolJoint->Position(0) + baseJoint->Position(0));
	if(hasLinearRail) point.positions.push_back(railJoint->Position(0));

	point.velocities.clear();
	point.velocities.push_back(0.0);
	point.velocities.push_back(0.0);
	point.velocities.push_back(0.0);
	point.velocities.push_back(0.0);
	if(hasLinearRail) point.velocities.push_back(0.0);

	point.effort.clear();
	point.effort.push_back(0);
	point.effort.push_back(0);
	point.effort.push_back(0);
	point.effort.push_back(0);
	if(hasLinearRail) point.effort.push_back(0.0);

	point.accelerations.clear();
	point.accelerations.push_back(0);
	point.accelerations.push_back(0);
	point.accelerations.push_back(0);
	point.accelerations.push_back(0);
	if(hasLinearRail) point.accelerations.push_back(0.0);
}

bool DobotSim::calcArmAngles(tf2::Vector3 goal, double rotation, ArmJointAngles &angles)
{
	double baseRotation = std::atan2(goal.y(),goal.x()); //Calculate the goal base rotation by using the tan function.
	if(std::fabs(baseRotation) > (M_PI*130.0/180.0))
	{
		ROS_ERROR("Unable to reach [%f,%f,%f]! Robot can only turn 130° left/right!",goal.x(),goal.y(),goal.z());
		return false;
	}

	//Length of the shoulder and elbow
	double l1 = 0.135;
	double l2 = 0.147;
	double l = tf2::Vector3(goal.x(),goal.y(),0).length() - toolOffset.x();
	if(l < 0)//is the point to close to the robot?
	{
		ROS_ERROR("Unable to reach [%f,%f,%f]! Arm can not go so close to the robot!",goal.x(),goal.y(),goal.z());
	}
	//it is 8.2cm from shoulder link to the robot base.
	//The inverse kinematics formula approximates that the shoulder_link is the world coordinate frame, not the base_link, hence we have to fix that.
	//Simular the hand joint is the target point, not the vacuum gripper. Thats why we have to subtract the toolOffset.
	double h = -(goal.z() - toolOffset.z() - 0.082);

	//https://www.youtube.com/watch?v=IKOGwoJ2HLk
	double beta = -std::acos((std::pow(l,2.0) + std::pow(h,2.0) - std::pow(l1,2.0) - std::pow(l2,2.0))  /  (2*l1*l2));
	double alpha = std::atan2(h,l) + std::atan2((l2*std::sin(beta)),(l1 + l2*std::cos(beta)));

	if(beta != beta || alpha != alpha)
	{
		ROS_ERROR("Unable to reach [%f,%f,%f]!",goal.x(),goal.y(),goal.z());
		return false;
	}

	if(alpha < (-M_PI_2 - 0.15))
	{
		ROS_ERROR("Unable to reach [%f,%f,%f]! Point is too close to the robot base!",goal.x(),goal.y(),goal.z());
		return false;
	}

	//Alpha should be between 0 and -M_PI_2
	angles.baseRotation = baseRotation;
	angles.shoulderRotation = M_PI_2 + alpha;
	angles.elbowRotation = (-beta) - M_PI_2;
	angles.toolRotation = rotation - baseRotation;
	return true;
}

double DobotSim::calcPathLength(ArmJointAngles start, ArmJointAngles goal)
{
	double basePathLength = std::fabs(goal.baseRotation - start.baseRotation);
	double shoulderPathLength = std::fabs(goal.shoulderRotation - start.shoulderRotation);
	double elbowPathLength = std::fabs(goal.elbowRotation - start.elbowRotation);
	double toolPathLength = std::fabs(goal.toolRotation - start.toolRotation);
	return std::max(basePathLength,std::max(shoulderPathLength,std::max(toolPathLength,elbowPathLength)));
}

bool DobotSim::createTrajectoryPoint(tf2::Vector3 goal, double rotation, ArmJointAngles currentArmAngles, double linearRailStart, double linearRailGoal,
		trajectory_msgs::JointTrajectory::_points_type::value_type &point, ArmJointAngles &newArmAngles, double &deltaTime)
{
	ArmJointAngles angles;
	if(!calcArmAngles(goal,rotation,angles)) return false;

	point.positions.clear();
	point.positions.push_back(angles.baseRotation);
	point.positions.push_back(angles.shoulderRotation);
	point.positions.push_back(angles.elbowRotation);
	point.positions.push_back(angles.toolRotation);
	if(hasLinearRail) point.positions.push_back(linearRailGoal);

	deltaTime = calcPathLength(angles,currentArmAngles) / maxSpeed;
	if(hasLinearRail)
	{
		double railTime = std::fabs(linearRailGoal - linearRailStart) / linearRailSpeed;
		if(railTime > deltaTime)
		{
			deltaTime = railTime;
		}
	}

	point.velocities.clear();
	point.velocities.push_back(0);
	point.velocities.push_back(0);
	point.velocities.push_back(0);
	point.velocities.push_back(0);
	if(hasLinearRail) point.velocities.push_back(0);

	point.effort.clear();
	point.effort.push_back(0);
	point.effort.push_back(0);
	point.effort.push_back(0);
	point.effort.push_back(0);
	if(hasLinearRail) point.effort.push_back(0);

	point.accelerations.clear();
	point.accelerations.push_back(0);
	point.accelerations.push_back(0);
	point.accelerations.push_back(0);
	point.accelerations.push_back(0);
	if(hasLinearRail) point.accelerations.push_back(0);

	newArmAngles = angles;
	return true;
}

bool DobotSim::createLinearTrajectoryPath(tf2::Vector3 start, double startRotation, tf2::Vector3 goal, double goalRotation,
		double linearRailStart, double linearRailGoal, double startTime, trajectory_msgs::JointTrajectory& msg, double& finishTime)
{
	ArmJointAngles currentAngles;
	double currentRailPosition = linearRailStart;
	calcArmAngles(start,startRotation,currentAngles);
	ArmJointAngles nextAngles;
	double distance = start.distance(goal);
	bool success = true;
	size_t i = 1;
	double alpha = 0.0;
	double time = 0.0;
	while(alpha < 1.0)//Alpha is the iterator, which starts at 0 and ends at 1. Based on the speed, less waypoints are added. Alpha is used to lerp between start and end.
	{
		alpha = std::min(1.0,(i*maxSpeed*0.0025)/distance);
		//Calculate the next waypoint
		tf2::Vector3 newPosition = tf2::lerp(start,goal,alpha);
		double newRotation = (startRotation * (1.0 - alpha)) + (goalRotation * alpha);	//Lerp(alpha,startRotation,EndRotation)
		double newRailPosition = (linearRailStart * (1.0 - alpha)) + (linearRailGoal * alpha); //Lerp(alpha,linearRailStart,linearRailEnd)
		double deltaTime;
		trajectory_msgs::JointTrajectory::_points_type::value_type point;
		success = createTrajectoryPoint(newPosition,newRotation,currentAngles,currentRailPosition,newRailPosition,point,nextAngles,deltaTime);
		if(success)
		{
			time += deltaTime;
			point.time_from_start = ros::Duration(startTime) + ros::Duration(time);
			msg.points.push_back(point);
			currentAngles = nextAngles;
			currentRailPosition = newRailPosition;

			if(alpha < 1.0) marker->publishMarker(newPosition + tf2::Vector3(0,-newRailPosition,0),"Dobot_Loader/Dobot_Loader",i,0,0,255,"Plan",0);
			else marker->publishMarker(newPosition  + tf2::Vector3(0,-newRailPosition,0),"Dobot_Loader/Dobot_Loader",i,255,255,255,"Plan",0);
		}else{
			marker->publishMarker(newPosition  + tf2::Vector3(0,-newRailPosition,0),"Dobot_Loader/Dobot_Loader",i,255,0,0,"Plan",0);
		}
		i++;
	}
	finishTime = startTime + time;
	return success;
}

bool DobotSim::executeCommand(tf2::Vector3 goal, double rotation, double railGoal, uint8_t mode)
{
	marker->clearAll("Plan");
	marker->clearAll("Traj");
	trajCounter = 0;
	tf2::Vector3 currentArmPosition = armPose.getOrigin();
	rotation = M_PI * rotation / 180.0; //Commands have rotation in degrees, but we need radiants

	trajectory_msgs::JointTrajectory msg;
	msg.header.seq = commandCounter;
	commandCounter++;
	//These are the joints we can command
	msg.joint_names.push_back(DobotName + "_base_joint");
	msg.joint_names.push_back(DobotName + "_shoulder_joint");
	msg.joint_names.push_back(DobotName + "_elbow_joint");
	msg.joint_names.push_back(DobotName + "_tool_joint");
	if(hasLinearRail) msg.joint_names.push_back(DobotName + "_rail_joint");

	msg.points.resize(1);
	//The trajectory starts at the current arm position
	createStartTrajectoryPoint(msg.points.at(0));
	nh->param("/speed",maxSpeed,maxSpeed);
	msg.points.at(0).time_from_start = ros::Duration(0.02);

	double currentRotation = toolJoint->Position(0) + baseJoint->Position(0);
	double currentRailPosition = 0.0;
	if(hasLinearRail)
	{
		currentRailPosition = railJoint->Position(0);
		if(railGoal != railGoal) railGoal = currentRailPosition;
	}else{
		if(railGoal != railGoal) railGoal = 0.0;
	}

	bool success = true;
	if(mode == 0)//Jump mode
	{
		nh->param("/jumpHeight",jumpHeight,jumpHeight);
		double targetZ = std::max(currentArmPosition.z(),goal.z()) + jumpHeight;

		tf2::Vector3 targetA = tf2::Vector3(currentArmPosition.x(),currentArmPosition.y(),targetZ);
		double finishTime = 0.0;
		success &= createLinearTrajectoryPath(currentArmPosition,currentRotation,targetA,currentRotation,currentRailPosition,currentRailPosition,msg.points.at(0).time_from_start.toSec()+0.1,msg,finishTime);
		std::cout << "currentRotation: " << 180.0/M_PI*currentRotation << std::endl;
		std::cout << "rotation: " << 180.0/M_PI*rotation << std::endl;
		std::cout << "ToolJointName: " << toolJoint->GetName() << std::endl;

		trajectory_msgs::JointTrajectory::_points_type::value_type point = msg.points.at(msg.points.size()-1);
		point.time_from_start = ros::Duration(finishTime) + ros::Duration(1.0);
		msg.points.push_back(point);

		tf2::Vector3 targetB = tf2::Vector3(goal.x(),goal.y(),targetZ);
		double deltaTime = 0.0;
		ArmJointAngles newAngles;
		ArmJointAngles prevAngles;
		prevAngles.setFromTrajectoryPoint(msg.points.at(msg.points.size()-1));
		trajectory_msgs::JointTrajectory::_points_type::value_type point2;
		success &= createTrajectoryPoint(targetB,rotation,prevAngles,currentRailPosition,railGoal,point2,newAngles,deltaTime);
		point2.time_from_start = msg.points.at(msg.points.size()-1).time_from_start + ros::Duration(deltaTime);
		msg.points.push_back(point2);

		trajectory_msgs::JointTrajectory::_points_type::value_type point3 = point2;
		point3.time_from_start = point2.time_from_start + ros::Duration(1.0);
		msg.points.push_back(point3);

		finishTime = 0.0;
		success &= createLinearTrajectoryPath(targetB,rotation,goal,rotation,railGoal,railGoal,point3.time_from_start.toSec()+0.1,msg,finishTime);
	}else if(mode == 1)//joint mode (arc mode)
	{
		msg.points.resize(2);
		double deltaTime = 0.0;
		ArmJointAngles prevAngles = ArmJointAngles(baseJoint->Position(0),shoulderJoint->Position(0),elbowJoint->Position(0),currentRotation);
		success &= createTrajectoryPoint(goal,rotation,prevAngles,currentRailPosition,railGoal,msg.points.at(1),prevAngles,deltaTime);
		msg.points.at(1).time_from_start = msg.points.at(0).time_from_start + ros::Duration(deltaTime);
	}else if(mode == 2)//linear mode
	{
		double finishTime = 0.0;
		success &= createLinearTrajectoryPath(currentArmPosition,currentRotation,goal,rotation,currentRailPosition,railGoal,msg.points.at(0).time_from_start.toSec(),msg,finishTime);
	}else
	{
		return false;
	}
	if(success)
	{
		//Set tolerances to "dont care". Otherwise the execution looks bad, if the tolerances are violated.
		control_msgs::FollowJointTrajectoryGoal actionMsg;
		actionMsg.trajectory = msg;
		actionMsg.goal_tolerance.clear();
		actionMsg.path_tolerance.clear();
		control_msgs::JointTolerance t;
		t.acceleration = 0;
		t.position = 0;
		t.velocity = 0;
		t.name = DobotName + "_base_joint";
		actionMsg.goal_tolerance.push_back(t);
		actionMsg.path_tolerance.push_back(t);
		t.name = DobotName + "_shoulder_joint";
		actionMsg.goal_tolerance.push_back(t);
		actionMsg.path_tolerance.push_back(t);
		t.name = DobotName + "_elbow_joint";
		actionMsg.goal_tolerance.push_back(t);
		actionMsg.path_tolerance.push_back(t);
		t.name = DobotName + "_tool_joint";
		actionMsg.goal_tolerance.push_back(t);
		actionMsg.path_tolerance.push_back(t);
		if(hasLinearRail)
		{
			t.name = DobotName + "_rail_joint";
			actionMsg.goal_tolerance.push_back(t);
			actionMsg.path_tolerance.push_back(t);
		}
		actionMsg.goal_time_tolerance = ros::Duration(5.0);

		actionMsg.trajectory.header.stamp = ros::Time::now();
		std::cout << "Sending goal:\n";
		trajectoryClient->sendGoal(actionMsg,boost::bind(&DobotSim::goalReachedCB, this, _1, _2),TrajectoryClient::SimpleActiveCallback(),TrajectoryClient::SimpleFeedbackCallback());
	}
	return true;
}

void DobotSim::QueueThread()
{
	static const double timeout = 0.01;

	while (alive && nh->ok()) {
		queue.callAvailable(ros::WallDuration(timeout));
	}
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DobotSim)
