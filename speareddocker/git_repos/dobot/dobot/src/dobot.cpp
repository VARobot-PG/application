//
// Created by lars on 12.03.19.
//
#include "../include/dobot/dobot.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace dobot
{
    Dobot::Dobot(std::string dobot_name)
    {
        ROBOT_NAME = dobot_name;
    }
    
    Dobot::Dobot(const Dobot& d)
    {
        this->ROBOT_NAME = d.ROBOT_NAME;
        if(d.nh)
            initArm(d.nh, d.ROBOT_NAME);
    }

	void Dobot::abortMovement()
	{
		ROS_WARN("DobotFunctions::abortMovement is not implemented.");
	}

	void Dobot::robotStateCallback(const std_msgs::Bool::ConstPtr &msg)
	{
		robotIsIdle = msg->data;
	}

	void Dobot::enableSuctionCup(bool suck)
	{
		dobot_msgs::SetEndEffectorSuctionCupRequest req;
		dobot_msgs::SetEndEffectorSuctionCupRequest res;
		req.enableCtrl = true;
		req.suck = suck;
		req.isQueued = false;
		end_effector_client.call(req, res);
	}

	bool Dobot::positionToPositionWithL(const tf2::Vector3 goal, tf2::Vector3 &out, double &lOut)
	{
		out = goal;
		if (goal.y() > 0.0) {
			lOut = 0.0;
			out.setY(goal.y());
		} else if (goal.y() < -1000.0) {
			lOut = 1000.0;
			out.setY(goal.y() + 1000.0);
		} else {
			lOut = -goal.y();
			out.setY(0.0);
		}
		return true;
	}

    bool Dobot::isAtPosition(tf2::Vector3 target)
    {
        std::cout << "Current arm pose: [" << armPosition.x() << "," << armPosition.y() << "," << armPosition.z() << "]"
                  << std::endl;
        double distance = target.distance(armPosition);
        std::cout << "Current distance: " << distance << std::endl;

        return distance < ALLOWED_ERROR;
    }

	bool Dobot::isAtPosition(tf2::Vector3 target, double r)
	{

		std::cout << "Current arm pose: [" << armPosition.x() << "," << armPosition.y() << "," << armPosition.z() << "]"
				  << std::endl;
		double distance = target.distance(armPosition);
		std::cout << "Current distance: " << distance << std::endl;


        dobot_msgs::GetPose srv;
        double rotation = 0;
        if(robotRotation.call(srv)){
            rotation = srv.response.r;
        }
       // std::cout << "rotation: " << rotation << std::endl;
        double rot_error = rotation - r;
        double rot_error_abs = abs(rot_error);
       // std:: cout << "rot_err: " << rot_error_abs << std::endl;

        distance+=rot_error_abs;

		return distance < ALLOWED_ERROR;
	}

	bool Dobot::isIdle()
	{
		return robotIsIdle;
	}

	void Dobot::moveArmToPosition(tf2::Vector3 pos, double r, size_t ptpMode)
	{
    	dobot_msgs::SetPTPCmd::Request req;
    	dobot_msgs::SetPTPCmd::Response res;

    	req.ptpMode = ptpMode;
    	req.x = pos.x();
    	req.y = pos.y();
    	req.z = pos.z();
    	req.r = r;
    	ptp_cmd_client.call(req, res);

		std::cout << "Moving arm to: [" << pos.x() << "," << pos.y() << "," << pos.z() << "," << r << "]" << std::endl;
		std::cout << "QueuedCmdIndex: " << res.queuedCmdIndex << " Result: " << res.result << std::endl;
	}


    void Dobot::moveArmToPositionBlocking(tf2::Vector3 pos, double r, size_t ptpMode) {

        while (!isAtPosition(pos,r) && ros::ok()) {
            if (isIdle()) {
                moveArmToPosition(pos, r, ptpMode);
            }
            ros::Duration(1).sleep();
            ros::spinOnce();
        }
    }

	void Dobot::moveArmToPositionWithL(tf2::Vector3 pos, double r, double l, size_t ptpMode)
	{
		dobot_msgs::SetPTPWithLCmd::Request req;
		dobot_msgs::SetPTPWithLCmd::Response res;
		req.ptpMode = ptpMode;
		req.x = pos.x();
		req.y = pos.y();
		req.z = pos.z();
		req.r = r;
		req.l = l;
		ptpL_cmd_client.call(req, res);
		std::cout << "Moving arm to: [" << pos.x() << "," << pos.y() << "," << pos.z() << "," << r << ", " << l << "]" << std::endl;
		std::cout << "QueuedCmdIndex: " << res.queuedCmdIndex << " Result: " << res.result << std::endl;
	}

    void Dobot::moveToPositionWithLBlocking(const tf2::Vector3 position, double r, double l) {

        while(!isAtPosition(position,r) && ros::ok()){
            if(isIdle()){
                moveArmToPositionWithL(position, r, l);
            }
            ros::Duration(1).sleep();
            ros::spinOnce();
        }
    }

    void Dobot::moveToPositionWithSafetyMarginZ(tf2::Vector3 pos, double r, double l, double safety_margin){

        pos.setZ(pos.getZ()+(safety_margin/2));
        moveToPositionWithLBlocking(pos, r, l);
        moveDownUntilToolCollision(r,safety_margin);
    }

    void Dobot::moveDownUntilToolCollision(double r, double safety_margin) {
        int step_size = 2;

        int steps = safety_margin / step_size;

        if(ROBOT_NAME != dobot::dobot_names::DOBOT_RAIL){
            return;
        }
        while(armPosition.x() == 0 && armPosition.y() == 0 && armPosition.z() == 0)
        {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        for(int step = 0; step < steps && !toolCollision; step++){
            dobot_msgs::SetPTPCmd::Request req;
            dobot_msgs::SetPTPCmd::Response res;
            req.ptpMode = 2;
            req.x = armPosition.x();
            req.y = armPosition.y();
            req.r = r;
            req.z = armPosition.z();
            req.z -= 2;
            ptp_cmd_client.call(req, res);
            tf2::Vector3 target(req.x,req.y,req.z);
            while(!isAtPosition(target,r) && ros::ok()){
                ros::spinOnce();
            }
        }
    }

    bool Dobot::hasLinearRail()
    {
    	return linearRail;
    }

	bool Dobot::isInitialized()
	{
		return initSuccess;
	}

	bool Dobot::convertToDobotPoint(const tf2::Vector3 graspPoint, tf2::Vector3& transformedGraspPoint, const std::string inputFrame)
	{
		if(hasLinearRail()) throw "Dobot::convertToDobotPoint not allowed on dobot with rail! Use Dobot::convertToDobotPointWithL instead!";

		tf2::Stamped<tf2::Transform> transform;
		geometry_msgs::TransformStamped tS;
		try{
			geometry_msgs::TransformStamped tS;
			tS = tfBuffer.lookupTransform(inputFrame,ROBOT_NAME,ros::Time(0));
			tf2::fromMsg(tS,transform);
		}catch(tf2::TransformException &ex)
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			return false;
		}

		transformedGraspPoint = (transform.inverse() * tf2::Transform(tf2::Quaternion(0,0,0,1),graspPoint)).getOrigin();
		transformedGraspPoint *= 1000.0;
		tf2::Vector3 graspNormal2D = tf2::Vector3(transformedGraspPoint.x(),transformedGraspPoint.y(),0.0);
		graspNormal2D = graspNormal2D.normalize();
		graspNormal2D = graspNormal2D*dobot::env::ARM_EXTENSION;
		//Using the normal vector pointing from dobot base to end effector to add 7cm tool offset.
		transformedGraspPoint = transformedGraspPoint - graspNormal2D;
		return true;
	}

	bool Dobot::convertToDobotPointWithL(const tf2::Vector3 graspPoint, tf2::Vector3& transformedGraspPoint, double &railPos, const std::string inputFrame)
	{
		if(!hasLinearRail()) throw "Dobot::convertToDobotPointWithL not allowed on dobot without rail! Use Dobot::convertToDobotPoint instead!";

		tf2::Stamped<tf2::Transform> transform;
		geometry_msgs::TransformStamped tS;
		try{
			geometry_msgs::TransformStamped tS;

			tS = tfBuffer.lookupTransform(inputFrame,ROBOT_NAME + "_rail",ros::Time(0));
			tf2::fromMsg(tS,transform);
		}catch(tf2::TransformException &ex)
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			return false;
		}

		transformedGraspPoint = (transform.inverse() * tf2::Transform(tf2::Quaternion(0,0,0,1),graspPoint)).getOrigin();
		transformedGraspPoint *= 1000.0;

		if(transformedGraspPoint.y() > 0)
		{
			railPos = 0.0;
		}else if(transformedGraspPoint.y() < -1000.0)
		{
			railPos = 1000.0;
			transformedGraspPoint.setY(transformedGraspPoint.y() + 1000.0);
		}else{
			railPos = -transformedGraspPoint.y();
			transformedGraspPoint.setY(0.0);
		}

		tf2::Vector3 graspNormal2D = tf2::Vector3(transformedGraspPoint.x(),transformedGraspPoint.y(),0.0);
		graspNormal2D = graspNormal2D.normalize();
		graspNormal2D = graspNormal2D*dobot::env::ARM_EXTENSION;
		//Using the normal vector pointing from dobot base to end effector to add 7cm tool offset.
		transformedGraspPoint = transformedGraspPoint - graspNormal2D;
		return true;
	}

	bool Dobot::convertToDobotPose(tf2::Transform graspPose, tf2::Transform& transformedGraspPose, const std::string inputFrame)
	{
		if(hasLinearRail()) throw "Dobot::convertToDobotPose not allowed on dobot with rail! Use Dobot::convertToDobotPoseWithL instead!";

		tf2::Stamped<tf2::Transform> transform;
		geometry_msgs::TransformStamped tS;
		try{
			geometry_msgs::TransformStamped tS;
			tS = tfBuffer.lookupTransform(inputFrame,ROBOT_NAME,ros::Time(0));

			tf2::fromMsg(tS,transform);
		}catch(tf2::TransformException &ex)
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			return false;
		}

		transformedGraspPose = transform.inverse() * graspPose;
		transformedGraspPose.setOrigin(transformedGraspPose.getOrigin() * 1000.0);

		tf2::Vector3 graspNormal2D = tf2::Vector3(transformedGraspPose.getOrigin().x(),transformedGraspPose.getOrigin().y(),0.0);
		graspNormal2D = graspNormal2D.normalize();
		graspNormal2D = graspNormal2D*dobot::env::ARM_EXTENSION;
		//Using the normal vector pointing from dobot base to end effector to add 7cm tool offset.
		transformedGraspPose.setOrigin(transformedGraspPose.getOrigin() - graspNormal2D);
		return true;
	}

	bool Dobot::convertToDobotPoseWithL(const tf2::Transform graspPose, tf2::Transform& transformedGraspPose, double &railPos, const std::string inputFrame)
	{
		if(!hasLinearRail()) throw "Dobot::convertToDobotPoseWithL not allowed on dobot with rail! Use Dobot::convertToDobotPose instead!";

		tf2::Stamped<tf2::Transform> transform;
		geometry_msgs::TransformStamped tS;
		try{
			geometry_msgs::TransformStamped tS;
			tS = tfBuffer.lookupTransform(inputFrame,ROBOT_NAME,ros::Time(0));

			tf2::fromMsg(tS,transform);
		}catch(tf2::TransformException &ex)
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			return false;
		}

		transformedGraspPose = transform.inverse() * graspPose;
		transformedGraspPose.setOrigin(transformedGraspPose.getOrigin() * 1000.0);

		if(transformedGraspPose.getOrigin().y() > 0)
		{
			railPos = 0.0;
		}else if(transformedGraspPose.getOrigin().y() < -1000.0)
		{
			railPos = -1000.0;
			transformedGraspPose.setOrigin(transformedGraspPose.getOrigin() + tf2::Vector3(0,1000.0,0));
		}else{
			railPos = transformedGraspPose.getOrigin().y();
			transformedGraspPose.setOrigin(transformedGraspPose.getOrigin() * tf2::Vector3(1.0,0.0,1.0));
		}

		tf2::Vector3 graspNormal2D = tf2::Vector3(transformedGraspPose.getOrigin().x(),transformedGraspPose.getOrigin().y(),0.0);
		graspNormal2D = graspNormal2D.normalize();
		graspNormal2D = graspNormal2D*dobot::env::ARM_EXTENSION;
		//Using the normal vector pointing from dobot base to end effector to add 7cm tool offset.
		transformedGraspPose.setOrigin(transformedGraspPose.getOrigin() - graspNormal2D);
		return true;
	}

	bool Dobot::initArm(std::shared_ptr<ros::NodeHandle> nh_ptr, std::string robotName, double jumpHeightInMM) {
		nh = nh_ptr;
		tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
		ROBOT_NAME = robotName;
		ros::ServiceClient client;
		// SetCmdTimeout
		client = nh->serviceClient<dobot_msgs::SetCmdTimeout>("/" + ROBOT_NAME + "/SetCmdTimeout");
		dobot_msgs::SetCmdTimeout srv1;
		srv1.request.timeout = 3000;
		if (client.call(srv1) == false) {
			ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
			return false;
		}

		client = nh->serviceClient<dobot_msgs::GetPoseL>("/" + ROBOT_NAME + "/GetPoseL");
		linearRail = client.exists();
		if(linearRail)
		{
			ROS_INFO("Found linear rail.");
		}else{
			ROS_INFO("Found no linear rail.");
		}

		// Clear the command queue
		client = nh->serviceClient<dobot_msgs::SetQueuedCmdClear>("/" + ROBOT_NAME + "/SetQueuedCmdClear");
		dobot_msgs::SetQueuedCmdClear srv2;
		client.call(srv2);

		// Start running the command queue
		client = nh->serviceClient<dobot_msgs::SetQueuedCmdStartExec>("/" + ROBOT_NAME + "/SetQueuedCmdStartExec");
		dobot_msgs::SetQueuedCmdStartExec srv3;
		client.call(srv3);

		// Set end effector parameters
		client = nh->serviceClient<dobot_msgs::SetEndEffectorParams>("/" + ROBOT_NAME + "/SetEndEffectorParams");
		dobot_msgs::SetEndEffectorParams srv5;
		srv5.request.xBias = 0;
		srv5.request.yBias = 0;
		srv5.request.zBias = 0;
		client.call(srv5);

		// Set PTP joint parameters
		client = nh->serviceClient<dobot_msgs::SetPTPJointParams>("/" + ROBOT_NAME + "/SetPTPJointParams");
		dobot_msgs::SetPTPJointParams jointParamSrv;

		for (int i = 0; i < 4; i++) {
			jointParamSrv.request.velocity.push_back(100);
		}
		for (int i = 0; i < 4; i++) {
			jointParamSrv.request.acceleration.push_back(100);
		}
		client.call(jointParamSrv);

		// Set PTP coordinate parameters
		client = nh->serviceClient<dobot_msgs::SetPTPCoordinateParams>("/" + ROBOT_NAME + "/SetPTPCoordinateParams");
		dobot_msgs::SetPTPCoordinateParams ptpCoordinateParamSrv;

		ptpCoordinateParamSrv.request.xyzVelocity = 100;
		ptpCoordinateParamSrv.request.xyzAcceleration = 100;
		ptpCoordinateParamSrv.request.rVelocity = 100;
		ptpCoordinateParamSrv.request.rAcceleration = 100;
		client.call(ptpCoordinateParamSrv);

		// Set PTP jump parameters
		client = nh->serviceClient<dobot_msgs::SetPTPJumpParams>("/" + ROBOT_NAME + "/SetPTPJumpParams");
		dobot_msgs::SetPTPJumpParams ptpJumpParamService;

		ptpJumpParamService.request.jumpHeight = jumpHeightInMM;
		ptpJumpParamService.request.zLimit = 200;
		client.call(ptpJumpParamService);

		// Set PTP common parameters
		client = nh->serviceClient<dobot_msgs::SetPTPCommonParams>("/" + ROBOT_NAME + "/SetPTPCommonParams");
		dobot_msgs::SetPTPCommonParams ptpCommonParamService;

		ptpCommonParamService.request.velocityRatio = 50;
		ptpCommonParamService.request.accelerationRatio = 50;
		client.call(ptpCommonParamService);

		robotStateSub = nh->subscribe<std_msgs::Bool>(ROBOT_NAME + "/idle", 1, &Dobot::robotStateCallback, this);
		robotArmPoseSub = nh->subscribe<geometry_msgs::Pose>(ROBOT_NAME + "/arm/Pose", 1, &Dobot::robotPoseCallback, this);
		ptp_cmd_client = nh->serviceClient<dobot_msgs::SetPTPCmd>(ROBOT_NAME + "/SetPTPCmd");
		ptpL_cmd_client = nh->serviceClient<dobot_msgs::SetPTPWithLCmd>(ROBOT_NAME + "/SetPTPWithLCmd");
		end_effector_client = nh->serviceClient<dobot_msgs::SetEndEffectorSuctionCup>(
				ROBOT_NAME + "/SetEndEffectorSuctionCup");
		toolCollisionSub = nh->subscribe<std_msgs::Bool>("arduino_rfid/DobotToolCollision", 1, &Dobot::toolCollisionCallback, this);
		robotRotation = nh_ptr->serviceClient<dobot_msgs::GetPose>(ROBOT_NAME + "/GetPose");
		initSuccess = true;
		return true;
	}

	void Dobot::robotPoseCallback(const geometry_msgs::Pose::ConstPtr &pose)
	{
		armPosition.setX(pose->position.x);
		armPosition.setY(pose->position.y);
		armPosition.setZ(pose->position.z);
	}

	void Dobot::toolCollisionCallback(const std_msgs::Bool::ConstPtr &msg) {
        toolCollision = msg->data;
    }


    BlockColor convert_rgb_to_block_color(const uint8_t r, const uint8_t g, const uint8_t b)
    {
        float c = r+g+b;
        float rf = r / c, gf = g / c, bf = b / c;

        if(rf > 0.3 && gf > 0.3 && bf < 0.4)
            return BlockColor::YELLOW;
        else if(rf > bf && rf > gf)
            return BlockColor::RED;
        else if(gf > rf && gf > bf)
            return BlockColor::GREEN;
        else if(bf > rf && bf > gf)
            return BlockColor::BLUE;

        return BlockColor::UNKNOWN;
    }
}
