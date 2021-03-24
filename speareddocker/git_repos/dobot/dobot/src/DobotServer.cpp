#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <algorithm>

#include "DobotDll.h"
#include "dobot_msgs/SetCmdTimeout.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include <tf_helper/tfHelper.h>

#include "../include/dobot/dobot.h"
#include <tf2/impl/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

const unsigned int TRANSPORTBELT_INDEX = 0;
float transportBeltSpeed = 0;
bool hasLinearRail = false;
const double ARM_IDLE_UPDATE_RATE = 4.0;
const double JOINT_UPDATE_RATE = 48.0;
const InfraredPort INFRARED_SENSOR_PORT = IF_PORT_GP2;
std::string ns = "Unknown";
tf2::Vector3 toolOffset = tf2::Vector3(dobot::env::ARM_EXTENSION / 1000.0,0,0);

bool SetCmdTimeoutService(dobot_msgs::SetCmdTimeout::Request &req, dobot_msgs::SetCmdTimeout::Response &res)
{
    res.result = SetCmdTimeout(req.timeout);

    return true;
}

void InitCmdTimeoutServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/SetCmdTimeout", SetCmdTimeoutService);
    serverVec.push_back(server);
}

/*
 * Device information
 */
#include "dobot_msgs/GetDeviceSN.h"
#include "dobot_msgs/SetDeviceName.h"
#include "dobot_msgs/GetDeviceName.h"
#include "dobot_msgs/GetDeviceVersion.h"

bool GetDeviceSNService(dobot_msgs::GetDeviceSN::Request &req, dobot_msgs::GetDeviceSN::Response &res)
{
    char deviceSN[256];

    res.result = GetDeviceSN(deviceSN, sizeof(deviceSN));
    if (res.result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceSN;
        res.deviceSN.data = ss.str();
    }

    return true;
}

bool SetDeviceNameService(dobot_msgs::SetDeviceName::Request &req, dobot_msgs::SetDeviceName::Response &res)
{
    res.result = SetDeviceName(req.deviceName.data.c_str());

    return true;
}

bool GetDeviceNameService(dobot_msgs::GetDeviceName::Request &req, dobot_msgs::GetDeviceName::Response &res)
{
    char deviceName[256];

    res.result = GetDeviceName(deviceName, sizeof(deviceName));
    if (res.result == DobotCommunicate_NoError) {
        std::stringstream ss;
        ss << deviceName;
        res.deviceName.data = ss.str();
    }

    return true;
}

bool GetDeviceVersionService(dobot_msgs::GetDeviceVersion::Request &req, dobot_msgs::GetDeviceVersion::Response &res)
{
    uint8_t majorVersion, minorVersion, revision;

    res.result = GetDeviceVersion(&majorVersion, &minorVersion, &revision);
    if (res.result == DobotCommunicate_NoError) {
        res.majorVersion = majorVersion;
        res.minorVersion = minorVersion;
        res.revision = revision;
    }

    return true;
}

bool IsLinearRailPositionHomed()
{
    IOMultiplexing multiplex;
    multiplex.address = 18;
    multiplex.multiplex = 5;
    uint64_t queuedCmdIndex;
    SetIOMultiplexing(&multiplex,false,&queuedCmdIndex);

    IODI ioDI;
    ioDI.address = 18;
    GetIODI(&ioDI);
    if(ioDI.level == 1){
        return true;
    }
    else {
        return false;
    }

}

void InitDeviceInfoServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/GetDeviceSN", GetDeviceSNService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetDeviceName", SetDeviceNameService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetDeviceName", GetDeviceNameService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetDeviceVersion", GetDeviceVersionService);
    serverVec.push_back(server);
}

/*
 * Pose
 */
#include "dobot_msgs/GetPose.h"

bool GetPoseService(dobot_msgs::GetPose::Request &req, dobot_msgs::GetPose::Response &res)
{
    Pose pose;

    res.result = GetPose(&pose);
    if (res.result == DobotCommunicate_NoError) {
        res.x = pose.x;
        res.y = pose.y;
        res.z = pose.z;
        res.r = pose.r;
        for (int i = 0; i < 4; i++) {
            res.jointAngle.push_back(pose.jointAngle[i]);
        }
    }

    return true;
}

void InitPoseServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/GetPose", GetPoseService);
    serverVec.push_back(server);
}

/*
 * Alarms
 */
#include "dobot_msgs/GetAlarmsState.h"
#include "dobot_msgs/ClearAllAlarmsState.h"

bool GetAlarmsStateService(dobot_msgs::GetAlarmsState::Request &req, dobot_msgs::GetAlarmsState::Response &res)
{
    uint8_t alarmsState[128];
    uint32_t len;

    res.result = GetAlarmsState(alarmsState, &len, sizeof(alarmsState));
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < len; i++) {
            res.alarmsState.push_back(alarmsState[i]);
        }
    }

    return true;
}

bool ClearAllAlarmsStateService(dobot_msgs::ClearAllAlarmsState::Request &req, dobot_msgs::ClearAllAlarmsState::Response &res)
{
    res.result = ClearAllAlarmsState();

    return true;
}

void InitAlarmsServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/GetAlarmsState", GetAlarmsStateService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/ClearAllAlarmsState", ClearAllAlarmsStateService);
    serverVec.push_back(server);
}

/*
 * HOME
 */
#include "dobot_msgs/SetHOMEParams.h"
#include "dobot_msgs/GetHOMEParams.h"
#include "dobot_msgs/SetHOMECmd.h"

bool SetHOMEParamsService(dobot_msgs::SetHOMEParams::Request &req, dobot_msgs::SetHOMEParams::Response &res)
{
    HOMEParams params;
    uint64_t queuedCmdIndex;

    params.x = req.x;
    params.y = req.y;
    params.z = req.z;
    params.r = req.r;

    res.result = SetHOMEParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetHOMEParamsService(dobot_msgs::GetHOMEParams::Request &req, dobot_msgs::GetHOMEParams::Response &res)
{
    HOMEParams params;

    res.result = GetHOMEParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.x = params.x;
        res.y = params.y;
        res.z = params.z;
        res.r = params.r;
    }

    return true;
}

bool SetHOMECmdService(dobot_msgs::SetHOMECmd::Request &req, dobot_msgs::SetHOMECmd::Response &res)
{
    HOMECmd cmd;
    uint64_t queuedCmdIndex;

    res.result = SetHOMECmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitHOMEServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/SetHOMEParams", SetHOMEParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetHOMEParams", GetHOMEParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetHOMECmd", SetHOMECmdService);
    serverVec.push_back(server);
}

/*
 * End effector
 */
#include "dobot_msgs/SetEndEffectorParams.h"
#include "dobot_msgs/GetEndEffectorParams.h"
#include "dobot_msgs/SetEndEffectorLaser.h"
#include "dobot_msgs/GetEndEffectorLaser.h"
#include "dobot_msgs/SetEndEffectorSuctionCup.h"
#include "dobot_msgs/GetEndEffectorSuctionCup.h"
#include "dobot_msgs/SetEndEffectorGripper.h"
#include "dobot_msgs/GetEndEffectorGripper.h"

bool SetEndEffectorParamsService(dobot_msgs::SetEndEffectorParams::Request &req, dobot_msgs::SetEndEffectorParams::Response &res)
{
    EndEffectorParams params;
    uint64_t queuedCmdIndex;

    params.xBias = req.xBias;
    params.yBias = req.yBias;
    params.zBias = req.zBias;

    res.result = SetEndEffectorParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorParamsService(dobot_msgs::GetEndEffectorParams::Request &req, dobot_msgs::GetEndEffectorParams::Response &res)
{
    EndEffectorParams params;

    res.result = GetEndEffectorParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xBias = params.xBias;
        res.yBias = params.yBias;
        res.zBias = params.zBias;
    }

    return true;
}

bool SetEndEffectorLaserService(dobot_msgs::SetEndEffectorLaser::Request &req, dobot_msgs::SetEndEffectorLaser::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorLaser(req.enableCtrl, req.on, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorLaserService(dobot_msgs::GetEndEffectorLaser::Request &req, dobot_msgs::GetEndEffectorLaser::Response &res)
{
    bool enableCtrl, on;

    res.result = GetEndEffectorLaser(&enableCtrl, &on);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.on = on;
    }

    return true;
}

bool SetEndEffectorSuctionCupService(dobot_msgs::SetEndEffectorSuctionCup::Request &req, dobot_msgs::SetEndEffectorSuctionCup::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorSuctionCup(req.enableCtrl, req.suck, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorSuctionCupService(dobot_msgs::GetEndEffectorSuctionCup::Request &req, dobot_msgs::GetEndEffectorSuctionCup::Response &res)
{
    bool enableCtrl, suck;

    res.result = GetEndEffectorSuctionCup(&enableCtrl, &suck);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.suck = suck;
    }

    return true;
}

bool SetEndEffectorGripperService(dobot_msgs::SetEndEffectorGripper::Request &req, dobot_msgs::SetEndEffectorGripper::Response &res)
{
    uint64_t queuedCmdIndex;

    res.result = SetEndEffectorGripper(req.enableCtrl, req.grip, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetEndEffectorGripperService(dobot_msgs::GetEndEffectorGripper::Request &req, dobot_msgs::GetEndEffectorGripper::Response &res)
{
    bool enableCtrl, grip;

    res.result = GetEndEffectorGripper(&enableCtrl, &grip);
    if (res.result == DobotCommunicate_NoError) {
        res.enableCtrl = enableCtrl;
        res.grip = grip;
    }

    return true;
}

void InitEndEffectorServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/SetEndEffectorParams", SetEndEffectorParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetEndEffectorParams", GetEndEffectorParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetEndEffectorLaser", SetEndEffectorLaserService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetEndEffectorLaser", GetEndEffectorLaserService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetEndEffectorSuctionCup", SetEndEffectorSuctionCupService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetEndEffectorSuctionCup", GetEndEffectorSuctionCupService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetEndEffectorGripper", SetEndEffectorGripperService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetEndEffectorGripper", GetEndEffectorGripperService);
    serverVec.push_back(server);
}

/*
 * JOG
 */
#include "dobot_msgs/SetJOGJointParams.h"
#include "dobot_msgs/GetJOGJointParams.h"
#include "dobot_msgs/SetJOGCoordinateParams.h"
#include "dobot_msgs/GetJOGCoordinateParams.h"
#include "dobot_msgs/SetJOGCommonParams.h"
#include "dobot_msgs/GetJOGCommonParams.h"
#include "dobot_msgs/SetJOGCmd.h"

bool SetJOGJointParamsService(dobot_msgs::SetJOGJointParams::Request &req, dobot_msgs::SetJOGJointParams::Response &res)
{
    JOGJointParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetJOGJointParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGJointParamsService(dobot_msgs::GetJOGJointParams::Request &req, dobot_msgs::GetJOGJointParams::Response &res)
{
    JOGJointParams params;

    res.result = GetJOGJointParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool SetJOGCoordinateParamsService(dobot_msgs::SetJOGCoordinateParams::Request &req, dobot_msgs::SetJOGCoordinateParams::Response &res)
{
    JOGCoordinateParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetJOGCoordinateParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGCoordinateParamsService(dobot_msgs::GetJOGCoordinateParams::Request &req, dobot_msgs::GetJOGCoordinateParams::Response &res)
{
    JOGCoordinateParams params;

    res.result = GetJOGCoordinateParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool SetJOGCommonParamsService(dobot_msgs::SetJOGCommonParams::Request &req, dobot_msgs::SetJOGCommonParams::Response &res)
{
    JOGCommonParams params;
    uint64_t queuedCmdIndex;

    params.velocityRatio = req.velocityRatio;
    params.accelerationRatio = req.accelerationRatio;
    res.result = SetJOGCommonParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetJOGCommonParamsService(dobot_msgs::GetJOGCommonParams::Request &req, dobot_msgs::GetJOGCommonParams::Response &res)
{
    JOGCommonParams params;

    res.result = GetJOGCommonParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.velocityRatio = params.velocityRatio;
        res.accelerationRatio = params.accelerationRatio;
    }

    return true;
}

bool SetJOGCmdService(dobot_msgs::SetJOGCmd::Request &req, dobot_msgs::SetJOGCmd::Response &res)
{
    JOGCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.isJoint = req.isJoint;
    cmd.cmd = req.cmd;
    res.result = SetJOGCmd(&cmd, false, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitJOGServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/SetJOGJointParams", SetJOGJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetJOGJointParams", GetJOGJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetJOGCoordinateParams", SetJOGCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetJOGCoordinateParams", GetJOGCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetJOGCommonParams", SetJOGCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetJOGCommonParams", GetJOGCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetJOGCmd", SetJOGCmdService);
    serverVec.push_back(server);
}

/*
 * PTP
 */
#include "dobot_msgs/SetPTPJointParams.h"
#include "dobot_msgs/GetPTPJointParams.h"
#include "dobot_msgs/SetPTPCoordinateParams.h"
#include "dobot_msgs/GetPTPCoordinateParams.h"
#include "dobot_msgs/SetPTPJumpParams.h"
#include "dobot_msgs/GetPTPJumpParams.h"
#include "dobot_msgs/SetPTPCommonParams.h"
#include "dobot_msgs/GetPTPCommonParams.h"
#include "dobot_msgs/SetPTPCmd.h"
#include "dobot_msgs/SetPTPWithLCmd.h"
#include "dobot_msgs/SetPTPLParams.h"

bool SetPTPJointParamsService(dobot_msgs::SetPTPJointParams::Request &req, dobot_msgs::SetPTPJointParams::Response &res)
{
    PTPJointParams params;
    uint64_t queuedCmdIndex;

    for (int i = 0; i < req.velocity.size(); i++) {
        params.velocity[i] = req.velocity[i];
    }
    for (int i = 0; i < req.acceleration.size(); i++) {
        params.acceleration[i] = req.acceleration[i];
    }
    res.result = SetPTPJointParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPJointParamsService(dobot_msgs::GetPTPJointParams::Request &req, dobot_msgs::GetPTPJointParams::Response &res)
{
    PTPJointParams params;

    res.result = GetPTPJointParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        for (int i = 0; i < 4; i++) {
            res.velocity.push_back(params.velocity[i]);
            res.acceleration.push_back(params.acceleration[i]);
        }
    }

    return true;
}

bool SetPTPCoordinateParamsService(dobot_msgs::SetPTPCoordinateParams::Request &req, dobot_msgs::SetPTPCoordinateParams::Response &res)
{
    PTPCoordinateParams params;
    uint64_t queuedCmdIndex;

    params.xyzVelocity = req.xyzVelocity;
    params.rVelocity = req.rVelocity;
    params.xyzAcceleration = req.xyzAcceleration;
    params.rAcceleration = req.rAcceleration;
    res.result = SetPTPCoordinateParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPCoordinateParamsService(dobot_msgs::GetPTPCoordinateParams::Request &req, dobot_msgs::GetPTPCoordinateParams::Response &res)
{
    PTPCoordinateParams params;

    res.result = GetPTPCoordinateParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xyzVelocity = params.xyzVelocity;
        res.rVelocity = params.rVelocity;
        res.xyzAcceleration = params.xyzAcceleration;
        res.rAcceleration = params.rAcceleration;
    }

    return true;
}

bool SetPTPJumpParamsService(dobot_msgs::SetPTPJumpParams::Request &req, dobot_msgs::SetPTPJumpParams::Response &res)
{
    PTPJumpParams params;
    uint64_t queuedCmdIndex;

    params.jumpHeight = req.jumpHeight;
    params.zLimit = req.zLimit;
    res.result = SetPTPJumpParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPJumpParamsService(dobot_msgs::GetPTPJumpParams::Request &req, dobot_msgs::GetPTPJumpParams::Response &res)
{
    PTPJumpParams params;

    res.result = GetPTPJumpParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.jumpHeight = params.jumpHeight;
        res.zLimit = params.zLimit;
    }

    return true;
}

bool SetPTPCommonParamsService(dobot_msgs::SetPTPCommonParams::Request &req, dobot_msgs::SetPTPCommonParams::Response &res)
{
    PTPCommonParams params;
    uint64_t queuedCmdIndex;

    params.velocityRatio = req.velocityRatio;
    params.accelerationRatio = req.accelerationRatio;
    res.result = SetPTPCommonParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetPTPCommonParamsService(dobot_msgs::GetPTPCommonParams::Request &req, dobot_msgs::GetPTPCommonParams::Response &res)
{
    PTPCommonParams params;

    res.result = GetPTPCommonParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.velocityRatio = params.velocityRatio;
        res.accelerationRatio = params.accelerationRatio;
    }

    return true;
}

bool SetPTPCmdService(dobot_msgs::SetPTPCmd::Request &req, dobot_msgs::SetPTPCmd::Response &res)
{
    PTPCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.ptpMode = req.ptpMode;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.r = req.r;
    res.result = SetPTPCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool SetPTPLParamsService(dobot_msgs::SetPTPLParams::Request &req, dobot_msgs::SetPTPLParams::Response &res)
{
    PTPLParams params;
    uint64_t queuedCmdIndex;

    params.acceleration = req.acceleration;
    params.velocity = req.velocity;
    res.result = SetPTPLParams(&params, true, &queuedCmdIndex);

    if(res.result == DobotCommunicate_NoError)
        res.queuedCmdIndex = queuedCmdIndex;

    return true;
}

bool SetPTPWithLCmdService(dobot_msgs::SetPTPWithLCmd::Request &req, dobot_msgs::SetPTPWithLCmd::Response &res)
{
    PTPWithLCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.ptpMode = req.ptpMode;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.r = req.r;
    cmd.l = req.l;
    res.result = SetPTPWithLCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitPTPServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/SetPTPJointParams", SetPTPJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetPTPJointParams", GetPTPJointParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetPTPCoordinateParams", SetPTPCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetPTPCoordinateParams", GetPTPCoordinateParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetPTPJumpParams", SetPTPJumpParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetPTPJumpParams", GetPTPJumpParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetPTPCommonParams", SetPTPCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetPTPCommonParams", GetPTPCommonParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetPTPCmd", SetPTPCmdService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetPTPWithLCmd", SetPTPWithLCmdService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetPTPLParams", SetPTPLParamsService);
    serverVec.push_back(server);

    if(hasLinearRail){
        uint64_t queuedCmdIndex;
        SetDeviceWithL(true,true,&queuedCmdIndex);
        ROS_INFO("%s has rail enabled!",ns.c_str());
    }
}

/*
 * CP
 */
#include "dobot_msgs/SetCPParams.h"
#include "dobot_msgs/GetCPParams.h"
#include "dobot_msgs/SetCPCmd.h"

bool SetCPParamsService(dobot_msgs::SetCPParams::Request &req, dobot_msgs::SetCPParams::Response &res)
{
    CPParams params;
    uint64_t queuedCmdIndex;

    params.planAcc = req.planAcc;
    params.juncitionVel = req.junctionVel;
    params.acc = req.acc;
    params.realTimeTrack = req.realTimeTrack;
    res.result = SetCPParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetCPParamsService(dobot_msgs::GetCPParams::Request &req, dobot_msgs::GetCPParams::Response &res)
{
    CPParams params;

    res.result = GetCPParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.planAcc = params.planAcc;
        res.junctionVel = params.juncitionVel;
        res.acc = params.acc;
        res.realTimeTrack = params.realTimeTrack;
    }

    return true;
}

bool SetCPCmdService(dobot_msgs::SetCPCmd::Request &req, dobot_msgs::SetCPCmd::Response &res)
{
    CPCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.cpMode = req.cpMode;
    cmd.x = req.x;
    cmd.y = req.y;
    cmd.z = req.z;
    cmd.velocity = req.velocity;

    res.result = SetCPCmd(&cmd,true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitCPServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/SetCPParams", SetCPParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetCPParams", GetCPParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetCPCmd", SetCPCmdService);
    serverVec.push_back(server);
}

/*
 * ARC
 */
#include "dobot_msgs/SetARCParams.h"
#include "dobot_msgs/GetARCParams.h"
#include "dobot_msgs/SetARCCmd.h"

bool SetARCParamsService(dobot_msgs::SetARCParams::Request &req, dobot_msgs::SetARCParams::Response &res)
{
    ARCParams params;
    uint64_t queuedCmdIndex;

    params.xyzVelocity = req.xyzVelocity;
    params.rVelocity = req.rVelocity;
    params.xyzAcceleration = req.xyzAcceleration;
    params.rAcceleration = req.rAcceleration;
    res.result = SetARCParams(&params, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetARCParamsService(dobot_msgs::GetARCParams::Request &req, dobot_msgs::GetARCParams::Response &res)
{
    ARCParams params;

    res.result = GetARCParams(&params);
    if (res.result == DobotCommunicate_NoError) {
        res.xyzVelocity = params.xyzVelocity;
        res.rVelocity = params.rVelocity;
        res.xyzAcceleration = params.xyzAcceleration;
        res.rAcceleration = params.rAcceleration;
    }

    return true;
}

bool SetARCCmdService(dobot_msgs::SetARCCmd::Request &req, dobot_msgs::SetARCCmd::Response &res)
{
    ARCCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.cirPoint.x = req.x1;
    cmd.cirPoint.y = req.y1;
    cmd.cirPoint.z = req.z1;
    cmd.cirPoint.r = req.r1;
    cmd.toPoint.x = req.x2;
    cmd.toPoint.y = req.y2;
    cmd.toPoint.z = req.z2;
    cmd.toPoint.r = req.r2;

    res.result = SetARCCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitARCServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/SetARCParams", SetARCParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetARCParams", GetARCParamsService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetARCCmd", SetARCCmdService);
    serverVec.push_back(server);
}

/*
 * WAIT
 */
#include "dobot_msgs/SetWAITCmd.h"

bool SetWAITCmdService(dobot_msgs::SetWAITCmd::Request &req, dobot_msgs::SetWAITCmd::Response &res)
{
    WAITCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.timeout = req.timeout;
    res.result = SetWAITCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitWAITServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/SetWAITCmd", SetWAITCmdService);
    serverVec.push_back(server);
}

/*
 * TRIG
 */
#include "dobot_msgs/SetTRIGCmd.h"

bool SetTRIGCmdService(dobot_msgs::SetTRIGCmd::Request &req, dobot_msgs::SetTRIGCmd::Response &res)
{
    TRIGCmd cmd;
    uint64_t queuedCmdIndex;

    cmd.address = req.address;
    cmd.mode = req.mode;
    cmd.condition = req.condition;
    cmd.threshold = req.threshold;
    res.result = SetTRIGCmd(&cmd, true, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitTRIGServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/SetTRIGCmd", SetTRIGCmdService);
    serverVec.push_back(server);
}

/*
 * EIO
 */
#include "dobot_msgs/SetIOMultiplexing.h"
#include "dobot_msgs/GetIOMultiplexing.h"
#include "dobot_msgs/SetIODO.h"
#include "dobot_msgs/GetIODO.h"
#include "dobot_msgs/SetIOPWM.h"
#include "dobot_msgs/GetIOPWM.h"
#include "dobot_msgs/GetIODI.h"
#include "dobot_msgs/GetIOADC.h"
#include "dobot_msgs/SetEMotor.h"
#include "dobot_msgs/SetEMotorS.h"

bool SetIOMultiplexingService(dobot_msgs::SetIOMultiplexing::Request &req, dobot_msgs::SetIOMultiplexing::Response &res)
{
    IOMultiplexing ioMultiplexing;
    uint64_t queuedCmdIndex;

    ioMultiplexing.address = req.address;
    ioMultiplexing.multiplex = req.multiplex;
    res.result = SetIOMultiplexing(&ioMultiplexing, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetIOMultiplexingService(dobot_msgs::GetIOMultiplexing::Request &req, dobot_msgs::GetIOMultiplexing::Response &res)
{
    IOMultiplexing ioMultiplexing;

    ioMultiplexing.address = req.address;
    res.result = GetIOMultiplexing(&ioMultiplexing);
    if (res.result == DobotCommunicate_NoError) {
        res.multiplex = ioMultiplexing.multiplex;
    }

    return true;
}

bool SetIODOService(dobot_msgs::SetIODO::Request &req, dobot_msgs::SetIODO::Response &res)
{
    IODO ioDO;
    uint64_t queuedCmdIndex;

    ioDO.address = req.address;
    ioDO.level = req.level;
    res.result = SetIODO(&ioDO, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetIODOService(dobot_msgs::GetIODO::Request &req, dobot_msgs::GetIODO::Response &res)
{
    IODO ioDO;

    ioDO.address = req.address;
    res.result = GetIODO(&ioDO);
    if (res.result == DobotCommunicate_NoError) {
        res.level = ioDO.level;
    }

    return true;
}

bool SetIOPWMService(dobot_msgs::SetIOPWM::Request &req, dobot_msgs::SetIOPWM::Response &res)
{
    IOPWM ioPWM;
    uint64_t queuedCmdIndex;

    ioPWM.address = req.address;
    ioPWM.frequency = req.frequency;
    ioPWM.dutyCycle = req.dutyCycle;
    res.result = SetIOPWM(&ioPWM, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool GetIOPWMService(dobot_msgs::GetIOPWM::Request &req, dobot_msgs::GetIOPWM::Response &res)
{
    IOPWM ioPWM;

    ioPWM.address = req.address;
    res.result = GetIOPWM(&ioPWM);
    if (res.result == DobotCommunicate_NoError) {
        res.frequency = ioPWM.frequency;
        res.dutyCycle = ioPWM.dutyCycle;
    }

    return true;
}

bool GetIODIService(dobot_msgs::GetIODI::Request &req, dobot_msgs::GetIODI::Response &res)
{
    IODI ioDI;

    ioDI.address = req.address;
    res.result = GetIODI(&ioDI);
    if (res.result == DobotCommunicate_NoError) {
        res.level = ioDI.level;
    }

    return true;
}

bool GetIOADCService(dobot_msgs::GetIOADC::Request &req, dobot_msgs::GetIOADC::Response &res)
{
    IOADC ioADC;

    ioADC.address = req.address;
    res.result = GetIOADC(&ioADC);
    if (res.result == DobotCommunicate_NoError) {
        res.value = ioADC.value;
    }

    return true;
}

bool SetEMotorService(dobot_msgs::SetEMotor::Request &req, dobot_msgs::SetEMotor::Response &res)
{
    EMotor eMotor;
    uint64_t queuedCmdIndex;

    eMotor.index = req.index;
    eMotor.isEnabled = req.isEnabled;
    eMotor.speed = req.speed;
    res.result = SetEMotor(&eMotor, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

bool SetEMotorSService(dobot_msgs::SetEMotorS::Request &req, dobot_msgs::SetEMotorS::Response &res)
{
    EMotorS eMotor;
    uint64_t queuedCmdIndex;

    eMotor.index = req.index;
    eMotor.isEnabled = req.enabled;
    eMotor.speed = req.speed;
    eMotor.distance = req.distance;
    res.result = SetEMotorS(&eMotor, req.isQueued, &queuedCmdIndex);
    if (res.result == DobotCommunicate_NoError) {
        res.queuedCmdIndex = queuedCmdIndex;
    }

    return true;
}

void InitEIOServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/SetIOMultiplexing", SetIOMultiplexingService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetIOMultiplexing", GetIOMultiplexingService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetIODO", SetIODOService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetIODO", GetIODOService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetIOPWM", SetIOPWMService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetIOPWM", GetIOPWMService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetIODI", GetIODIService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/GetIOADC", GetIOADCService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetEMotor", SetEMotorService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetEMotorS", SetEMotorSService);
    serverVec.push_back(server);
}

/*
 * Queued command control
 */
#include "dobot_msgs/SetQueuedCmdStartExec.h"
#include "dobot_msgs/SetQueuedCmdStopExec.h"
#include "dobot_msgs/SetQueuedCmdForceStopExec.h"
#include "dobot_msgs/SetQueuedCmdClear.h"
#include "dobot_msgs/SetTransportbeltSpeed.h"
#include "dobot_msgs/SetColorSensor.h"
#include "dobot_msgs/GetColorSensor.h"
#include "dobot_msgs/SetInfraredSensor.h"
#include "dobot_msgs/GetInfraredSensor.h"
#include "dobot_msgs/GetPoseL.h"
#include "dobot_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "../include/dobot/dobot.h"

bool SetQueuedCmdStartExecService(dobot_msgs::SetQueuedCmdStartExec::Request &req, dobot_msgs::SetQueuedCmdStartExec::Response &res)
{
    res.result = SetQueuedCmdStartExec();

    return true;
}

bool SetQueuedCmdStopExecService(dobot_msgs::SetQueuedCmdStopExec::Request &req, dobot_msgs::SetQueuedCmdStopExec::Response &res)
{
    res.result = SetQueuedCmdStopExec();

    return true;
}

bool SetQueuedCmdForceStopExecService(dobot_msgs::SetQueuedCmdForceStopExec::Request &req, dobot_msgs::SetQueuedCmdForceStopExec::Response &res)
{
    res.result = SetQueuedCmdForceStopExec();

    return true;
}

bool SetQueuedCmdClearService(dobot_msgs::SetQueuedCmdClear::Request &req, dobot_msgs::SetQueuedCmdClear::Response &res)
{
    res.result = SetQueuedCmdClear();

    return true;
}

void InitQueuedCmdServices(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec)
{
    ros::ServiceServer server;

    server = n.advertiseService(n.getNamespace() + "/SetQueuedCmdStartExec", SetQueuedCmdStartExecService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetQueuedCmdStopExec", SetQueuedCmdStopExecService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetQueuedCmdForceStopExec", SetQueuedCmdForceStopExecService);
    serverVec.push_back(server);
    server = n.advertiseService(n.getNamespace() + "/SetQueuedCmdClear", SetQueuedCmdClearService);
    serverVec.push_back(server);
}

bool SetTransportbeltService(dobot_msgs::SetTransportbeltSpeed::Request &req, dobot_msgs::SetTransportbeltSpeed::Response &res){
	EMotor eMotor;
	uint64_t queuedCmdIndex;
	eMotor.index = TRANSPORTBELT_INDEX;
	//maximum 120mm/s
	req.speed = std::min(120.0F,std::max(req.speed,-120.0F));
	transportBeltSpeed = req.speed;
	//1000 steps/second is 60cm after 94s.
	int32_t stepsPerSecond = (int32_t)(req.speed / 0.00638297872);
	eMotor.speed = stepsPerSecond;
	eMotor.isEnabled = std::fabs(stepsPerSecond) > 0;

	res.result = SetEMotor(&eMotor, req.isQueued, &queuedCmdIndex);
	if (res.result == DobotCommunicate_NoError) {
		res.queuedCmdIndex = queuedCmdIndex;
		return true;
	}else{
		return false;
	}
}

bool SetColorSensor(dobot_msgs::SetColorSensor::Request &req, dobot_msgs::SetColorSensor::Response &res){
	ColorPort colorPort;
	switch(req.colorPort){
		case 1: {
			colorPort = CL_PORT_GP1;
			break;
		}
		case 2: {
			colorPort = CL_PORT_GP2;
			break;
		}
		case 4: {
			colorPort = CL_PORT_GP4;
			break;
		}
		case 5: {
			colorPort = CL_PORT_GP5;
			break;
		}
		default:{
			ROS_ERROR("Unknown color sensor port: %i. Only GP1, GP2, GP4, GP5 allowed!",req.colorPort);
			return false;
		}
	}

	res.result = SetColorSensor(req.enable,colorPort);
	if (res.result == DobotCommunicate_NoError) {
		return true;
	}else{
		return false;
	}
}

bool SetInfraredSensor(dobot_msgs::SetInfraredSensor::Request &req, dobot_msgs::SetInfraredSensor::Response &res){
	InfraredPort infraredPort;
	switch (req.infraredPort) {
	case 1: {
		infraredPort = IF_PORT_GP1;
		break;
	}
	case 2: {
		infraredPort = IF_PORT_GP2;
		break;
	}
	case 4: {
		infraredPort = IF_PORT_GP4;
		break;
	}
	case 5: {
		infraredPort = IF_PORT_GP5;
		break;
	}
	default: {
		ROS_ERROR("Unknown infrared sensor port: %i. Only GP1, GP2, GP4, GP5 allowed!", req.infraredPort);
		return false;
	}
	}

	res.result = SetInfraredSensor(req.enable, infraredPort);
	if (res.result == DobotCommunicate_NoError) {
		return true;
	} else {
		return false;
	}
}

bool GetColorSensor(dobot_msgs::GetColorSensor::Request &req, dobot_msgs::GetColorSensor::Response &res){
	GetColorSensor(&res.r,&res.g,&res.b);
	if (res.result == DobotCommunicate_NoError) {
		return true;
	}else{
		return false;
	}
}

bool GetInfraredSensor(dobot_msgs::GetInfraredSensor::Request &req, dobot_msgs::GetInfraredSensor::Response &res){
	InfraredPort infraredPort;
	switch (req.infraredPort) {
	case 1: {
		infraredPort = IF_PORT_GP1;
		break;
	}
	case 2: {
		infraredPort = IF_PORT_GP2;
		break;
	}
	case 4: {
		infraredPort = IF_PORT_GP4;
		break;
	}
	case 5: {
		infraredPort = IF_PORT_GP5;
		break;
	}
	default: {
		ROS_ERROR("Unknown infrared sensor port: %i. Only GP1, GP2, GP4, GP5 allowed!", req.infraredPort);
		return false;
	}
	}

	res.result = GetInfraredSensor(infraredPort,&res.value);
    std::cout << "GetInfraredSensor: " << infraredPort << ", " << int(res.value) << "\n";
	if (res.result == DobotCommunicate_NoError) {
		return true;
	} else {
		return false;
	}
}

bool GetPoseLService(dobot_msgs::GetPoseL::Request &req, dobot_msgs::GetPoseL::Response &res)
{
    res.result = GetPoseL(&res.l);

    if(res.result == DobotCommunicate_NoError)
        return true;
    else
        return false;

}

bool SetToolOffset(dobot_msgs::Vector3::Request &req, dobot_msgs::Vector3::Response &res)
{
	toolOffset = tf2::Vector3(req.xIn,req.yIn,req.zIn);
	ROS_INFO("[%s]: Changed tool offset to: [%f,%f,%f]",ns.c_str(),toolOffset.x(),toolOffset.y(),toolOffset.z());
	return true;
}

void InitSensorsAndMotors(ros::NodeHandle &n, std::vector<ros::ServiceServer> &serverVec, ros::Publisher &jointPub,
                            ros::Publisher &linear_rail_publisher) {
	/*
	ros::ServiceServer server;
	server = n.advertiseService(n.getNamespace() + "/SetTransportbeltSpeed", SetTransportbeltService);
	serverVec.push_back(server);
	server = n.advertiseService(n.getNamespace() + "/SetColorSensor", SetColorSensor);
	serverVec.push_back(server);
	server = n.advertiseService(n.getNamespace() + "/GetColorSensor", GetColorSensor);
	serverVec.push_back(server);
	server = n.advertiseService(n.getNamespace() + "/SetInfraredSensor", SetInfraredSensor);
	serverVec.push_back(server);
	server = n.advertiseService(n.getNamespace() + "/GetInfraredSensor", GetInfraredSensor);
	serverVec.push_back(server);*/

	jointPub = n.advertise<sensor_msgs::JointState>(n.getNamespace() + "/joint_states", 1);

	ros::ServiceServer server;
	server = n.advertiseService(n.getNamespace() + "/SetTfToolOffset",SetToolOffset);
			serverVec.push_back(server);

	if(hasLinearRail)
    {
		linear_rail_publisher = n.advertise<std_msgs::Float64>(n.getNamespace() + "/linear_rail/pos", 1);
		server = n.advertiseService(n.getNamespace() + "/GetPoseL", GetPoseLService);
		serverVec.push_back(server);
    }

}

void publishTransportBeltSpeed(ros::Publisher &speed_publisher){
    std_msgs::Float64 speed;
    speed.data = transportBeltSpeed;
    speed_publisher.publish(speed);
}

void publishInfraredState(ros::Publisher &infrared_publisher){
    std_msgs::Bool isBlocked;
    uint8_t infrared_value;
    int32_t infrared_result;

    infrared_result = GetInfraredSensor(INFRARED_SENSOR_PORT,&infrared_value);
    if (!(infrared_result == DobotCommunicate_NoError)) {
        ROS_ERROR("Communication error: %i", infrared_result);
    }
    isBlocked.data = infrared_value;

    infrared_publisher.publish(isBlocked);
}

void publishLinearRailState(ros::Publisher &linear_rail_publisher, float *current_posL, tf2_ros::TransformBroadcaster *tfBroadcaster)
{
    std_msgs::Float64 pos;

    int32_t result = GetPoseL(current_posL);
    pos.data = *current_posL;

    if(result != DobotCommunicate_NoError)
        ROS_ERROR("Communication error: %i", result);

    tf2::Transform base(tf2::Quaternion(0,0,0,1),tf2::Vector3(0,-*current_posL/1000.0,0));
    geometry_msgs::TransformStamped ts;
    ts = TfHelper::toTransformStamped(base,ros::Time::now(),ns + "_rail",ns);
    tfBroadcaster->sendTransform(ts);

    linear_rail_publisher.publish(pos);
}

void publishJointState(ros::Publisher *jointPub, std::string jointName, float angle){
	sensor_msgs::JointState msg;
	msg.effort.push_back(0.0);
	msg.header.frame_id = jointName;
	msg.name.push_back(jointName);
	msg.position.push_back(angle);
	msg.velocity.push_back(0.0);
	jointPub->publish(msg);
}

void publishJointStates(ros::Publisher *endEffectorPub, ros::Publisher *jointPub, Pose *current_pos, tf2_ros::TransformBroadcaster *tfBroadcaster){
	GetPose(current_pos);
	geometry_msgs::Pose pose;

	pose.position.x = current_pos->x;
	pose.position.y = current_pos->y;
	pose.position.z = current_pos->z;
	/*
	pose.orientation.w = std::cos(current_pos->r / 2.0);
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = std::sin(current_pos->r / 2.0);*/

	tf2::Quaternion rotation(0.0,0.0,M_PI * current_pos->r / 180.0);
	pose.orientation.x = rotation.x();
	pose.orientation.y = rotation.y();
	pose.orientation.z = rotation.z();
	pose.orientation.w = rotation.w();

	endEffectorPub->publish(pose);

	geometry_msgs::TransformStamped ts;
	tf2::Vector3 armToEndEffector(pose.position.x,pose.position.y,pose.position.z);//Coordinates in mm
	armToEndEffector = armToEndEffector / 1000.0; //Converting to m
	tf2::Transform baseToEndEffector(tf2::Quaternion(0,0,0,1),armToEndEffector);

	ts = TfHelper::toTransformStamped(baseToEndEffector,ros::Time::now(),ns,ns + "_endEffector");
	tfBroadcaster->sendTransform(ts);

	//Extend the arm length to constant 7cm, because the tool is 7cm longer.
	tf2::Vector3 armToEndEffector2D = tf2::Vector3(armToEndEffector.x(),armToEndEffector.y(),0.0);
	armToEndEffector = armToEndEffector2D.normalize() * toolOffset.x();
	tf2::Transform endEffectorToTool(tf2::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w),armToEndEffector);
	ts = TfHelper::toTransformStamped(endEffectorToTool,ros::Time::now(),ns + "_endEffector",ns + "_tool");
	tfBroadcaster->sendTransform(ts);

	tf2::Transform standToBase(tf2::Quaternion(0,0,(current_pos->jointAngle[0] / 360.0) * 2 * M_PI),tf2::Vector3(0,0,0));
	ts = TfHelper::toTransformStamped(standToBase,ros::Time::now(),ns,ns + "_base_link");
	tfBroadcaster->sendTransform(ts);

	tf2::Transform baseToShoulder(tf2::Quaternion((current_pos->jointAngle[1] / 360.0) * 2 * M_PI,0,0),tf2::Vector3(0,0,0.082477));
	ts = TfHelper::toTransformStamped(baseToShoulder,ros::Time::now(),ns + "_base_link", ns + "_shoulder_link");
	tfBroadcaster->sendTransform(ts);

	double elbowAngle = -current_pos->jointAngle[1] + current_pos->jointAngle[2];
	tf2::Transform shoulderToElbow(tf2::Quaternion((elbowAngle / 360.0) * 2 * M_PI,0,0),tf2::Vector3(0,0,0.13635));
	ts = TfHelper::toTransformStamped(shoulderToElbow,ros::Time::now(),ns + "_shoulder_link", ns + "_elbow_link");
	tfBroadcaster->sendTransform(ts);

	tf2::Transform elbowToHand(tf2::Quaternion(((-current_pos->jointAngle[2] / 360.0) * 2 * M_PI) + M_PI_2,0,0),tf2::Vector3(0.14847,0,0.0));
	ts = TfHelper::toTransformStamped(elbowToHand,ros::Time::now(),ns + "_elbow_link", ns + "_hand_link");
	tfBroadcaster->sendTransform(ts);

	tf2::Transform handToTool(tf2::Quaternion((current_pos->jointAngle[3] / 360.0) * 2 * M_PI,0,0),tf2::Vector3(0,0,0));
	ts = TfHelper::toTransformStamped(handToTool,ros::Time::now(),ns + "_hand_link", ns + "_tool_link");
	tfBroadcaster->sendTransform(ts);

	sensor_msgs::JointState msg;
	msg.header.frame_id = ns;

	msg.effort.resize(4);//Create empty efforts and velocitys. We can not measure it...
	msg.velocity.resize(4);
	msg.name.push_back("arm_base");
	msg.position.push_back(current_pos->jointAngle[0]);
	msg.name.push_back("arm_elbow");
	msg.position.push_back(current_pos->jointAngle[1]);
	msg.name.push_back("arm_shoulder");
	msg.position.push_back(current_pos->jointAngle[2]);
	msg.name.push_back("arm_tool");
	msg.position.push_back(current_pos->jointAngle[3]);
	jointPub->publish(msg);
}

void publishArmState(ros::Publisher *idlePub, Pose *last_pos, float *last_posL, Pose *current_pos, float *current_posL){
	double sumOfSpeeds  = std::fabs(last_pos->jointAngle[0] - current_pos->jointAngle[0]);
	sumOfSpeeds += std::fabs(last_pos->jointAngle[1] - current_pos->jointAngle[1]);
	sumOfSpeeds += std::fabs(last_pos->jointAngle[2] - current_pos->jointAngle[2]);
	sumOfSpeeds += std::fabs(last_pos->jointAngle[3] - current_pos->jointAngle[3]);
	if(hasLinearRail){
		sumOfSpeeds += std::fabs((*last_posL) - (*current_posL));
		*last_posL = *current_posL;
	}

	std_msgs::Bool msg;
	bool idle;
	idle = sumOfSpeeds < 0.01;
	msg.data = idle;
	idlePub->publish(msg);

	*last_pos = *current_pos;
}

void initPos(Pose *pos){
	pos->r = 0.0; pos->x = 0.0;
	pos->y = 0.0; pos->z = 0.0;
	pos->jointAngle[0] = 0.0; pos->jointAngle[1] = 0.0;
	pos->jointAngle[2] = 0.0; pos->jointAngle[3] = 0.0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        ROS_ERROR("[USAGE]: Application portName [robot_namespace]");
        return -1;
    }
    // Connect Dobot before start the service
    int result = ConnectDobot(argv[1], 115200, 0, 0);
    switch (result) {
        case DobotConnect_NoError:
        break;
        case DobotConnect_NotFound:
            ROS_ERROR("Dobot not found!");
            return -2;
        break;
        case DobotConnect_Occupied:
            ROS_ERROR("Invalid port name or Dobot is occupied by other application!");
            return -3;
        break;
        default:
        break;
    }

    //Get RobotName
    char deviceName[256];
    result = GetDeviceName(deviceName, sizeof(deviceName));
    if(result != DobotConnect_NoError) return -4;
    ns = std::string(deviceName);
    if(ns.length() == 0 || ns == "0") ns = "Unnamed_Device";
    ROS_INFO("Has found %s!",ns.c_str());
    if(ns == dobot::dobot_names::DOBOT_RAIL) hasLinearRail = true;

    ros::init(argc, argv, ns.c_str());
    ros::NodeHandle n(ns.c_str());

    std::vector<ros::ServiceServer> serverVec;
    ros::Publisher jointStatePubVec;
    //ros::Publisher infrared_publisher;
    //ros::Publisher speed_publisher;
    tf2_ros::TransformBroadcaster tfBroadcaster;
    ros::Publisher linear_rail_publisher;
    Pose last_pos, current_pos;
    float last_posL = 0.0;
    float current_posL = 0.0;
    initPos(&last_pos);
	initPos(&current_pos);

    InitCmdTimeoutServices(n, serverVec);
    InitDeviceInfoServices(n, serverVec);
    InitPoseServices(n, serverVec);
    InitAlarmsServices(n, serverVec);
    InitHOMEServices(n, serverVec);
    InitEndEffectorServices(n, serverVec);
    InitJOGServices(n, serverVec);
    InitPTPServices(n, serverVec);
    InitCPServices(n, serverVec);
    InitARCServices(n, serverVec);
    InitWAITServices(n, serverVec);
    InitTRIGServices(n, serverVec);
    InitEIOServices(n, serverVec);
    InitQueuedCmdServices(n, serverVec);
    InitSensorsAndMotors(n, serverVec, jointStatePubVec, linear_rail_publisher);


    if(hasLinearRail)
    {
		if (IsLinearRailPositionHomed()) {
			ROS_INFO("%s is at the left most position!", ns.c_str());
		} else {
			ROS_ERROR("%s is not at the left most postion! Aborting...", ns.c_str());
			return 4;
		}
    }

    ros::Publisher endEffectorPub = n.advertise<geometry_msgs::Pose>(n.getNamespace() + "/arm/Pose", 1);
    ros::Publisher idlePub = n.advertise<std_msgs::Bool>(n.getNamespace() + "/idle",1);

    ROS_INFO("Dobot service running with namespace %s", n.getNamespace().c_str());

    SetCmdTimeout(3000);

    ros::Duration(0.1).sleep(); //Check that clock has been published and ros::Time is working.
    ros::Time lastArmUpdateTime = ros::Time::now();
    ros::Time lastJointUpdateTime = ros::Time::now();

    //Needed to decide if robot is idle
    while(ros::ok()){
    	ros::Time current_time = ros::Time::now();

    	if((current_time - lastJointUpdateTime).toSec() > (1.0 / JOINT_UPDATE_RATE)){
    		publishJointStates(&endEffectorPub,&jointStatePubVec,&current_pos,&tfBroadcaster);
    		lastJointUpdateTime = current_time;
    		if(hasLinearRail)
            {
    		    publishLinearRailState(linear_rail_publisher,&current_posL,&tfBroadcaster);
            }
    	}
    	if((current_time - lastArmUpdateTime).toSec() > (1.0 / ARM_IDLE_UPDATE_RATE)){
    		publishArmState(&idlePub,&last_pos,&last_posL,&current_pos,&current_posL);
    		lastArmUpdateTime = current_time;
    	}
    	ros::spinOnce();
    }
    std::cout << "Disconnecting from " << ns << "." << std::endl;

    // Disconnect Dobot
    SetQueuedCmdForceStopExec();
    DisconnectDobot();

    return 0;
}

