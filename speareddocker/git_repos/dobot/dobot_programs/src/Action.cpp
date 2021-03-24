//
// Created by speeters on 13.05.19.
//

#include "../include/Action.h"



/**
 * SCANRFIDACTION
 */

/**
 * Constructor of ScanRFIDAction
 * @param name name of the action server
 * @param handle ros node handle shared pointer
 * @param dob dobot used
 * @param node rfid node that is involved in the action
 */
ScanRFIDAction::ScanRFIDAction(std::string name, std::shared_ptr<ros::NodeHandle> handle, dobot::Dobot &dob , arduino::RFIDNode& node):
        Action<dobot_msgs::ScanRFIDGoalConstPtr>(name, handle, dob),
        action_server(*handle, name, boost::bind(&ScanRFIDAction::callback, this, _1), false),
        arduino_node(node){
    action_server.start();
    std::cout << "ScanRFIDAction" << std::endl;
}


/**
 * Callback that will be executed by the action server
 */
void ScanRFIDAction::callback(const dobot_msgs::ScanRFIDGoalConstPtr &goal) {
    ROS_INFO("Executing scanRFIDAction");


    // get approximated height of block in meters
    double block_height = goal.get()->block_height;
    double safety_margin = 20;

    ROS_INFO_STREAM("Block height: " << block_height);


    // disable suction cup
    //dobot.enableSuctionCup(false);

    // move arm to wait position
    //dobotMoveToPosition(dobot::env::R_WAIT_POSITION);

    // move arm to lkw pickup position
    //dobotMoveToPosition(dobot::env::R_LKW_PICKUP_POSITION);

    // enable suction cup
    dobot.enableSuctionCup(true);

    ROS_INFO_STREAM("Move arm to RFID_SCAN_POSITION.");

    // move arm to rfid scan position
    tf2::Vector3 scan_position = dobot::env::SCAN_RFID_POSITION;


    ROS_INFO_STREAM("Move to Scanner.");
    //dobot.moveToPositionWithSafetyMarginZ(scan_position, 0, 0, 20);
    scan_position.setZ(scan_position.getZ() + (block_height - 10));
    dobot.moveToPositionWithSafetyMarginZ(scan_position,0,0,safety_margin);

    ROS_INFO_STREAM("Scan.");
    // wait for rfid sensor
    ros::Duration(0.5).sleep();


    // create result and return it to client
    uint8_t b0 = 0, b1 = 0, b2 = 0, b3 = 0;
    result.success = arduino_node.get_rfid_uid(b0, b1, b2, b3);

    ROS_INFO_STREAM(
            "Scanned UID:" << unsigned(b0) << " " << unsigned(b1) << " " << unsigned(b2) << " " << unsigned(b3));

    result.b0 = b0;
    result.b1 = b1;
    result.b2 = b2;
    result.b3 = b3;


    action_server.setSucceeded(result);

    // move arm back to wait position
    dobot.moveToPositionWithLBlocking(dobot::env::R_WAIT_POSITION, 0,0);
    ROS_INFO("Finished scanRFIDAction");


}


/**
 * TruckBuildAction
**/

TruckBuildAction::TruckBuildAction(std::string name, std::shared_ptr<ros::NodeHandle> handle, dobot::Dobot &dob) :
        Action<dobot_msgs::TruckBuildGoalConstPtr>(name, handle, dob),
        action_server(*handle, name, boost::bind(&TruckBuildAction::callback, this, _1), false) {
    action_server.start();
    tfHelper = std::make_unique<TfHelper>();
    markerManager = std::make_unique<MarkerHelper>(handle,"/TruckBuild/marker","objects");
    std::cout << "TruckBuildAction" << std::endl;
}



tf2::Vector3 TRUCK_BASE_RELATIVE(30,55,30);
tf2::Vector3 TRAILER_BASE_RELATIVE(152,55,35);

tf2::Vector3 TRUCK_COCKPIT_RELATIVE(-1,54,82);

tf2::Vector3 BLOCK_1_RELATIVE(56,52,75);
tf2::Vector3 BLOCK_2_RELATIVE(136,52,75);
tf2::Vector3 BLOCK_3_RELATIVE(198,52,75);
void TruckBuildAction::callback(const dobot_msgs::TruckBuildGoalConstPtr &goal) {

    // get position and rotation of grid
    // red x
    // green y



    ROS_INFO_STREAM("Started TruckBuildAction");
    struct dobot::rfid::uid uid = {goal.get()->b0, goal.get()->b1, goal.get()->b2, goal.get()->b3};

    // determine relative point in gridConstructionFrame



    //zero position (offset couse of camera inconsistencies
    tf2::Vector3 zeroPosition(-5,0,0);

    tf2::Vector3 position;
    double linear_rail = 0;


    if (uid == dobot::rfid::block_1_uid) {
        position = zeroPosition +  BLOCK_1_RELATIVE;
    } else if (uid == dobot::rfid::block_2_uid) {
        position = zeroPosition + BLOCK_2_RELATIVE;
    } else if (uid == dobot::rfid::block_3_uid) {
        position = zeroPosition + BLOCK_3_RELATIVE;
    } else if (uid == dobot::rfid::truck_base_uid) {
        position = zeroPosition + TRUCK_BASE_RELATIVE;
    } else if (uid == dobot::rfid::trailer_base_uid) {
        position = zeroPosition +  TRAILER_BASE_RELATIVE;
    } else if (uid == dobot::rfid::truck_cockpit_uid) {
        position = zeroPosition +  TRUCK_COCKPIT_RELATIVE;
    } else {
        result.success = 0;
        action_server.setAborted(result,"uid not known");
        return;
    }





    position /= 1000;


    //position
    tf2::Vector3 position_in_dobot_frame;

    dobot.convertToDobotPointWithL(position, position_in_dobot_frame, linear_rail, "gridDetection");

    //rotation
    tf2::Quaternion rot_quat;
    rot_quat.setRPY(0,0,0);

    tfHelper->transformQuaternion("gridDetection", "Dobot_Rail", ros::Time(0), rot_quat);
    tf2::Matrix3x3 m(rot_quat);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double rotation =  (yaw * (double) 180) / M_PI + 90;

    ROS_INFO_STREAM("Position Grid: " << position.x() <<","<< position.y() <<","<< position.z() <<",");
    ROS_INFO_STREAM("Transformed to:" << position_in_dobot_frame.x() << "," << position_in_dobot_frame.y() << "," << position_in_dobot_frame.z());
    ROS_INFO_STREAM("Linear Rail" << linear_rail);
    ROS_INFO_STREAM("Rotation: " << rotation);
    ROS_INFO_STREAM("Move arm to truck grid and place block");
    //dobot.moveToPositionWithSafetyMarginZ(position, rotation, linear_rail, 20);

    // publish markers for debugging
    tf2::Transform arrow(rot_quat, position_in_dobot_frame);

    ROS_INFO_STREAM("Publish Marker");
    markerManager->publishMarker(position_in_dobot_frame, "Dobot_Rail_rail", 0,1,1,0, "Middle", 5);
    markerManager->publishArrow(arrow, "Dobot_Rail_rail", 0, 1, 1, 1, 0, "Placement", 5);

    //dobot.moveToPositionWithSafetyMarginZ(position_in_dobot_frame , rotation, linear_rail, 40);
    dobot.moveToPositionWithLBlocking(position_in_dobot_frame, rotation, linear_rail);

    dobot.enableSuctionCup(false);

    dobot.moveToPositionWithLBlocking(dobot::env::R_WAIT_POSITION, 0, 0);

    ROS_INFO_STREAM("Finished TruckBuildPlaceAction");
    result.success = 0;
    action_server.setSucceeded(result);

}

/**
 * TruckStoragePlaceAction
 */

TruckStoragePlaceAction::TruckStoragePlaceAction(std::string name, std::shared_ptr<ros::NodeHandle> handle,
                                                 dobot::Dobot &dob) :
        Action<dobot_msgs::TruckStoragePlaceGoalConstPtr>(name, handle, dob),
        action_server(*handle, name, boost::bind(&TruckStoragePlaceAction::callback, this, _1), false) {
    action_server.start();
    std::cout << "TruckStoragePlaceAction" << std::endl;
}


tf2::Vector3 truck_base_storage(205, 0, -40);
int truck_base_storage_rail = 200;
tf2::Vector3 trailer_base_storage(198, 0, -53);
int trailer_base_storage_rail = 265;
tf2::Vector3 truck_cockpit_storage(220, 0, -15);
int truck_cockpit_storage_rail = 310;
tf2::Vector3 block_1_storage(220, 0, -18);
int block_1_storage_rail = 370;
tf2::Vector3 block_2_storage(170, 0, -18);
int block_2_storage_rail = 310;
tf2::Vector3 block_3_storage(170, 0, -18);
int block_3_storage_rail = 370;
tf2::Vector3 storage_waiting_position(200,0,50);
void TruckStoragePlaceAction::callback(const dobot_msgs::TruckStoragePlaceGoalConstPtr &goal) {

    ROS_INFO_STREAM("Started TruckStoragePlaceAction");
    struct dobot::rfid::uid uid = {goal.get()->b0, goal.get()->b1, goal.get()->b2, goal.get()->b3};

    dobot.moveToPositionWithLBlocking(storage_waiting_position, 0, linear_rail_base);

    if (uid == dobot::rfid::block_1_uid) {
        dobot.moveToPositionWithLBlocking(block_1_storage, 0, block_1_storage_rail);
        ROS_INFO_STREAM("MOVE ARM TO " << block_1_storage.x() << "," << block_1_storage.y() << "," << block_1_storage.z());
    } else if (uid == dobot::rfid::block_2_uid) {
        dobot.moveToPositionWithLBlocking(block_2_storage, 0, block_2_storage_rail);
        ROS_INFO_STREAM("MOVE ARM TO " << block_2_storage.x() << "," << block_2_storage.y() << "," << block_2_storage.z());
    } else if (uid == dobot::rfid::block_3_uid) {
        ROS_INFO_STREAM("MOVE ARM TO " << block_3_storage.x() << "," << block_3_storage.y() << "," << block_3_storage.z());
        dobot.moveToPositionWithLBlocking(block_3_storage, 0, block_3_storage_rail);
    } else if (uid == dobot::rfid::truck_base_uid) {
        dobot.moveToPositionWithLBlocking(truck_base_storage, 90, truck_base_storage_rail);
        ROS_INFO_STREAM("MOVE ARM TO " << truck_base_storage.x() << "," << truck_base_storage.y() << "," << truck_base_storage.z());
    } else if (uid == dobot::rfid::trailer_base_uid) {
        dobot.moveToPositionWithLBlocking(trailer_base_storage, 90, trailer_base_storage_rail);
        ROS_INFO_STREAM("MOVE ARM TO " << trailer_base_storage.x() << "," << trailer_base_storage.y() << "," << trailer_base_storage.z());
    } else if (uid == dobot::rfid::truck_cockpit_uid) {
        dobot.moveToPositionWithLBlocking(truck_cockpit_storage, 0, truck_cockpit_storage_rail);
        ROS_INFO_STREAM("MOVE ARM TO " << truck_cockpit_storage.x() << "," << truck_cockpit_storage.y() << "," << truck_cockpit_storage.z());
    }

    dobot.enableSuctionCup(false);

    dobot.moveToPositionWithLBlocking(dobot::env::R_WAIT_POSITION, 0, 0);

    ROS_INFO_STREAM("Finished TruckStoragePlaceAction");
    action_server.setSucceeded(result);

}

/**
 * TruckStorageTakeAction
 */
 TruckStorageTakeAction::TruckStorageTakeAction(std::string name, std::shared_ptr<ros::NodeHandle> handle,
                                                dobot::Dobot &dob) :
         Action<dobot_msgs::TruckStorageTakeGoalConstPtr>(name, handle, dob),
         action_server(*handle, name, boost::bind(&TruckStorageTakeAction::callback, this, _1), false) {
    action_server.start();
    std::cout << "TruckStorageTakeAction" << std::endl;
}

void TruckStorageTakeAction::callback(const dobot_msgs::TruckStorageTakeGoalConstPtr &goal) {
    ROS_INFO_STREAM("Started TruckStorageTakeAction");
    struct dobot::rfid::uid uid = {goal.get()->b0, goal.get()->b1, goal.get()->b2, goal.get()->b3};

    dobot.moveToPositionWithLBlocking(storage_waiting_position, 0, linear_rail_base);


    if (uid == dobot::rfid::block_1_uid) {
        dobot.moveToPositionWithSafetyMarginZ(block_1_storage, 0, block_1_storage_rail,20);
        ROS_INFO_STREAM("MOVE ARM TO " << block_1_storage.x() << "," << block_1_storage.y() << "," << block_1_storage.z());
    } else if (uid == dobot::rfid::block_2_uid) {
        dobot.moveToPositionWithSafetyMarginZ(block_2_storage, 0, block_2_storage_rail,20);
        ROS_INFO_STREAM("MOVE ARM TO " << block_2_storage.x() << "," << block_2_storage.y() << "," << block_2_storage.z());
    } else if (uid == dobot::rfid::block_3_uid) {
        ROS_INFO_STREAM("MOVE ARM TO " << block_3_storage.x() << "," << block_3_storage.y() << "," << block_3_storage.z());
        dobot.moveToPositionWithSafetyMarginZ(block_3_storage, 0, block_3_storage_rail,20);
    } else if (uid == dobot::rfid::truck_base_uid) {
        dobot.moveToPositionWithSafetyMarginZ(truck_base_storage, -90, truck_base_storage_rail,20);
        ROS_INFO_STREAM("MOVE ARM TO " << truck_base_storage.x() << "," << truck_base_storage.y() << "," << truck_base_storage.z());
    } else if (uid == dobot::rfid::trailer_base_uid) {
        dobot.moveToPositionWithSafetyMarginZ(trailer_base_storage, -90, trailer_base_storage_rail,20);
        ROS_INFO_STREAM("MOVE ARM TO " << trailer_base_storage.x() << "," << trailer_base_storage.y() << "," << trailer_base_storage.z());
    } else if (uid == dobot::rfid::truck_cockpit_uid) {
        dobot.moveToPositionWithSafetyMarginZ(truck_cockpit_storage, 0, truck_cockpit_storage_rail,20);
        ROS_INFO_STREAM("MOVE ARM TO " << truck_cockpit_storage.x() << "," << truck_cockpit_storage.y() << "," << truck_cockpit_storage.z());
    }

    dobot.enableSuctionCup(true);
    dobot.moveToPositionWithLBlocking(storage_waiting_position, 0, linear_rail_base);


    ROS_INFO_STREAM("Finished TruckTakePlaceAction");
    action_server.setSucceeded(result);
 }
