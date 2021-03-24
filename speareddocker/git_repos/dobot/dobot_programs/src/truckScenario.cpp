//
// Created by speeters on 29.04.19.
//

#include <actionlib/client/simple_action_client.h>
#include <dobot_msgs/ScanRFIDAction.h>
#include <dobot_msgs/TruckBuildAction.h>
#include <dobot_msgs/TruckStoragePlaceAction.h>
#include <dobot_msgs/TruckStorageTakeAction.h>
#include <dobot_msgs/PickupAction.h>

#include <detection/detection.h>
#include <detection_msgs/DetectedObject.h>

#include <tf_helper/tfHelper.h>
#include <queue>
#include <condition_variable>
#include <mutex>
#include <detection_msgs/GetDetectedObject.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <arduino/arduino.h>
#include <dobot/dobot.h>



/**
 * Variables that indicate if the truck base and the trailer base are placed on the grid. true --> block is on grid
 */
bool truck_base_state = false;
bool trailer_base_state = false;

/**
 * Variables that indicate which blocks are in the storage. true --> block is in storage
 */
bool truck_base_in_storage = false;
bool trailer_base_in_storage = false;
bool truck_cockpit_in_storage = false;
bool block_1_in_storage = false;
bool block_2_in_storage = false;
bool block_3_in_storage = false;

/**
 * Variables for the ros callbacks
 */
bool conveyor_blocked = false;

std::condition_variable conveyor;
std::mutex m;

/**
 * Tfhelper to transform vector between frames
 */
std::shared_ptr<TfHelper> tfHelper;

/**
 * Function definitions
 */
bool isPlacableOnGrid(dobot::rfid::uid uid);
bool fromStoragePlaceable(dobot::rfid::uid &uid);
void placeOnGrid(dobot::rfid::uid uid);
void placeInStorage(dobot::rfid::uid uid);
void takeFromStorage(dobot::rfid::uid uid);
dobot::rfid::uid scanRFID(double block_height,  actionlib::SimpleActionClient<dobot_msgs::ScanRFIDAction> &client);
double pickFromConveyorBelt(dobot::Dobot &dob, detection::RailGraspObjectGetter &railGraspObjectGetter);



/**
 * Callback for the infrared sensor of the conveyor belt.
 * @param obj  true if conveyor belt is blocked
 */

bool conveyor_dirty = false;

void conveyorCallback(const std_msgs::Bool::ConstPtr &obj) {
    conveyor_blocked = obj->data;
    if(conveyor_blocked){
        conveyor.notify_all();
    } else {
        conveyor_dirty = true;
    }
}



/**
 * Truck Scenario
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char *argv[]) {

    ros::init(argc, argv, "truck_scenario");

    ros::NodeHandle nh;
    std::shared_ptr<ros::NodeHandle> handle = std::make_shared<ros::NodeHandle>(nh);

    dobot::Dobot dob(dobot::dobot_names::DOBOT_RAIL);
    dob.initArm(handle, dobot::dobot_names::DOBOT_RAIL);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Subscriber conveyorSub = handle->subscribe<std_msgs::Bool>("/arduino_conveyor/GetInfraredSensorBlocked", 1,
                                                                    conveyorCallback);
    ros::ServiceClient blockqueueClient = handle->serviceClient<detection_msgs::GetDetectedObject>(
            "/transportbelt/popObject");

    actionlib::SimpleActionClient<dobot_msgs::ScanRFIDAction> rfid_client("scanRFID", true);

    detection::RailGraspObjectGetter rgog(handle);

    while (ros::ok()) {

        ROS_INFO_STREAM("Next round!");
        dobot::rfid::uid uid;
        if (fromStoragePlaceable(uid)) {
            ROS_INFO_STREAM("Block from Storage is placeable");
            //take block out of storage
            takeFromStorage(uid);
            // pace on grid
            placeOnGrid(uid);
        } else {
            ROS_INFO_STREAM("Grasp block from conveyor belt");
            //grasp block from conveyor belt and scan uid
            double  block_height = pickFromConveyorBelt(dob, rgog);
            block_height *= 1000 + 50;
            dobot::rfid::uid scanned_uid = scanRFID(block_height, rfid_client);

            // scan rfid and place on grid or in storage
            dobot::rfid::uid zero_uid(0,0,0,0);
            int tries = 1;
            while(scanned_uid == zero_uid && tries <= 3){
                tries++;
                ROS_INFO_STREAM("Zero uid scanned. Rescan try " << tries << " of 3");
                scanned_uid = scanRFID(block_height, rfid_client);
            }

            if(scanned_uid == zero_uid){
                ROS_INFO_STREAM("Zero uid scanned. Drop block to trash.");
                tf2::Vector3 drop_position(113, -166, -110);
                dob.moveToPositionWithLBlocking(drop_position,0,0);
                continue;
            }

            if (isPlacableOnGrid(scanned_uid))
                placeOnGrid(scanned_uid);
            else
                placeInStorage(scanned_uid);
        }
    }

    return 0;
}

/**
 * Uses detection::RailGraspObjectGetter to grasp the rioght most object on the conveyor belt.
 * Blocks until the infrared sensor of the conveyor belt is blocked. Waits then until a object
 * is detected and grasps it.
 */
double pickFromConveyorBelt(dobot::Dobot &dob, detection::RailGraspObjectGetter &railGraspObjectGetter){
    double block_height = 0;
    do{
        std::unique_lock<std::mutex> lock(m);
        while(!conveyor_blocked){
            conveyor.wait(lock);
        }
        while(ros::ok() && railGraspObjectGetter.isEmpty()){
            ROS_INFO_STREAM("Check if there is a Object.");
            ros::Duration(3).sleep();
        }

        double rotation_degree = 0;
        detection_msgs::DetectedObject detectedObject;

        int cnt = 0;
        tf2::Vector3 gp;
        detectedObject = railGraspObjectGetter.getRightMostObject(ros::Time(0), 0.9);
        do{
            ROS_INFO_STREAM("Object detected.");
            detectedObject = railGraspObjectGetter.getRightMostObject(ros::Time(0), 0.9);
            tf2::Vector3 gp_inter;
            tf2::fromMsg(detectedObject.graspPoint,gp_inter);
            geometry_msgs::Quaternion rotation_msg = detectedObject.rotation;
            tf2::Quaternion rotation;
            tf2::fromMsg(rotation_msg, rotation);
            tf2::Matrix3x3 rotation_mat(rotation);
            double roll, pitch, yaw;
            rotation_mat.getRPY(roll, pitch, yaw);
            double rotation_degree_inter =  (yaw * (double) 180 / M_PI) + 180;
            ROS_INFO_STREAM("Detected rotation: " << rotation_degree_inter);
            if(rotation_degree_inter == 180){
                rotation_degree_inter -= 180;
            }
            if(rotation_degree_inter > 180)
                rotation_degree_inter -= 360;
            if(!isnan(rotation_degree)){
                cnt++;
                gp+=gp_inter;
                rotation_degree += rotation_degree_inter;
            }
            ros::Duration(1).sleep();
        } while(cnt < 5);

        gp /= 5;
        rotation_degree /= 5;

        ROS_INFO_STREAM("Rotation of object: " << rotation_degree);
        tf2::Vector3 gp_dobot;

        double linear_rail = 0;
        dob.convertToDobotPointWithL(gp, gp_dobot, linear_rail, railGraspObjectGetter.getFrameID());

        ROS_INFO_STREAM("Pick up Object");
        actionlib::SimpleActionClient<dobot_msgs::PickupAction> gp_pick_up_client("pickup", true);

        gp_pick_up_client.waitForServer();
        dobot_msgs::PickupGoal pick_goal;

        ROS_INFO_STREAM("Pick up at: \nx:" << gp_dobot.x() <<"\ny:"<<  gp_dobot.y() <<",\nz:" << gp_dobot.z() << ",\nr:" << rotation_degree);

        pick_goal.position.x = gp_dobot.x();
        pick_goal.position.y = gp_dobot.y();
        pick_goal.position.z = gp_dobot.z() - 10;
        pick_goal.rotation = rotation_degree;

        gp_pick_up_client.sendGoal(pick_goal);
        ROS_INFO_STREAM("Send goal");
        gp_pick_up_client.waitForResult();

        ROS_INFO_STREAM("Block height: " << (detectedObject.graspPoint.z - detectedObject.startPoint.z));
        block_height = detectedObject.graspPoint.z - detectedObject.startPoint.z;

        dob.moveToPositionWithLBlocking(dobot::env::R_WAIT_POSITION, 0, 0);
        ros::Duration(1).sleep();

        ROS_INFO_STREAM("Picked up:" << conveyor_dirty);
    } while(!conveyor_dirty);

    conveyor_dirty = false;
    return block_height;

}

/**
 * RFID scans the grasped block. Returns uid(0,0,0,0) if nothing was scanned. Blocks until scanning is done. Block height in
 * millimeter is used to calculate the right scanning position.
 * Uses ScanRFIDAction.
 */
dobot::rfid::uid scanRFID(double block_height, actionlib::SimpleActionClient<dobot_msgs::ScanRFIDAction> &client){
    client.waitForServer();

    dobot_msgs::ScanRFIDGoal goal;
    goal.block_height = block_height;
    client.sendGoal(goal);
    client.waitForResult();

    auto res = client.getResult();

    ROS_INFO_STREAM("RFID Scanned UID:" << unsigned(res->b0) << " " << unsigned(res->b1) << " " << unsigned(res->b2) << " "
                                        << unsigned(res->b3));

    dobot::rfid::uid scanned_uid(res->b0, res->b1, res->b2, res->b3);
    return scanned_uid;
}

/**
 * Places the block depending on the UID on the grid. Blocks until block is placed.
 */
void placeOnGrid(dobot::rfid::uid uid) {
    // place on grid
    ROS_INFO_STREAM("Created truck builder action client");
    actionlib::SimpleActionClient<dobot_msgs::TruckBuildAction> truck_builder_client("truckBuild", true);

    truck_builder_client.waitForServer();

    dobot_msgs::TruckBuildGoal truck_builder_goal;
    truck_builder_goal.b0 = uid.b0;
    truck_builder_goal.b1 = uid.b1;
    truck_builder_goal.b2 = uid.b2;
    truck_builder_goal.b3 = uid.b3;

    ROS_INFO_STREAM("Starting TruckBuildAction");
    truck_builder_client.sendGoal(truck_builder_goal);
    truck_builder_client.waitForResult();
    ROS_INFO_STREAM("Finished TruckBuildAction");

    auto res = truck_builder_client.getResult();

    // set grid state accordingly
    if (uid == dobot::rfid::truck_base_uid)
        truck_base_state = true;
    else if (uid == dobot::rfid::trailer_base_uid)
        trailer_base_state = true;
}

/**
 * Places  the block depending on the UID in the storage. Blocks until block is placed.
 */
void placeInStorage(dobot::rfid::uid uid) {
// place in storage
    ROS_INFO_STREAM("Created truck storage place action client");
    actionlib::SimpleActionClient<dobot_msgs::TruckStoragePlaceAction> truck_storage_client("truckStoragePlace", true);

    truck_storage_client.waitForServer();

    dobot_msgs::TruckStoragePlaceGoal goal_truck_storage;
    goal_truck_storage.b0 = uid.b0;
    goal_truck_storage.b1 = uid.b1;
    goal_truck_storage.b2 = uid.b2;
    goal_truck_storage.b3 = uid.b3;

    ROS_INFO_STREAM("Starting TruckStorageAction");
    truck_storage_client.sendGoal(goal_truck_storage);
    truck_storage_client.waitForResult();
    ROS_INFO_STREAM("Finished TruckStorageAction");

    auto res_lkw = truck_storage_client.getResult();

// set storage state accordingly
    if (uid == dobot::rfid::truck_base_uid)
        truck_base_in_storage = true;
    else if (uid == dobot::rfid::trailer_base_uid)
        trailer_base_in_storage = true;
    else if (uid == dobot::rfid::truck_cockpit_uid)
        truck_cockpit_in_storage = true;
    else if (uid == dobot::rfid::block_1_uid)
        block_1_in_storage = true;
    else if (uid == dobot::rfid::block_2_uid)
        block_2_in_storage = true;
    else if (uid == dobot::rfid::block_3_uid)
        block_3_in_storage = true;
}

/**
 * Takes the next placable block form storage and places it depending on the uid on the grid.
 */
void takeFromStorage(dobot::rfid::uid uid) {
// place in storage
    actionlib::SimpleActionClient<dobot_msgs::TruckStorageTakeAction> truck_storage_client("truckStorageTake", true);

    truck_storage_client.waitForServer();

    dobot_msgs::TruckStorageTakeGoal goal_truck_storage;
    goal_truck_storage.b0 = uid.b0;
    goal_truck_storage.b1 = uid.b1;
    goal_truck_storage.b2 = uid.b2;
    goal_truck_storage.b3 = uid.b3;

    ROS_INFO_STREAM("Starting TruckStorageTakeAction");
    truck_storage_client.sendGoal(goal_truck_storage);
    truck_storage_client.waitForResult();
    ROS_INFO_STREAM("Finished TruckStorageTakeAction");

    auto res_lkw = truck_storage_client.getResult();

// set storage state accordingly
    if (uid == dobot::rfid::truck_base_uid) {
        truck_base_in_storage = false;
        ROS_INFO_STREAM("TOOK TRUCK BASE");
    } else if (uid == dobot::rfid::trailer_base_uid) {
        trailer_base_in_storage = false;
        ROS_INFO_STREAM("TOOK TRAILER BASE");
    } else if (uid == dobot::rfid::truck_cockpit_uid) {
        truck_cockpit_in_storage = false;
        ROS_INFO_STREAM("TOOK TRUCK COCKPIT");
    } else if (uid == dobot::rfid::block_1_uid) {
        block_1_in_storage = false;
        ROS_INFO_STREAM("TOOK BLOCK 1");
    } else if (uid == dobot::rfid::block_2_uid) {
        block_2_in_storage = false;
        ROS_INFO_STREAM("TOOK BLOCK 2");
    } else if (uid == dobot::rfid::block_3_uid) {
        ROS_INFO_STREAM("TOOK BLOCK 3");
        block_3_in_storage = false;
    }

}

/**
 * Returns true is the block with uid UID is palcable on the grid.
 */
bool isPlacableOnGrid(dobot::rfid::uid uid) {
    if (uid == dobot::rfid::truck_base_uid) {
        return true;
    } else if (uid == dobot::rfid::truck_cockpit_uid || uid == dobot::rfid::trailer_base_uid) {
        return truck_base_state;
    } else
        return trailer_base_state;
}

/**
 * Returns true if a block form the storage is placeable. Corresponding uid is saved uid.
 */
bool fromStoragePlaceable(dobot::rfid::uid &uid) {
    if (truck_base_in_storage) {
        uid = dobot::rfid::truck_base_uid;
        return true;
    } else if (truck_base_state) {
        if (trailer_base_in_storage) {
            uid = dobot::rfid::trailer_base_uid;
            return true;
        } else if (truck_cockpit_in_storage) {
            uid = dobot::rfid::truck_cockpit_uid;
            return true;
        } else if (trailer_base_state) {
            if (block_1_in_storage) {
                uid = dobot::rfid::block_1_uid;
                return true;
            } else if (block_2_in_storage) {
                uid = dobot::rfid::block_2_uid;
                return true;
            } else if (block_3_in_storage) {
                uid = dobot::rfid::block_3_uid;
                return true;
            }
        }
    }
    return false;
}
