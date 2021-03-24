//
// Created by lars on 12.03.19.
//

#ifndef PROJECT_DOBOT_H
#define PROJECT_DOBOT_H

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <memory>

#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <detection_msgs/DetectedObject.h>
#include <visualization_msgs/Marker.h>

#include <dobot_msgs/SetPTPWithLCmd.h>
#include <dobot_msgs/GetPose.h>
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
#include "dobot_msgs/GetPoseL.h"
#include "dobot_msgs/SetEndEffectorSuctionCup.h"

struct Storage
{
	tf2::Vector3 start;
	double rotation;
	int maxNumberBlocksX;
	int maxNumberBlocksY;
	int itemCount;
	std::string name;

	Storage(tf2::Vector3 startPosition, int blocksX, int blocksY, std::string storageName, double zRotation = 0.0) : start(startPosition), rotation(zRotation),
			maxNumberBlocksX(blocksX), maxNumberBlocksY(blocksY), name(storageName)
	{
		itemCount = 0;
	}

	tf2::Vector3 getItemPosition(int id, const tf2::Vector3 BLOCK_SIZE)
	{
		int blocksPerLayer = maxNumberBlocksX * maxNumberBlocksY; //How many blocks can be put on one layer?
		int z = id / blocksPerLayer; //On which floor do we have to put this block?
		id = id % blocksPerLayer;	 //Which position on the current floor do we choose?

		tf2::Vector3 offset = tf2::Vector3((int)(id/maxNumberBlocksY),-id%maxNumberBlocksY,z); //Offset measured in number of blocks
		offset = offset * BLOCK_SIZE;							 //Convert offset from number of blocks to millimeter

		//ROTATION:
		tf2::Vector3 offsetRotated;
		offsetRotated.setX((offset.x() * cos(rotation)) -(offset.y() * sin(rotation)));
		offsetRotated.setY((offset.x() * sin(rotation)) +(offset.y() * cos(rotation)));
		offsetRotated.setZ(offset.z());
		ROS_INFO("Offset: [%f, %f, %f]",offsetRotated.x(),offsetRotated.y(),offsetRotated.z());

		return (start + offsetRotated);						 //Convert offset to LOAD_POSITION coordinates
	}
};

//TODO which namespace?
struct Object
{
	//Pose = grasp point
	tf2::Transform pose;
	//Smallest point on x axis up to greatest point on x axis
	double length = 0.0;
	//Smallest point on y axis up to greatest point on y axis
	double width = 0.0;
	//From z=0 to the smallest point on the z axis. Because pose is grasp point, there is no higher point than z=0
	double height = 0.0;
	//Name of the objects type e.g. cube
	std::string type = "UNKNOWN";
	//Average color of the object
	double color[3] = {0.0,0.0,0.0};
	//Chance that this object is identified correctly
	double probability = 0.0;
	//Moment in time when the pose was updated last
	ros::Time timestamp;
	Object()
	{

	}
	/*Conversion from detectedObject to object*/
	Object(detection_msgs::DetectedObject obj)
	{
		pose.setOrigin(tf2::Vector3(obj.graspPoint.x,obj.graspPoint.y,obj.graspPoint.z));
		pose.setRotation(tf2::Quaternion(obj.rotation.x,obj.rotation.y,obj.rotation.z,obj.rotation.w));
		length = std::fabs(obj.endPoint.x - obj.startPoint.x);
		width = std::fabs(obj.endPoint.y - obj.startPoint.y);
		height = std::fabs(obj.endPoint.z - obj.startPoint.z);
		type = obj.type;
		if(obj.color.size() == 3)
		{
			color[0] = obj.color.at(0);
			color[1] = obj.color.at(1);
			color[2] = obj.color.at(2);
		}
		probability = obj.probability;
		timestamp = obj.timestamp;
	}
	/*Conversion from object to detectedObject*/
	detection_msgs::DetectedObject toDetectedObject()
	{
		detection_msgs::DetectedObject obj;
		obj.graspPoint.x = pose.getOrigin().x();
		obj.graspPoint.y = pose.getOrigin().y();
		obj.graspPoint.z = pose.getOrigin().z();
		obj.startPoint.x = obj.graspPoint.x - (length/2.0);
		obj.startPoint.y = obj.graspPoint.y - (width/2.0);
		obj.startPoint.z = obj.graspPoint.z - height;
		obj.endPoint.x = obj.graspPoint.x + (length/2.0);
		obj.endPoint.y = obj.graspPoint.y + (width/2.0);
		obj.endPoint.z = obj.graspPoint.z;

		obj.rotation.x = pose.getRotation().x();
		obj.rotation.y = pose.getRotation().y();
		obj.rotation.z = pose.getRotation().z();
		obj.rotation.w = pose.getRotation().w();

		obj.probability = probability;
		obj.color.push_back(color[0]);
		obj.color.push_back(color[1]);
		obj.color.push_back(color[2]);
		obj.timestamp = timestamp;
		obj.type = type;
		return obj;
	}
};

/**
 * This class should offer all usually needed functions to command the dobot, to remove redundancy in the code.
 * */
namespace dobot
{
    namespace rfid{
        struct uid {
            uint8_t b0, b1, b2, b3;

            uid(){};
            uid(uint8_t u0, uint8_t u1, uint8_t u2, uint8_t u3) : b0(u0), b1(u1), b2(u2), b3(u3) {};

            bool operator==(const uid &other) {
                return b0 == other.b0 && b1 == other.b1 && b2 == other.b2 && b3 == other.b3;
            };
        };

        const struct uid block_1_uid = {4, 187, 16, 66};
        const struct uid block_2_uid = {4, 216, 16, 66};
        const struct uid block_3_uid = {4, 248, 16, 66};
        const struct uid truck_base_uid = {4, 174, 18, 66};
        const struct uid truck_cockpit_uid = {4, 145, 18, 66};
        const struct uid trailer_base_uid = {4, 24, 16, 66};
    };

    enum PtpMode{
    	JUMP_XYZ=0,
		MOVJ_XYZ=1,
		MOVL_XYZ=2,
		JUMP_ANGLE=3,
		MOVJ_ANGLE=4,
		MOVL_ANGLE=5,
		MOVJ_INC=6,
		MOVL_INC=7,
		MOVJ_XYZ_INC=8,
		JUMP_MOVL_XYZ=9
    };

	enum BlockColor{
		UNKNOWN,
		RED,
		GREEN,
		BLUE,
		YELLOW,
		MAGENTA,
		CYAN
	};

	/**
	 * namespace for all dobot names
	 */
	namespace dobot_names
    {
	    const std::string DOBOT_LOADER = "Dobot_Loader";
	    const std::string DOBOT_RAIL = "Dobot_Rail";
	    const std::string DOBOT_LAIR = "Dobot_Lair";
    }

	/**
 	* namespace for the dobot environment constants
 	*/
	namespace env
	{
		//pickup position for right dobot
		const tf2::Vector3 R_PICKUP_POSITION = tf2::Vector3(201, 17.7, -55.0);

		const tf2::Vector3 R_PICKUP_Z_0_OFFSET = tf2::Vector3(0,0,54.0);

		//pickup position for left dobot
		const tf2::Vector3 L_PICKUP_POSITION(131.4, 168.4, -45.2);

		//drop position of left dobot
		const tf2::Vector3 L_DROP_POSITION(207.4,20.0,0.0);


		//position for scanning the color of one block
		const tf2::Vector3 SCAN_COLOR_POSITION = tf2::Vector3(120, -15.25,-50.4);

		// position for scanning the rfid tag of a block
		const tf2::Vector3 SCAN_RFID_POSITION = tf2::Vector3(77.2, 125.5, -55);

		//size of one block in storage
		const tf2::Vector3 R_BLOCK_SIZE(32, 32, 25);

		//size of one block to be loaded
		const tf2::Vector3 L_BLOCK_SIZE(25.0,25.0,25.0);

		// wait position for rail dobot
		const tf2::Vector3 R_WAIT_POSITION(77 , 125, 50);

		//wait position for rail dobot when using picture scenario
		const tf2::Vector3 R_PICTURE_WAIT(200,-150,0);

        // lkw pickup position for rail dobot
        const tf2::Vector3 R_LKW_PICKUP_POSITION(200, 30, -30);

        const tf2::Vector3 LKW_BRICK(243, -0.1, -51.23);

        //difference of the two distance sensors in mm
		const tf2::Vector3 DIST_SENSOR_GAP(0,50.0,0);

		//conveyorbelt speed in mm/s at 15% speed
		const double CONVEYORBELT_SPEED = 35.4103;

		//normal speed in percent
        const int8_t CONVEYORBELT_SPEED_IN_PERCENT = 30;

        //measurement speed in percent
        const int8_t CONVEYORBELT_MEASUREMENT_SPEED_IN_PERCENT = 1;

        //measurement speed in mm/s at 1% speed
        const double CONVEYORBELT_MEASUREMENT_SPEED = 2.3607;

		const double MAX_BLOCK_DEVIATION = 25 / std::sqrt(2);

		//Offset from dobot endeffector to exact tool position in mm.
		const double ARM_EXTENSION = 65.5;
	}

    /**
     * Converts an rgb color into a BlockColor.
     *
     * @param r red value of rgb color
     * @param g green value of rgb color
     * @param b blue value of rgb color
     * @return the rgb color represented as block color
     */
    BlockColor convert_rgb_to_block_color(const uint8_t r, const uint8_t g, const uint8_t b);


	class Dobot {
	public:
	    Dobot(std::string dobot_name);
	    Dobot(const Dobot& d);
        /**
         * Will update robotIsIdle variable.
         */
        void robotStateCallback(const std_msgs::Bool::ConstPtr &msg);

        /**
         * Will update the armPosition variable
         */
        void robotPoseCallback(const geometry_msgs::Pose::ConstPtr &pose);

        /**
         * Will update the toolCollision variable
         */
         void toolCollisionCallback(const std_msgs::Bool::ConstPtr &msg);

        /**
         * Stops the current operation. Not implemented //TODO
         */
        void abortMovement();

        /**
         * Checks if the dobot is idle/not moving.
         * @return returns if dobot is not moving.
         */
        bool isIdle(); //TODO is currently ignoring linear rail

        /**
         * Will call the suction cup service and enables/disables the suction cup.
         * @param suck If set to true, the suction cup will suck items.
         */
        void enableSuctionCup(bool suck);

        /**
       * Checks if the robot is at a given position.
       * @param target Position where the arm could be located.
       * @return Is true, if the arm is next to the target position.
       */
        bool isAtPosition(tf2::Vector3 target);

        /**
         * Checks if the robot is at a given position.
         * @param target Position where the arm could be located.
         * @return Is true, if the arm is next to the target position.
         */
        bool isAtPosition(tf2::Vector3 target, double rotation);

		/**
         * Moves the robot arm to a position without using linear rail.
         * @param pos Target position of the arm, relative to the arm base.
         * @param r Rotation of the tool in degree.
         */
		void moveArmToPosition(tf2::Vector3 pos, double r, size_t ptpMode=0);

		void moveArmToPositionBlocking(tf2::Vector3 pos, double r, size_t ptpMode=0);

        /**
         * Moves the robot arm to a position using linear rail.
         * @param pos Target position of the arm, relative to the arm base.
         * @param r Rotation of the tool in degree.
         * @param l Position on the linear rail. 0 == left. 1000 == right.
         */
		void moveArmToPositionWithL(tf2::Vector3 pos, double r, double l, size_t ptpMode=0);

        /**
         *  Moves the robot arm to a position. Blocks until finished.
         * @param position Target position of the arm, relative to the arm base.
         * @param r Rotation of the tool in degree.
         * @param l Position on the linear rail. 0 == left. 1000 == right.
         */
        void moveToPositionWithLBlocking(const tf2::Vector3 position, double r, double l);

        /**
       *  Moves the robot arm to a position. Blocks until finished.
       * @param position Target position of the arm, relative to the arm base.
       * @param r Rotation of the tool in degree.
       */
        void moveToPositionBlocking(const tf2::Vector3 position, double r);

        /**
         * Moves the robot arm down until the tool collides with something. Only possible with dobot rail!
         * @param r Rotation of the tool in degree.
         * @param safety_margin Maximal distance to go down stepwise.
         */
        void moveDownUntilToolCollision(double r, double safety_margin);

        /**
         * Moves arm to the position pos  To z coordinate the safety margin divided by 2 is added. Then moves down until the tool
         * collides with something or the safety margin is exceeded.
         * @param pos  position without safety margin
         * @param r Rotation of the tool in degree.
         * @param l Position on the linear rail. 0 == left. 1000 == right.
         * @param safety_margin Distance to pos in millimeters.
         */
        void moveToPositionWithSafetyMarginZ(tf2::Vector3 pos, double r, double l, double safety_margin);

        /**
         * Convert target position in storage to rail and arm coordinates.
         * @param goal Target position in storage.
         * @param out Arm position to reach block position in storage.
         * @param lOut Robot position on linear rail to reac storage.
         * @return returns true if success
         */
        bool positionToPositionWithL(const tf2::Vector3 goal, tf2::Vector3 &out, double &lOut);


        /**
         * Initializes this class.
         * @param nh_ptr Node which can be used for sending messages, using services.
         * @param robotName Robot prefix which has to be targeted with all commands.
         * @return Will return true, if dobot can be reached and everything has been succesfull.
         */
        bool initArm(std::shared_ptr<ros::NodeHandle> nh_ptr, std::string robotName, double jumpHeightInMM=50.0);


        /**
         * Checks if initArm() has been succesfully called.
         * @return success
         */
        bool isInitialized();

        /**
         * Converts points from any frame into the local dobot frame. DO NOT USE WITH LINEAR RAIL!!! Output coordinates in mm, input coordinates in m.
         * @param graspPoint is the point to grasp.
         * @param transformedGraspPoint is the point in dobot coordinates.
         * @param inputFrame is the frame of the graspPoint.
         * @return Will return true if conversion has been succesfull.
         */
        bool convertToDobotPoint(const tf2::Vector3 graspPoint, tf2::Vector3& transformedGraspPoint, const std::string inputFrame="world");
        /**
		 * Converts points from any frame into the local dobot frame. Output coordinates in mm, input coordinates in m.
		 * transformdGraspPose is only ment for arm, railPos is only ment to be used for rail!
		 * @param graspPoint is the point to grasp.
		 * @param transformedGraspPoint is the point in dobot coordinates.
		 * @param inputFrame is the frame of the graspPoint.
		 * @return Will return true if conversion has been succesfull.
		 */
        bool convertToDobotPointWithL(const tf2::Vector3 graspPoint, tf2::Vector3& transformedGraspPoint, double &railPos, const std::string inputFrame);
        /**
		 * Converts a pose from any frame into the local dobot frame. DO NOT USE WITH LINEAR RAIL!!! Output coordinates in mm, input coordinates in m. Rotation in rad.
		 * @param graspPose is the point+rotation to grasp.
		 * @param transformedGraspPose is the point+rotation in dobot coordinates.
		 * @param inputFrame is the frame of the graspPose.
		 * @return Will return true if conversion has been succesfull.
		 */
        bool convertToDobotPose(tf2::Transform graspPose, tf2::Transform& transformedGraspPose, const std::string inputFrame);
        /**
		 * Converts a pose from any frame into the local dobot frame. Output coordinates in mm, input coordinates in m. Rotation in rad.
		 * transformdGraspPose is only ment for arm, railPos is only ment to be used for rail!
		 * @param graspPose is the point+rotation to grasp.
		 * @param transformedGraspPose is the point+rotation in dobot coordinates.
		 * @param inputFrame is the frame of the graspPose.
		 * @return Will return true if conversion has been succesfull.
		 */
        bool convertToDobotPoseWithL(const tf2::Transform graspPose, tf2::Transform& transformedGraspPose, double &railPos, const std::string inputFrame);

        /**
		 * @return Returns true if this dobot has a linear rail attached.
		 */
        bool hasLinearRail();

	private:
        const double ALLOWED_ERROR = 1.0;    /**< Allowed error in millimeters. Used by "isAtPosition" function. */
        std::string ROBOT_NAME = dobot_names::DOBOT_RAIL;
        std::shared_ptr<ros::NodeHandle> nh;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        tf2_ros::Buffer tfBuffer;
        tf2::Vector3 armPosition = tf2::Vector3(0, 0, 0);
        bool robotIsIdle = false;
        bool initSuccess = false;
        bool linearRail = false;

        ros::ServiceClient ptp_init_client;
        ros::ServiceClient ptp_cmd_client;
        ros::ServiceClient end_effector_client;
        ros::ServiceClient ptpL_cmd_client;
        ros::ServiceClient robotRotation;
        ros::Subscriber robotStateSub;
        ros::Subscriber robotArmPoseSub;
        ros::Subscriber toolCollisionSub;

        bool toolCollision;
    };
}

#endif /* PROJECT_DOBOT_H */
