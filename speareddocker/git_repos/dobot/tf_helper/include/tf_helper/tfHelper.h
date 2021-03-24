/*
 * tf_helper.h
 *
 *  Created on: May 20, 2019
 *      Author: philipf
 */

#ifndef SRC_DOBOT_DOBOT_PROGRAMS_INCLUDE_LIBRARIES_TFHELPER_H_
#define SRC_DOBOT_DOBOT_PROGRAMS_INCLUDE_LIBRARIES_TFHELPER_H_

#include <ros/ros.h>
#include <memory>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>

class TfHelper {

	public:
		TfHelper();
		/**
		 * Transforms a vector from the source to the target coordinate frame.
		 * If no transform is available, the function will wait up to 3 seconds until it returns false.
		 * @param time The nearest transform to the given time will be picked. If time=ros::Time(0), any available transform will be choosen.
		 * @return If an exception has happened, false will be returned.
		 */
		bool transformVector(const std::string source_frame, const std::string target_frame, const ros::Time time, tf2::Vector3 &vec);
		/**
		 * Transforms a quaternion from the source to the target coordinate frame.
		 * If no transform is available, the function will wait up to 3 seconds until it returns false.
		 * @param time The nearest transform to the given time will be picked. If time=ros::Time(0), any available transform will be choosen.
		 * @return If an exception has happened, false will be returned.
		 */
		bool transformQuaternion(const std::string source_frame, const std::string target_frame, const ros::Time time, tf2::Quaternion &rot);
		/**
		 * Transforms a transform from the source to the target coordinate frame.
		 * If no transform is available, the function will wait up to 3 seconds until it returns false.
		 * @param time The nearest transform to the given time will be picked. If time=ros::Time(0), any available transform will be choosen.
		 * @return If an exception has happened, false will be returned.
		 */
		bool transformTransform(const std::string source_frame, const std::string target_frame, const ros::Time time, tf2::Transform &transform);
		/**
		 * Returns the fransform from source to target. Use this function if the transform has to be reused multiple times.
		 * If no transform is available, the function will wait up to 3 seconds until it returns false.
		 * @param time The nearest transform to the given time will be picked. If time=ros::Time(0), any available transform will be choosen.
		 * @return If an exception has happened, false will be returned.
		 */
		bool getTransform(const std::string source_frame, const std::string target_frame, const ros::Time time, tf2::Transform &transform);

		static geometry_msgs::TransformStamped toTransformStamped(tf2::Transform transform, ros::Time time, std::string targetFrame, std::string childFrame)
		{
			geometry_msgs::TransformStamped out;
			out.child_frame_id = childFrame;
			out.header.frame_id = targetFrame;
			out.header.stamp = time;
			out.transform.translation.x = transform.getOrigin().x();
			out.transform.translation.y = transform.getOrigin().y();
			out.transform.translation.z = transform.getOrigin().z();
			out.transform.rotation.x = transform.getRotation().x();
			out.transform.rotation.y = transform.getRotation().y();
			out.transform.rotation.z = transform.getRotation().z();
			out.transform.rotation.w = transform.getRotation().w();
			return out;
		}
		static tf2::Transform fromTransformStamped(geometry_msgs::TransformStamped transform)
		{
			tf2::Transform out;
			out.setRotation(tf2::Quaternion(transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w));
			out.setOrigin(tf2::Vector3(transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z));
			return out;
		}

	private:
		tf2_ros::Buffer tfBuffer;
		std::shared_ptr<tf2_ros::TransformListener> listener;
};



#endif /* SRC_DOBOT_DOBOT_PROGRAMS_INCLUDE_LIBRARIES_TFHELPER_H_ */
