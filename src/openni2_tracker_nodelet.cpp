/*
* Copyright (c) 2013, Marcus Liebhardt, Yujin Robot.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Yujin Robot nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * Inspired by the openni_tracker by Tim Field and PrimeSense's NiTE 2.0 - Simple Skeleton Sample
 */
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

//#include <OpenNI.h>
//#include <XnCodecIDs.h>
//#include <XnCppWrapper.h>
#include <NiTE.h>

#include <openni2_tracker/openni2_tracker.h>

PLUGINLIB_EXPORT_CLASS(openni2_tracker::OpenNI2TrackerNodelet, nodelet::Nodelet)

namespace openni2_tracker {

OpenNI2TrackerNodelet::OpenNI2TrackerNodelet() : max_users_(10) {}

OpenNI2TrackerNodelet::~OpenNI2TrackerNodelet() { nite::NiTE::shutdown(); }

using std::string;

#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

void OpenNI2TrackerNodelet::onInit() {

	for (int i = 0; i < max_users_; ++i) {
		g_visibleUsers.push_back(false);
		g_skeletonStates.push_back(nite::SKELETON_NONE);
	}

	nh_.reset(new ros::NodeHandle(getMTNodeHandle()));
	pnh_.reset(new ros::NodeHandle(getMTPrivateNodeHandle()));
	it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

	frame_id_ = "openni_depth_frame";
	pnh_->getParam("camera_frame_id", frame_id_);
	if (!pnh_->getParam("device_id", device_id_)) {
		device_id_ = "#1";
	}
	NODELET_INFO("device_id: %s", device_id_.c_str());

	nh_->param<double>("publish_period", publish_period_, 33);
	publish_period_ = publish_period_ / 1000.0;
	publish_timer_ = nh_->createTimer(
									ros::Duration(publish_period_),
									boost::bind(&OpenNI2TrackerNodelet::timeCallback, this, _1));

	bool is_standalone;
	if (pnh_->getParam("is_standalone", is_standalone) && is_standalone) {
		device_initialization();
	} else {
		NODELET_INFO("waiting for %s to initialize OpenNI", nh_->resolveName("image").c_str());
		depth_img_sub_ = it_->subscribe("image", 1, &OpenNI2TrackerNodelet::imageCallback, this);
	}

}

void OpenNI2TrackerNodelet::device_initialization() {
	boost::mutex::scoped_lock lock(mutex_);
	if (openni::OpenNI::initialize() != openni::STATUS_OK) {
		NODELET_FATAL("OpenNI initial error");
		return;
	}

	openni::Array<openni::DeviceInfo> deviceInfoList;
	openni::OpenNI::enumerateDevices(&deviceInfoList);

	if (device_id_.size() == 0 || device_id_ == "#1") {
		device_id_ = deviceInfoList[0].getUri();
	}

	if (devDevice_.open(device_id_.c_str()) != openni::STATUS_OK) {
		NODELET_FATAL("Couldn't open device: %s", device_id_.c_str());
		return;
	}

	userTrackerFrame_.reset(new nite::UserTrackerFrameRef);
	userTracker_.reset(new nite::UserTracker);
	nite::NiTE::initialize();

	niteRc_ = userTracker_->create(&devDevice_);
	if (niteRc_ != nite::STATUS_OK) {
		NODELET_FATAL("Couldn't create user tracker");
	return;
	}
	NODELET_INFO("OpenNI userTracker initialized");
}

void OpenNI2TrackerNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	depth_img_sub_.shutdown();
	device_initialization();
}

void OpenNI2TrackerNodelet::timeCallback(const ros::TimerEvent) {
	boost::mutex::scoped_lock lock(mutex_);
	if (!devDevice_.isValid())
		return;
	niteRc_ = userTracker_->readFrame(&(*userTrackerFrame_));
	if (niteRc_ != nite::STATUS_OK) {
		NODELET_WARN("Get next frame failed.");
		return;
	}

	const nite::Array<nite::UserData> &users = userTrackerFrame_->getUsers();
	for (int i = 0; i < users.getSize(); ++i) {
		const nite::UserData &user = users[i];
		updateUserState(user, userTrackerFrame_->getTimestamp());
		if (user.isNew()) {
			userTracker_->startSkeletonTracking(user.getId());
			NODELET_INFO("Found a new user.");
		} else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED) {
			NODELET_INFO("Now tracking user %d", user.getId());
		}
	}
	publishTransforms(frame_id_, users);
}

void OpenNI2TrackerNodelet::publishTransform(nite::UserData const& user, nite::JointType const& joint, string const& frame_id, string const& child_frame_id) {
	static tf::TransformBroadcaster br;

	nite::SkeletonJoint joint_position = user.getSkeleton().getJoint(joint);
	double x = joint_position.getPosition().x / 1000.0;
	double y = joint_position.getPosition().z / 1000.0;
	double z = joint_position.getPosition().y / 1000.0;

	double qx = joint_position.getOrientation().x;
	double qy = joint_position.getOrientation().z;
	double qz = -joint_position.getOrientation().y;
	double qw = joint_position.getOrientation().w;

	char child_frame_no[128];
	snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user.getId());

	if (qx == 0 && qy == 0 && qz == 0) {
		printf("%s has no Orientation: %f %f %f %f. Using default.\n", child_frame_no, qx, qy, qz, qw);
		qw = 1.0;
	}

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, z));
	transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

	tf::Transform change_frame;
	change_frame.setOrigin(tf::Vector3(0, 0, 0));
	tf::Quaternion frame_rotation;
	frame_rotation.setEulerZYX(1.5708, 3.1416, 3.1416);
	change_frame.setRotation(frame_rotation);

	transform = change_frame * transform;

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}


void OpenNI2TrackerNodelet::publishTransforms(const std::string& frame_id, const nite::Array<nite::UserData>& users) {
	for (int i = 0; i < users.getSize(); ++i) {
		const nite::UserData& user = users[i];

		if (user.getSkeleton().getState() != nite::SKELETON_TRACKED)
			continue;

		publishTransform(user, nite::JOINT_HEAD,           frame_id, "head");
		publishTransform(user, nite::JOINT_NECK,           frame_id, "neck");
		publishTransform(user, nite::JOINT_TORSO,          frame_id, "torso");

		publishTransform(user, nite::JOINT_LEFT_SHOULDER,  frame_id, "left_shoulder");
		publishTransform(user, nite::JOINT_LEFT_ELBOW,     frame_id, "left_elbow");
		publishTransform(user, nite::JOINT_LEFT_HAND,      frame_id, "left_hand");

		publishTransform(user, nite::JOINT_RIGHT_SHOULDER, frame_id, "right_shoulder");
		publishTransform(user, nite::JOINT_RIGHT_ELBOW,    frame_id, "right_elbow");
		publishTransform(user, nite::JOINT_RIGHT_HAND,     frame_id, "right_hand");

		publishTransform(user, nite::JOINT_LEFT_HIP,       frame_id, "left_hip");
		publishTransform(user, nite::JOINT_LEFT_KNEE,      frame_id, "left_knee");
		publishTransform(user, nite::JOINT_LEFT_FOOT,      frame_id, "left_foot");

		publishTransform(user, nite::JOINT_RIGHT_HIP,      frame_id, "right_hip");
		publishTransform(user, nite::JOINT_RIGHT_KNEE,     frame_id, "right_knee");
		publishTransform(user, nite::JOINT_RIGHT_FOOT,     frame_id, "right_foot");
	}
}

void OpenNI2TrackerNodelet::updateUserState(const nite::UserData& user, unsigned long long ts)
{
	if (user.isNew())
		USER_MESSAGE("New")
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		USER_MESSAGE("Visible")
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		USER_MESSAGE("Out of Scene")
	else if (user.isLost())
		USER_MESSAGE("Lost")

	g_visibleUsers[user.getId()] = user.isVisible();


	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}

} // end of openni2_tracker namespace
