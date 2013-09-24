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

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

//#include <OpenNI.h>
//#include <XnCodecIDs.h>
//#include <XnCppWrapper.h>
#include <NiTE.h>

using std::string;

//xn::Context        g_Context;
//xn::DepthGenerator g_DepthGenerator;
//xn::UserGenerator  g_UserGenerator;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

void publishTransform(nite::UserData const& user, nite::JointType const& joint, string const& frame_id, string const& child_frame_id) {
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

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}


void publishTransforms(const std::string& frame_id, const nite::Array<nite::UserData>& users) {
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


void updateUserState(const nite::UserData& user, unsigned long long ts)
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

int main(int argc, char **argv) {

    ros::init(argc, argv, "openni_tracker");
    ros::NodeHandle nh, nh_priv("~");

    nite::UserTracker userTracker;
    nite::Status niteRc;

    nite::NiTE::initialize();

	ros::Rate r(30);
    std::string frame_id("openni_depth_frame");
    nh_priv.getParam("camera_frame_id", frame_id);

	niteRc = userTracker.create();
	if (niteRc != nite::STATUS_OK)
	{
		printf("Couldn't create user tracker\n");
		return 3;
	}
	printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

	nite::UserTrackerFrameRef userTrackerFrame;

	while (ros::ok())
	{
		niteRc = userTracker.readFrame(&userTrackerFrame);
		if (niteRc == nite::STATUS_OK)
		{
			const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
			for (int i = 0; i < users.getSize(); ++i)
			{
				const nite::UserData& user = users[i];
				updateUserState(user,userTrackerFrame.getTimestamp());
				if (user.isNew())
				{
					userTracker.startSkeletonTracking(user.getId());
					ROS_INFO_STREAM("Found a new user.");
				}
				else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
				{
					ROS_INFO_STREAM("Now tracking user " << user.getId());
				}
			}
                        publishTransforms(frame_id, users);
		}
		else
		{
			ROS_WARN_STREAM("Get next frame failed.");
		}

		r.sleep();
	}

        nite::NiTE::shutdown();

	return 0;
}
