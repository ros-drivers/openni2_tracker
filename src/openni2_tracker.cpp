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

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <NiTE.h>

#include <map>
#include <sstream>

using std::string;

using namespace nite;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

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
    const std::map<nite::JointType, std::string> joints = {
        {JOINT_HEAD, "head"}, {JOINT_NECK, "neck"}, 
        {JOINT_LEFT_HAND, "left_hand"}, {JOINT_LEFT_ELBOW, "left_elbow"}, {JOINT_LEFT_SHOULDER, "left_shoulder"},
        {JOINT_RIGHT_HAND, "right_hand"}, {JOINT_RIGHT_ELBOW, "right_elbow"}, {JOINT_RIGHT_SHOULDER, "right_shoulder"},
        {JOINT_RIGHT_FOOT, "right_foot"}, {JOINT_RIGHT_KNEE, "right_knee"}, {JOINT_RIGHT_HIP, "right_hip"},
        {JOINT_LEFT_FOOT, "left_foot"}, {JOINT_LEFT_KNEE, "left_knee"}, {JOINT_LEFT_HIP, "left_hip"},
        {JOINT_TORSO, "torso"},
    };

    ros::init(argc, argv, "openni_tracker");
    ros::NodeHandle nh, nh_priv("~");
    tf2_ros::TransformBroadcaster br;

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
					ROS_INFO_THROTTLE(1, "Now tracking user %d", user.getId());
                    const nite::Skeleton skeleton = user.getSkeleton();

                    geometry_msgs::TransformStamped transform;
                    transform.header.stamp = ros::Time::now();
                    transform.header.frame_id = frame_id;

                    for(const auto jointNamePair : joints) {
                        const nite::SkeletonJoint& joint = skeleton.getJoint(jointNamePair.first);

						nite::Point3f position = joint.getPosition();
						nite::Quaternion orientation = joint.getOrientation();

						// If the orientation isn't a unit quaternion, replace it with the identy quaternion
                        // (Should only occur for hands and feet)
						if(orientation.x*orientation.x + 
						   orientation.y*orientation.y +
						   orientation.z*orientation.z < 0.001) orientation = nite::Quaternion(1,0,0,0);

						std::stringstream frame;
						frame << jointNamePair.second << "_" << user.getId();

						// Fill out the message
                        transform.child_frame_id = frame.str();
                        transform.transform.translation.x = position.x / 1000.0;
                        transform.transform.translation.y = position.y / 1000.0;
                        transform.transform.translation.z = position.z / 1000.0;

						transform.transform.rotation.x = orientation.x;
						transform.transform.rotation.y = orientation.y;
						transform.transform.rotation.z = orientation.z;
						transform.transform.rotation.w = orientation.w;

                        br.sendTransform(transform);
                    }
				}
			}
		}
		else
		{
			ROS_WARN_STREAM("Get next frame failed.");
		}

		r.sleep();
	}
	return 0;
}

