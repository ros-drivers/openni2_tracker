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
#include <NiTE.h>

using std::string;
using namespace nite;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
SkeletonState g_skeletonStates[MAX_USERS] = {SKELETON_NONE};

#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

typedef std::map<string, SkeletonJoint> JointMap;

void updateUserState(const UserData &user,
                     unsigned long long ts)
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
    case SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
    case SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
    case SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
    case SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
    case SKELETON_CALIBRATION_ERROR_HANDS:
    case SKELETON_CALIBRATION_ERROR_LEGS:
    case SKELETON_CALIBRATION_ERROR_HEAD:
    case SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}

bool publishJointTF(const string &j_name,
                    const SkeletonJoint &j,
                    const string &tf_prefix,
                    const string &relative_frame,
                    const int &uid,
                    tf::TransformBroadcaster &br)
{
	if (j.getPositionConfidence() > 0.0){
	  tf::Transform transform;
	  transform.setOrigin(tf::Vector3(j.getPosition().x/1000.0, j.getPosition().y/1000.0, j.getPosition().z/1000.0));
	  transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    std::stringstream frame_id_stream;
    string frame_id;
	  frame_id_stream << "/" << tf_prefix << "/user_" << uid << "/" << j_name;
	  frame_id = frame_id_stream.str();
	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), relative_frame, frame_id));
	}
	return true;
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "openni_tracker");
	ros::NodeHandle nh, nh_priv("~");
	tf::TransformBroadcaster br;
  string tf_prefix, relative_frame = "";

	openni::Status rc = openni::STATUS_OK;
	rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK)
	{
	  ROS_ERROR("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
	  return 4;
	}
	
  UserTracker userTracker;
  Status niteRc;

	// Get Tracker Parameters
	if(!nh_priv.getParam("tf_prefix", tf_prefix)){
		ROS_ERROR("tf_prefix not found on Param Server! Check your launch file!");
	}
	if(!nh_priv.getParam("relative_frame", relative_frame)){
		ROS_ERROR("relative_frame not found on Param Server! Check your launch file!");
	}

  NiTE::initialize();

	ros::Rate r(30);
  string frame_id("openni_depth_frame");
	nh_priv.getParam("camera_frame_id", frame_id);

	niteRc = userTracker.create();
  if (niteRc != STATUS_OK)
	{
    printf("Couldn't create user tracker\n%s\n", openni::OpenNI::getExtendedError());
		return 3;
	}
  ROS_INFO_STREAM("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

  UserTrackerFrameRef userTrackerFrame;

	while (ros::ok())
	{
		niteRc = userTracker.readFrame(&userTrackerFrame);
    if (niteRc == STATUS_OK)
		{
      const Array<UserData>& users = userTrackerFrame.getUsers();
			for (int i = 0; i < users.getSize(); ++i)
			{
        const UserData& user = users[i];
				updateUserState(user,userTrackerFrame.getTimestamp());
				if (user.isNew())
				{
					userTracker.startSkeletonTracking(user.getId());
					ROS_INFO_STREAM("Found a new user.");
				}
        else if (user.getSkeleton().getState() == SKELETON_TRACKED)
				{
					ROS_INFO_STREAM("Now tracking user " << user.getId());
					JointMap named_joints;
				
          named_joints["head"] = (user.getSkeleton().getJoint(JOINT_HEAD) );
          named_joints["neck"] = (user.getSkeleton().getJoint(JOINT_NECK) );
          named_joints["left_shoulder"] = (user.getSkeleton().getJoint(JOINT_LEFT_SHOULDER) );
          named_joints["right_shoulder"] = (user.getSkeleton().getJoint(JOINT_RIGHT_SHOULDER) );
          named_joints["left_elbow"] = (user.getSkeleton().getJoint(JOINT_LEFT_ELBOW) );
          named_joints["right_elbow"] = (user.getSkeleton().getJoint(JOINT_RIGHT_ELBOW) );
          named_joints["left_hand"] = (user.getSkeleton().getJoint(JOINT_LEFT_HAND) );
          named_joints["right_hand"] = (user.getSkeleton().getJoint(JOINT_RIGHT_HAND) );
          named_joints["torso"] = (user.getSkeleton().getJoint(JOINT_TORSO) );
          named_joints["left_hip"] = (user.getSkeleton().getJoint(JOINT_LEFT_HIP) );
          named_joints["right_hip"] = (user.getSkeleton().getJoint(JOINT_RIGHT_HIP) );
          named_joints["left_knee"] = (user.getSkeleton().getJoint(JOINT_LEFT_KNEE) );
          named_joints["right_knee"] = (user.getSkeleton().getJoint(JOINT_RIGHT_KNEE) );
          named_joints["left_foot"] = (user.getSkeleton().getJoint(JOINT_LEFT_FOOT) );
          named_joints["right_foot"] = (user.getSkeleton().getJoint(JOINT_RIGHT_FOOT) );

          for (JointMap::iterator it=named_joints.begin(); it!=named_joints.end(); ++it)
          {
            publishJointTF(it->first,it->second,tf_prefix,relative_frame,user.getId(),br);
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

  NiTE::shutdown();
	return 0;
}
