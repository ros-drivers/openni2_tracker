#ifndef OPENNI2_TRACKER_NODELET_CLASS_H_
#define OPENNI2_TRACKER_NODELET_CLASS_H_

#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#include <NiTE.h>
#include <OpenNI.h>

namespace openni2_tracker {

class OpenNI2TrackerNodelet : public nodelet::Nodelet {
public:
  OpenNI2TrackerNodelet();
  ~OpenNI2TrackerNodelet();

  void onInit();
  void publishTransform(nite::UserData const &user,
                        nite::JointType const &joint,
                        std::string const &frame_id,
                        std::string const &child_frame_id);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void timeCallback(const ros::TimerEvent);
  void publishTransforms(const std::string &frame_id,
                         const nite::Array<nite::UserData> &users);

  void updateUserState(const nite::UserData &user,
                       unsigned long long ts);

  void device_initialization();

private:
  boost::shared_ptr<ros::NodeHandle> nh_;
  boost::shared_ptr<ros::NodeHandle> pnh_;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber depth_img_sub_;
  ros::Timer publish_timer_;
  tf::TransformBroadcaster broadcaster_;

  boost::mutex mutex_;
  boost::shared_ptr<nite::UserTrackerFrameRef> userTrackerFrame_;
  boost::shared_ptr<nite::UserTracker> userTracker_;
  nite::Status niteRc_;
  std::vector<bool> g_visibleUsers;
  std::vector<nite::SkeletonState> g_skeletonStates;
  openni::Device devDevice_;

  std::string frame_id_;
  std::string device_id_;
  double publish_period_;
  int max_users_;
};

} // namespace openni2_tracker

#endif /* OPENNI2_TRACKER_NODELET_CLASS_H_ */
