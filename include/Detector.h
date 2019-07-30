/* Copyright (c) 2012-2019 ImFusion GmbH, Munich, Germany. All rights reserved.
 */
#pragma once

#include "Utils.h"
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class Detector {
public:
  explicit Detector(const ros::NodeHandle &nh)
      : nh(nh) {}

  void onCameraInfo(const sensor_msgs::CameraInfo &msg);
  void onImage(const sensor_msgs::ImageConstPtr &img);
  void start();

protected:
  virtual void startImpl() = 0;
  virtual void onImageImpl(const sensor_msgs::ImageConstPtr &img) = 0;

  ros::NodeHandle nh;
  std::string cameraInfoTopic;
  std::string cameraImageTopic;
  std::string cameraNamespace;
  std::string cameraFrame;
  std::string referenceFrame;

  tf2_ros::Buffer buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
  std::unique_ptr<tf2::Transform> cameraToReference;

  ros::Subscriber imageSubscriber;
  ros::Subscriber cameraInfoSubscriberOnce;
  ros::Publisher debugImagePublisher;
  std::unique_ptr<CameraParameters> cameraParameters;
};
