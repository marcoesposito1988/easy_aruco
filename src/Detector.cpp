/* Copyright (c) 2012-2019 ImFusion GmbH, Munich, Germany. All rights reserved.
 */
#include "Detector.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace cv;
using namespace std;

void Detector::onCameraInfo(const sensor_msgs::CameraInfo &cam_info) {
  // assumes that the image is undistorted!
  // assumes that the cameraFrame never moves wrt the referenceFrame!

  auto newParams = make_unique<CameraParameters>();

  Mat &cameraMatrix = newParams->cameraMatrix;
  cameraMatrix.setTo(0);
  cameraMatrix.at<double>(0, 0) = cam_info.P[0];
  cameraMatrix.at<double>(0, 1) = cam_info.P[1];
  cameraMatrix.at<double>(0, 2) = cam_info.P[2];
  cameraMatrix.at<double>(1, 0) = cam_info.P[4];
  cameraMatrix.at<double>(1, 1) = cam_info.P[5];
  cameraMatrix.at<double>(1, 2) = cam_info.P[6];
  cameraMatrix.at<double>(2, 0) = cam_info.P[8];
  cameraMatrix.at<double>(2, 1) = cam_info.P[9];
  cameraMatrix.at<double>(2, 2) = cam_info.P[10];

  for (int i = 0; i < 4; ++i)
    newParams->distorsionCoeff.at<double>(i, 0) = 0;

  newParams->size = Size(cam_info.width, cam_info.height);

  cameraParameters = move(newParams);

  // now that everything is initialized, lookup extrinsic transform
  auto cameraToReferenceTransform = std::make_unique<tf2::Transform>();
  tf2::fromMsg(
      buffer.lookupTransform(referenceFrame, cameraFrame, ros::Time(0))
          .transform,
      *cameraToReferenceTransform);

  cameraToReference = std::move(cameraToReferenceTransform);

  cameraInfoSubscriberOnce.shutdown();
}

void Detector::onImage(const sensor_msgs::ImageConstPtr &img) {
  if (!cameraParameters || !cameraToReference)
    return;

  onImageImpl(img);
}

void Detector::start() {

  nh.param<string>("camera_namespace", cameraNamespace, "/camera");
  nh.param<string>("camera_frame", cameraFrame, "camera_rgb_optical_frame");
  nh.param<string>("reference_frame", referenceFrame, "camera_link");
  nh.param<string>("camera_info_topic", cameraInfoTopic,
                   cameraNamespace + "/camera_info");
  nh.param<string>("camera_image_topic", cameraImageTopic,
                   cameraNamespace + "/image_rect_color");

  if (referenceFrame.empty())
    referenceFrame = cameraFrame;

  broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();
  listener = std::make_unique<tf2_ros::TransformListener>(buffer);

  debugImagePublisher = nh.advertise<sensor_msgs::Image>("debug_image", 1);
  cameraInfoSubscriberOnce =
      nh.subscribe(cameraInfoTopic, 1, &Detector::onCameraInfo, this);

  startImpl();

  imageSubscriber = nh.subscribe(cameraImageTopic, 5, &Detector::onImage, this);
}