/* Copyright (c) 2012-2019 ImFusion GmbH, Munich, Germany. All rights reserved.
 */
#pragma once

#include "Detector.h"
#include "Utils.h"

#include <opencv2/aruco.hpp>
#include <sensor_msgs/Image.h>

class ArucoMarkerDetector : public Detector {
public:
  explicit ArucoMarkerDetector(const ros::NodeHandle &nh)
      : Detector(nh) {}

protected:
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  float markerSideLengthMeters = -1;

  void startImpl() override;
  void onImageImpl(const sensor_msgs::ImageConstPtr &img) override;
};
