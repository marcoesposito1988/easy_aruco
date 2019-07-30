/* Copyright (c) 2012-2019 ImFusion GmbH, Munich, Germany. All rights reserved.
 */
#pragma once

#include "Detector.h"
#include <opencv2/aruco/charuco.hpp>
#include <sensor_msgs/Image.h>

class CharucoBoardDetector: public Detector {
public:
  explicit CharucoBoardDetector(const ros::NodeHandle &nh)
      : Detector(nh) {}

protected:
  cv::Ptr<cv::aruco::CharucoBoard> board;
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  float markerSideLengthMeters = -1;
  float squareSideLengthMeters = -1;
  int squareNumberX = -1;
  int squareNumberY = -1;

  void startImpl() override;
  void onImageImpl(const sensor_msgs::ImageConstPtr &img) override;
};
