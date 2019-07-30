/* Copyright (c) 2012-2019 ImFusion GmbH, Munich, Germany. All rights reserved.
 */
#pragma once

#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core.hpp>
#include <tf2/LinearMath/Transform.h>

struct CameraParameters {
  cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64FC1);
  cv::Mat distorsionCoeff = cv::Mat(4, 1, CV_64FC1);
  cv::Size size;
};


cv::Vec3d rotationVectorWithROSAxes(const cv::Vec3d &Rvec);

tf2::Transform
rotationAndTranslationVectorsToTransform(const cv::Vec3d &Rvec, const cv::Vec3d &Tvec);

cv::aruco::PREDEFINED_DICTIONARY_NAME arucoDictionaryIdFromString(const std::string& idString);