/* Copyright (c) 2012-2019 ImFusion GmbH, Munich, Germany. All rights reserved.
 */

#include "Utils.h"
#include <opencv2/calib3d.hpp>

cv::Vec3d rotationVectorWithROSAxes(const cv::Vec3d &Rvec) {
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Rodrigues(Rvec, rot);

  // Rotate axis direction as to fit ROS
  cv::Mat rotate_to_ros =
      (cv::Mat_<double>(3, 3) << 0, 0, 1, 0, -1, 0, 1, 0, 0);
  rot = rot * rotate_to_ros.t();

  cv::Vec3d ret;
  cv::Rodrigues(rot, ret);
  return ret;
}

tf2::Transform rotationAndTranslationVectorsToTransform(const cv::Vec3d &Rvec,
                                                        const cv::Vec3d &Tvec) {
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Rodrigues(rotationVectorWithROSAxes(Rvec), rot);

  tf2::Matrix3x3 tf_rot(
      rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
      rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
      rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));
  tf2::Vector3 tf_orig(Tvec[0], Tvec[1], Tvec[2]);
  tf2::Transform tf_transf(tf_rot, tf_orig);

  return tf_transf;
}

#define str(a) #a

#define PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(id)                               \
  if (idString == str(id))                                                     \
    return cv::aruco::PREDEFINED_DICTIONARY_NAME::id;

cv::aruco::PREDEFINED_DICTIONARY_NAME
arucoDictionaryIdFromString(const std::string &idString) {
  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_4X4_50)
  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_4X4_100)
  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_4X4_250)
  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_4X4_1000)

  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_5X5_50)
  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_5X5_100)
  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_5X5_250)
  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_5X5_1000)

  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_6X6_50)
  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_6X6_100)
  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_6X6_250)
  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_6X6_1000)

  PARSE_PREDEFINED_ARUCO_DICTIONARY_ID(DICT_ARUCO_ORIGINAL)

  throw std::runtime_error("Unsupported predefined dictionary " + idString);
}
