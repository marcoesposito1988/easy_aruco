/* Copyright (c) 2012-2019 ImFusion GmbH, Munich, Germany. All rights reserved.
 */

#include "CharucoBoardDetector.h"
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace cv;
using namespace std;

void CharucoBoardDetector::startImpl() {
  nh.getParam("marker_size", markerSideLengthMeters);
  nh.getParam("square_size", squareSideLengthMeters);
  nh.getParam("square_number_x", squareNumberX);
  nh.getParam("square_number_y", squareNumberY);

  string dictionaryIdString;
  nh.getParam("dictionary", dictionaryIdString);
  const auto dictionaryId = arucoDictionaryIdFromString(dictionaryIdString);
  dictionary = aruco::getPredefinedDictionary(dictionaryId);

  detectorParams = aruco::DetectorParameters::create();

  board = cv::aruco::CharucoBoard::create(squareNumberX, squareNumberY,
                                          squareSideLengthMeters,
                                          markerSideLengthMeters, dictionary);
}

void CharucoBoardDetector::onImageImpl(const sensor_msgs::ImageConstPtr &img) {

  auto image = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::RGB8);

  // detect markers and estimate pose
  vector<int> ids;
  vector<vector<Point2f>> corners, rejected;
  Vec3d rvec, tvec;
  aruco::detectMarkers(image->image, dictionary, corners, ids, detectorParams,
                       rejected);

  bool boardDetected = false;
  std::vector<cv::Point2f> charucoCorners;
  std::vector<int> charucoIds;
  if (!ids.empty()) {
    aruco::interpolateCornersCharuco(
        corners, ids, image->image, board, charucoCorners, charucoIds,
        cameraParameters->cameraMatrix, cameraParameters->distorsionCoeff);

    boardDetected = cv::aruco::estimatePoseCharucoBoard(
        charucoCorners, charucoIds, board, cameraParameters->cameraMatrix,
        cameraParameters->distorsionCoeff, rvec, tvec);
  }

  // publish to tf
  std_msgs::Header hdr;
  hdr.frame_id = referenceFrame;
  hdr.stamp = img->header.stamp;

  if (boardDetected) {
    tf2::Transform cameraToBoard =
        rotationAndTranslationVectorsToTransform(rvec, tvec);
    if (referenceFrame != cameraFrame) {
      cameraToBoard = *cameraToReference * cameraToBoard;
    }

    geometry_msgs::TransformStamped cameraToBoardStamped;
    cameraToBoardStamped.header = hdr;
    cameraToBoardStamped.child_frame_id = "board";
    cameraToBoardStamped.transform = tf2::toMsg(cameraToBoard);

    broadcaster->sendTransform(cameraToBoardStamped);
  }

  // draw on debug image if anyone is interested in it
  if (debugImagePublisher.getNumSubscribers() > 0) {
    Mat imageOutput(image->image);

    if (boardDetected) {

      aruco::drawAxis(imageOutput, cameraParameters->cameraMatrix,
                      cameraParameters->distorsionCoeff, rvec, tvec,
                      squareSideLengthMeters * 0.75f);
    }

    cv_bridge::CvImage imageOutputBridge;
    imageOutputBridge.image = imageOutput;
    imageOutputBridge.header = img->header;
    imageOutputBridge.encoding = sensor_msgs::image_encodings::RGB8;

    debugImagePublisher.publish(imageOutputBridge.toImageMsg());
  }
}
