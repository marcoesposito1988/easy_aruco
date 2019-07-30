/* Copyright (c) 2012-2019 ImFusion GmbH, Munich, Germany. All rights reserved.
 */
#include "ArucoMarkerDetector.h"

#include <sensor_msgs/image_encodings.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

void ArucoMarkerDetector::startImpl()
{
  nh.getParam("marker_size", markerSideLengthMeters);

  string dictionaryIdString;
  nh.getParam("dictionary", dictionaryIdString);
  const auto dictionaryId = arucoDictionaryIdFromString(dictionaryIdString);
  dictionary = aruco::getPredefinedDictionary(dictionaryId);

  detectorParams = aruco::DetectorParameters::create();
}

void ArucoMarkerDetector::onImageImpl(const sensor_msgs::ImageConstPtr &img) {

  auto image = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::RGB8);

  // detect markers and estimate pose
  vector<int> ids;
  vector<vector<Point2f>> corners, rejected;
  vector<Vec3d> rvecs, tvecs;
  aruco::detectMarkers(image->image, dictionary, corners,
                       ids, detectorParams, rejected);
  if (!ids.empty())
    aruco::estimatePoseSingleMarkers(
        corners, markerSideLengthMeters,
        cameraParameters->cameraMatrix, cameraParameters->distorsionCoeff,
        rvecs, tvecs);

  // publish to tf
  std_msgs::Header hdr;
  hdr.frame_id = referenceFrame;
  hdr.stamp = img->header.stamp;

  for (unsigned long i = 0; i < ids.size(); i++) {
    tf2::Transform cameraToMarker_i =
        rotationAndTranslationVectorsToTransform(rvecs[i], tvecs[i]);
    if (referenceFrame != cameraFrame)
    {
      cameraToMarker_i = *cameraToReference*cameraToMarker_i;
    }

    geometry_msgs::TransformStamped cameraToMarker_iStamped;
    cameraToMarker_iStamped.header = hdr;
    cameraToMarker_iStamped.child_frame_id = "marker_" + to_string(ids[i]);
    cameraToMarker_iStamped.transform = tf2::toMsg(cameraToMarker_i);

    broadcaster->sendTransform(cameraToMarker_iStamped);
  }

  // draw on debug image if anyone is interested in it
  if (debugImagePublisher.getNumSubscribers() > 0) {
    Mat imageOutput(image->image);
//    aruco::drawDetectedMarkers(imageOutput, corners, ids);

    for (unsigned long i = 0; i < ids.size(); i++)
      aruco::drawAxis(imageOutput, cameraParameters->cameraMatrix,
                      cameraParameters->distorsionCoeff,
                      rotationVectorWithROSAxes(rvecs[i]),
                      tvecs[i],
                      markerSideLengthMeters * 0.5f);

    cv_bridge::CvImage imageOutputBridge;
    imageOutputBridge.image = imageOutput;
    imageOutputBridge.header = img->header;
    imageOutputBridge.encoding = img->encoding;

    debugImagePublisher.publish(imageOutputBridge.toImageMsg());
  }
}
