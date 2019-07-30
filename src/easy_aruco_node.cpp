
#include <ArucoMarkerDetector.h>
#include <CharucoBoardDetector.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "easy_aruco_node");
  ros::NodeHandle nh("~");

  std::string objectType;
  nh.getParam("object_type", objectType);

  std::unique_ptr<Detector> detector;

  if (objectType == "aruco_marker")
    detector = std::make_unique<ArucoMarkerDetector>(nh);
  else if (objectType == "charuco_board")
    detector = std::make_unique<CharucoBoardDetector>(nh);
  else
    throw std::runtime_error("unsupported tracking object type " + objectType);

  detector->start();

  ros::spin();
}