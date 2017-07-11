//contains cameraInfo for snapcam
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "camera_info_manager/camera_info_manager.h"
using namespace std;

/**Code adapted from
http://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/
  More efficient code to modify image before republishing at
http://answers.ros.org/question/53234/processing-an-image-outside-the-callback-function/
*/
class publishCameraInfo
{
public:
  publishCameraInfo()
  {
    //Topic you want to publish
    image_pub_ = n_.advertise<sensor_msgs::CameraInfo>("snap_cam_camInfo/CameraInfo", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("snap_cam_highres_publisher/image", 1, &publishCameraInfo::callback, this);
  }

  void callback(const sensor_msgs::Image& imgmsg)
  {
    const std::string camname="snapcam1";
//    const std::string camurl="file://~/catkin_ws/src/undistort_images/calib_files/snapcam1.yaml";
//    const std::string camurl="file://~/.ros/camera_info/snapcam1.yaml";
//    const std::string camurl="file://${ROS_HOME}/camera_info/snapcam1.yaml";
    const std::string camurl="file:///home/rnl/.ros/camera_info/snapcam1.yaml";
    camera_info_manager::CameraInfoManager caminfo(n_, camname,camurl);
//    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(caminfo->getCameraInfo()));  //copied
//    camera_info_manager::CameraInfoManager caminfo(n_, camname);
    sensor_msgs::CameraInfo ci;
    ci=caminfo.getCameraInfo();

    ci.header.stamp = imgmsg.header.stamp;
    ci.header.frame_id = imgmsg.header.frame_id;
    
    // Publish via image_transport
    image_pub_.publish(ci);
  }
  

private:
  ros::NodeHandle n_; 
  ros::Publisher image_pub_;
  ros::Subscriber sub_;

};//End of class

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publishCameraInfo");
  publishCameraInfo cameraPubObject;
  ros::spin();

  return 0;
}



