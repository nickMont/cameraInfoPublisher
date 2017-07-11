//contains cameraInfo for snapcam.
//MUST RENAME NODE:  snap_cam_highres_publisher/image (etc) to image_raw
//MUST SET CAMFILE:  rosparam set camfile snapcam1 (etc)
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
  publishCameraInfo(std::string filename)
  {
    image_pub_ = n_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    sub_ = n_.subscribe("image_raw", 1, &publishCameraInfo::callback, this);
  }
  void callback(const sensor_msgs::Image& imgmsg)
  {
    //not matching camname to camera name in input file throws an error in camera_info_manager.
    //This error can be ignored.
//    const std::string camurl="file:///home/rnl/.ros/camera_info/snapcam1.yaml";
    std::string camname;
    n_.getParam ( "camfile", camname);  //camname is set by:  rosparam set camfile _____
    const std::string camnameConst=camname;
    const std::string camurlRead="file:///home/rnl/.ros/camera_info/" + camname + ".yaml";
    camera_info_manager::CameraInfoManager caminfo(n_, camnameConst,camurlRead);
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
  publishCameraInfo cameraPubObject("derp");
  ros::spin();

  return 0;
}



