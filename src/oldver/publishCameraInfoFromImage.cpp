#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
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
    camerapub_ = n_.advertise<sensor_msgs::CameraInfo>("/published_topic", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/subscribed_topic", 1, &publishCameraInfo::callback, this);
  }

  void callback(const sensor_msgs::Image jpegmsg)
  {

    sensor_msgs::CameraInfo cameraPub;

    //CALIBRATION PARAMS
    //cameraPub.K={202.52310061128475,0,640.79858539940767,  0,202.52310061128475,361.327674840486,  0,0,1};
    cameraPub.K[0]=202.52310061128475;
    cameraPub.K[1]=0;
    cameraPub.K[2]=640.79858539940767;
    cameraPub.K[3]=0;
    cameraPub.K[4]=202.52310061128475;
    cameraPub.K[5]=361.327674840486;
    cameraPub.K[6]=0;
    cameraPub.K[7]=0;
    cameraPub.K[8]=1;
    //cameraPub.D={0,0,-0.0045420432212910523 -0.00051334513554468541,0}; ///see paper, k1=k2=0 are output by calibration.  k3 assumed equal to zero
    cameraPub.D[0]=0;
    cameraPub.D[1]=0;
    cameraPub.D[2]=-0.0045420432212910523;
    cameraPub.D[3]=-0.00051334513554468541;
    cameraPub.D[4]=0;
    //cameraPub.P={202.52310061128475,0,640.79858539940767,0,  0,202.52310061128475,361.327674840486,0,  0,0,1,1};
    cameraPub.P[0]=202.52310061128475;
    cameraPub.P[1]=0;
    cameraPub.P[2]=640.79858539940767;
    cameraPub.P[3]=0;
    cameraPub.P[4]=0;
    cameraPub.P[5]=202.52310061128475;
    cameraPub.P[6]=361.327674840486;
    cameraPub.P[7]=0;
    cameraPub.P[8]=0;
    cameraPub.P[9]=0;
    cameraPub.P[10]=1;
    cameraPub.P[11]=1;
    //cameraPub.R={1,0,0,  0,1,0,  0,0,1}; //eye(3) for mono camera
    cameraPub.R[0]=1;
    cameraPub.R[1]=0;
    cameraPub.R[2]=0;
    cameraPub.R[3]=0;
    cameraPub.R[4]=1;
    cameraPub.R[5]=0;
    cameraPub.R[6]=0;
    cameraPub.R[7]=0;
    cameraPub.R[8]=1;

    cameraPub.distortion_model="plumb_bob";
    cameraPub.binning_x=1; //shrinkage in x,y
    cameraPub.binning_y=1;
    cameraPub.roi.x_offset=0; //removes part of image if ~=0
    cameraPub.roi.y_offset=0;
    cameraPub.roi.height=1280;
    cameraPub.roi.width= 720;
    cameraPub.roi.do_rectify=false;


    cameraPub.header.frame_id=jpegmsg.header.frame_id;
    cameraPub.header.stamp=jpegmsg.header.stamp;
    camerapub_.publish(cameraPub);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher camerapub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{

  ros::init(argc, argv, "publishCameraInfo");
  ros::spin();

  return 0;
}