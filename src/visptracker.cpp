#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"

#include "visp_bridge/image.h"
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpMeLine.h>
#include <visp/vpTemplateTracker.h>
#include <visp/vpTemplateTrackerSSDESM.h>
#include <visp/vpTemplateTrackerSSDForwardAdditional.h>
#include <visp/vpTemplateTrackerSSDForwardCompositional.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerZNCCForwardAdditional.h>
#include <visp/vpTemplateTrackerZNCCInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpHomography.h>
#include <visp/vpTrackingException.h>

#include "detector/landing_mark_detection.h"

#include "kuri_mbzirc_challenge_1/TrackerData.h"

vpImage<unsigned char> I;
vpDisplayX * display;

// Set 0 for SSDESM, 1 for SSDForwardAdditional,
// 2 for SSDForwardCompositionall, 3 for SSDInverseCompositional 
// 4 for ZNCCForwardAdditional, 5 for ZNCCInverseCompositional
#define TRACKER_TYPE 5

// template tracker variables

vpTemplateTrackerWarpHomography * warp;
vpTemplateTracker * tracker;
bool initialized = false;

// detector
DetectLandingMark detector;

ros::Publisher trackerDataPub;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO("Image Recevied, %d %d %d", msg->header.seq, msg->width, msg->height);
  I  = visp_bridge::toVispImage(*msg);

  vpDisplay::display(I);

  if(!initialized)
  {
    // if we're not tracking, do detection
    bool detected = detector.detect(msg);

    if(detected){
      landing_mark mark = detector.get_landing_mark();

      ROS_INFO("A marker has been detected, (%f, %f, %f, %f)", mark.x, mark.y, mark.width, mark.height );
      vpTemplateTrackerZone tz;
      tz.add(vpTemplateTrackerTriangle(
        vpImagePoint(mark.y, mark.x), 
        vpImagePoint(mark.y + mark.height, mark.x),
        vpImagePoint(mark.y, mark.x + mark.width)
      ));
      tz.add(vpTemplateTrackerTriangle(
        vpImagePoint(mark.y, mark.x + mark.width),
        vpImagePoint(mark.y + mark.height, mark.x),
        vpImagePoint(mark.y + mark.height, mark.x + mark.width)
      ));
	  tracker->initFromZone(I, tz);
      initialized = true;
    }
  }
  
  if(initialized)
  {
    try{
      tracker->track(I);

      // get tracker information
      vpColVector p = tracker->getp();
      vpHomography H = warp->getHomography(p);
      std::cout << "Homography: \n" << H << std::endl;
	  vpTemplateTrackerZone zone_ref = tracker->getZoneRef();
	  vpTemplateTrackerZone zone_warped;
	  warp->warpZone(zone_ref, p, zone_warped);
      
	  // Display the information
      tracker->display(I, vpColor::red);
      
      // create a message with tracker data and publish it
      kuri_mbzirc_challenge_1::TrackerData data;
	  data.minX = zone_warped.getMinx();
	  data.minY = zone_warped.getMiny();
	  data.maxX = zone_warped.getMaxx();
	  data.maxY = zone_warped.getMaxy();
	  trackerDataPub.publish(data);
	  
    }catch(vpTrackingException e)
    {
      ROS_INFO("An exception occurred.. cancelling tracking.");
      tracker->resetTracker();
      initialized = false;
    }
  }
 
  vpDisplay::flush(I);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "visptracker");
  ros::NodeHandle n;
  
  I.init(480, 640);
  
  ros::Subscriber sub = n.subscribe("/ardrone/downward_cam/camera/image", 100, imageCallback);
  trackerDataPub = n.advertise<kuri_mbzirc_challenge_1::TrackerData>("visptracker_data", 1000);
  
  
  display = new vpDisplayX(I, 0, 0, "Image");
  vpDisplay::display(I);
  vpDisplay::flush(I);

  warp = new vpTemplateTrackerWarpHomography();
#if TRACKER_TYPE==0
  tracker = new vpTemplateTrackerSSDESM(warp);
#elif TRACKER_TYPE==1
  tracker = new vpTemplateTrackerSSDForwardAdditional(warp);
#elif TRACKER_TYPE==2
  tracker = new vpTemplateTrackerSSDForwardCompositional(warp);
#elif TRACKER_TYPE==3
  tracker = new vpTemplateTrackerSSDInverseCompositional(warp);
#elif TRACKER_TYPE==4
  tracker = new vpTemplateTrackerZNCCForwardAdditional(warp);
#elif TRACKER_TYPE==5
  tracker = new vpTemplateTrackerZNCCInverseCompositional(warp);
#else
#error "Tracker type not defined properly"
#endif
  tracker->setSampling(2, 2);
  tracker->setLambda(0.001);
  tracker->setIterationMax(200);
  tracker->setPyramidal(4, 1);

  ros::spin();
 
  free(tracker);
  free(warp);
  
  return 0;
}