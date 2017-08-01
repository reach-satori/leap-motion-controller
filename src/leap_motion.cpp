
// Copyright (c) 2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
// 
//     * Neither the name of the copyright holder nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "Leap.h"	// comes from Leap Motion SDK

#include <iostream>
#include <sstream>
#include <cstring>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "leap_motion_controller/Set.h"
#include <visualization_msgs/Marker.h>

namespace leap_motion_controller
{
  ros::NodeHandle* nhPtr;
}

class Marker_Utility {
   public:
      static void createMarkers(visualization_msgs::Marker* m1, visualization_msgs::Marker* m2);
};

void Marker_Utility::createMarkers(visualization_msgs::Marker* m1, visualization_msgs::Marker* m2) {
   visualization_msgs::Marker* markers[] = {m1, m2};
   for (int i = 0; i <= 1; i++) {
      markers[i]->header.frame_id = "frontframe";
      markers[i]->header.stamp = ros::Time::now();
      markers[i]->ns = "leapmotion";
      markers[i]->id = i;
      markers[i]->type = visualization_msgs::Marker::SPHERE;
      markers[i]->action = visualization_msgs::Marker::ADD;
      markers[i]->pose.position.x = 0;
      markers[i]->pose.position.y = 0;
      markers[i]->pose.position.z = 0;
      markers[i]->pose.orientation.x= 0;
      markers[i]->pose.orientation.y = 0;
      markers[i]->pose.orientation.z = 0;
      markers[i]->pose.orientation.w = 1;
      markers[i]->scale.x = 0.5;
      markers[i]->scale.y = 0.5;
      markers[i]->scale.z = 0.5;
      markers[i]->color.r = 0.0f;
      markers[i]->color.g = 1.0f;
      markers[i]->color.b = 0.0f;
      markers[i]->color.a = 1.0;
      markers[i]->lifetime = ros::Duration();
   }
}



/** Lowpass filter class.
   Refer to Julius O. Smith III, Intro. to Digital Filters with Audio Applications
   This is a 2nd-order Butterworth LPF  */
class lpf
{
  public:
   lpf();
   double filter(const double& new_msrmt);
   double c_ = 1.;	// Related to the cutoff frequency of the filter.
			// c=1 results in a cutoff at 1/4 of the sampling rate.
			// See bitbucket.org/AndyZe/pid if you want to get more sophisticated.
			// Larger c --> trust the filtered data more, trust the measurements less.

  private:
   double prev_msrmts_ [3] = {0., 0., 0.};
   double prev_filtered_msrmts_ [2] = {0., 0.};
};

/** Implementation of Leap::Listener class. */
class LeapListener : public Leap::Listener
{
  public:
   virtual void onInit(const Leap::Controller&);
   virtual void onConnect(const Leap::Controller&);
   virtual void onDisconnect(const Leap::Controller&);
   virtual void onExit(const Leap::Controller&);
   virtual void onFrame(const Leap::Controller&);

   // ROS publisher for Leap Motion Controller data
   ros::Publisher ros_publisher_;
  private:

   // Lowpass filters for the hand positions
   lpf lpf_lhx; // Left hand x-position
   lpf lpf_lhy;
   lpf lpf_lhz;

   lpf lpf_rhx;  // Right hand x-position
   lpf lpf_rhy;
   lpf lpf_rhz;

   // Could also filter orientations, if necessary
};

lpf::lpf(void)
{
  if ( leap_motion_controller::nhPtr->hasParam("/leap_filter_param") )
  {
   ROS_INFO_STREAM("Setting a custom low-pass filter cutoff frequency of " << c_);
   leap_motion_controller::nhPtr->getParam("/leap_filter_param", c_);
  }
  else
   c_ = 4.;
}

double lpf::filter(const double& new_msrmt)
{
  // Push in the new measurement
  prev_msrmts_[2] = prev_msrmts_[1];
  prev_msrmts_[1] = prev_msrmts_[0];
  prev_msrmts_[0] = new_msrmt;

  double new_filtered_msrmt = (1/(1+c_*c_+1.414*c_))*(prev_msrmts_[2]+2*prev_msrmts_[1]+prev_msrmts_[0]-(c_*c_-1.414*c_+1)*prev_filtered_msrmts_[1]-(-2*c_*c_+2)*prev_filtered_msrmts_[0]);;

  // Store the new filtered measurement
  prev_filtered_msrmts_[1] = prev_filtered_msrmts_[0];
  prev_filtered_msrmts_[0] = new_filtered_msrmt;

  return new_filtered_msrmt;
}

void LeapListener::onInit(const Leap::Controller& controller)
{
  std::cout << "Leap Motion Controller Initialized" << std::endl;
}

void LeapListener::onConnect(const Leap::Controller& controller)
{
  std::cout << "Leap Motion Controller Connected" << std::endl;
  
  // Enable recognition of the key tap gesture
  controller.enableGesture(Leap::Gesture::TYPE_KEY_TAP);

  // Configure KeyTapGesture parameters
  controller.config().setFloat("Gesture.KeyTap.MinDownVelocity", 50.0);
  controller.config().setFloat("Gesture.KeyTap.HistorySeconds", 0.1);
  controller.config().setFloat("Gesture.KeyTap.MinDistance", 3.0);
  controller.config().save();
}

void LeapListener::onDisconnect(const Leap::Controller& controller)
{
   std::cout << "Leap Motion Controller Disconnected" << std::endl;
}

void LeapListener::onExit(const Leap::Controller& controller)
{
   std::cout << "Leap Motion Controller Exited" << std::endl;
}

/** This method is executed when new Leap motion frame is available.
 *  It extracts relevant data from Leap::Frame and publishes a corresponding ROS message. 
 */
void LeapListener::onFrame(const Leap::Controller& controller)
{
  // Get the most recent frame
  const Leap::Frame work_frame = controller.frame();

  // Create 'default' static markers to be shown if no hands are in frame.
  visualization_msgs::Marker markerR, markerL;
  visualization_msgs::Marker *rptr = &markerR, *lptr = &markerL;
  Marker_Utility::createMarkers(rptr, lptr);

  // FYI
  std::cout << "- Frame id: " << work_frame.id()
            << ", timestamp: " << work_frame.timestamp()
            << ", hands: " << work_frame.hands().count()
            << ", extended fingers: " << work_frame.fingers().extended().count()
            << ", gestures: " << work_frame.gestures().count() << std::endl;
  
  Leap::HandList hands_in_frame = work_frame.hands();			// get HandList from frame
  if (hands_in_frame.count() > 2)					// if, for some reason, there are more than 2 hands, that's NOT OK, imho
    ROS_INFO("WHAT? There are more than 2 hands in the frame - that's way too many hands! Going to pretend that there are no hands until next frame.");
  else if (hands_in_frame.count() > 0)					// if there are more than 0 hands
  {
   Leap::Hand left_hand, right_hand;					// create empty objects for left_hand and right_hand
   for(int i = 0; i < hands_in_frame.count(); i++)			// go thru all the elements in HandList
   {
      // If Hand is left
      if (hands_in_frame[i].isLeft())
         {
         left_hand = hands_in_frame[i];					// set this hand as left_hand
         lptr->pose.position.x = lpf_lhx.filter(left_hand.palmPosition().x/1000);
         lptr->pose.position.y = lpf_lhy.filter(left_hand.palmPosition().y/1000);
         lptr->pose.position.z = lpf_lhz.filter(left_hand.palmPosition().z/1000);

      // FYI
         std::cout << std::fixed << std::setprecision(1)
           << "Left hand  sphere radius: " << left_hand.sphereRadius()
           << std::fixed << std::setprecision(2)
           << " and pinch strength: " << left_hand.pinchStrength() << std::endl;

      // FYI
         std::cout << std::fixed << std::setprecision(4)
           << "           position: x= " << markerL.pose.position.x
           << " y= " << markerL.pose.position.y
           << " z= " << markerL.pose.position.z << std::endl;
         }
         
         // If Hand is right
      else if (hands_in_frame[i].isRight()) {
         right_hand = hands_in_frame[i];					// set this hand as right_hand
         rptr->pose.position.x = lpf_rhx.filter(right_hand.palmPosition().x/1000);
         rptr->pose.position.y = lpf_rhy.filter(right_hand.palmPosition().y/1000);
         rptr->pose.position.z = lpf_rhz.filter(right_hand.palmPosition().z/1000);

      // FYI
         std::cout << std::fixed << std::setprecision(1)
           << "right hand  sphere radius: " << right_hand.sphereRadius()
           << std::fixed << std::setprecision(2)
           << " and pinch strength: " << right_hand.pinchStrength() << std::endl;

      // FYI
         std::cout << std::fixed << std::setprecision(4)
           << "           position: x= " << markerR.pose.position.x
           << " y= " << markerR.pose.position.y
           << " z= " << markerR.pose.position.z << std::endl;
         }
   } // for
  } // else if (hands_in_frame.count() > 0)
   
  // Publish the ROS message based on this Leap Motion Controller frame.
  ros_publisher_.publish( markerR );
  ros_publisher_.publish( markerL );

  // Throttle the loop
  ros::Duration(0.1).sleep();
 
} // end LeapListener::onFrame()


/** Main method. */
int main(int argc, char** argv)
{
  // ROS init
  ros::init(argc, argv, "leap_motion");
  
  // ROS node handle
  ros::NodeHandle nh;
  leap_motion_controller::nhPtr = &nh; // Allow other functions to interact with the nodehandle via this pointer

  // Instance of LeapListener
  LeapListener listener;

  // Set up ROS publisher for LeapListener
  listener.ros_publisher_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  // Instance of LEAP Controller
  Leap::Controller controller;
  
  // Have the listener receive events from the controller
  controller.addListener(listener);

  // Do ros::spin() until CTRL+C is pressed
  ros::spin();
 
  // Remove the listener when done
  controller.removeListener(listener);

  return 0;
} // main
