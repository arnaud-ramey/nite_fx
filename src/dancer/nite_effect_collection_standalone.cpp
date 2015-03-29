/*!
  \file        nite_effect_collection_standalone.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2013/9/20
  
________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

A node subscribing to NITE's RGB, depth, user mask images and skeletons
and applying visual FX on them.

It is a ROS nodelet, cf:
https://code.ros.org/svn/ros-pkg/stacks/common_tutorials/trunk/nodelet_tutorial_math/
http://www.embeddedheaven.com/ros-nodelet.htm

\section Parameters
  - \b "resize_scale"
    [double] (default: 1)
    The scale of the interface.
    The original image is 320x240,
    the GUI size scales it by this parameter.

  - \b DISPLAY
    [bool] (default:1)
    if 1, display results in a GUI

\section Subscriptions

\section Publications

 */

#include "vision_utils/skill_templates/nite/nite_subscriber_template.h"
#include "effect_collection.h"


class NiteEffectCollectionStandalone : public NiteSubscriberTemplate {
public:

  virtual void init() {
    maggieDebug2("NiteEffectCollectionStandalone::int()");
    NiteSubscriberTemplate::init();
    effect_collection.init();
  }

  void fn(const cv::Mat3b & color,
          const cv::Mat1f & depth,
          const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list) {
    effect_collection.fn(color, depth, user, skeleton_list);
  } // end image_callback();

private:
  EffectCollection effect_collection;
}; // end class NiteEffectCollectionStandalone

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "foo_nite_reciever");
  NiteEffectCollectionStandalone receiver;
  receiver.init();
  ros::spin();
}
