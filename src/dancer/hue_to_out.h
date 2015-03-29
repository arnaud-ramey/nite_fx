/*!
  \file        hue_to_out.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/6

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

\class HueToOut
\brief A \a EffectInterface that simply copies the saturated hue channel of
the Kinect RGB image to the ouput image.

 */

#ifndef HUE_TO_OUT_H
#define HUE_TO_OUT_H

#include "effect_interface.h"
#include "layer_utils.h"

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class HueToOut : virtual public EffectInterface {
public:
  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    color_utils::rgb2hue(color, img_out, hue);
    color_utils::hue2rgb(hue, img_out);
  } // end fn();

  const char* name() const { return "HueToOut"; }
  cv::Mat1b hue;
}; // end class HueToOut

#endif // HUE_TO_OUT_H
