/*!
  \file        copy_user_to_out.h
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

\class CopyColorToOut
\brief A \a EffectInterface that simply copies the user masks image to the ouput image.

 */

#ifndef COPY_USER_TO_OUT_H
#define COPY_USER_TO_OUT_H

#include "effect_interface.h"
#include "skeleton_utils.h"
#include "user_image_to_rgb.h"

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class CopyUserToOut : virtual public EffectInterface {
public:
  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    user_image_to_rgb(user, img_out, 8);
    skeleton_utils::draw_skeleton_list(img_out, skeleton_list, 2);
  } // end fn();

  const char* name() const { return "CopyUserToOut"; }
}; // end class CopyUserToOut

#endif // COPY_USER_TO_OUT_H
