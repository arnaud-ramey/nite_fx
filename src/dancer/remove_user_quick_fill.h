/*!
  \file        remove_user_quick_fill.h
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

\class RemoveUserQuickFill
\brief A \a EffectInterface that removes the user by propagating the values on his left.

 */

#ifndef REMOVE_USER_QUICK_FILL_H
#define REMOVE_USER_QUICK_FILL_H

#include "effect_interface.h"
#include "value_remover.h"
#include "set_user_to_black.h"

#define DILATE_KERNEL_SIZE 10

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class RemoveUserQuickFill : virtual public EffectInterface {
public:
  RemoveUserQuickFill() {
    dilate_kernel = cv::Mat(DILATE_KERNEL_SIZE, DILATE_KERNEL_SIZE, CV_8U, 255);
  }

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    color.copyTo(img_out);
    set_user_pixels_to_color_in_out(user, img_out, dilate_kernel, mask,
                                    CV_RGB(0, 0, 0));
    image_utils::remove_value_left_propagation(img_out, cv::Vec3b(0,0,0));
  } // end fn();

  const char* name() const { return "RemoveUserQuickFill"; }
  cv::Mat dilate_kernel;
  cv::Mat1b mask;
}; // end class RemoveUserQuickFill

#endif // REMOVE_USER_QUICK_FILL_H
