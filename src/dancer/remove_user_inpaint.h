/*!
  \file        remove_user_inpaint.h
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

\class RemoveUserInPaint
\brief A \a EffectInterface that removes the user by inpaint it.

 */

#ifndef REMOVE_USER_INPAINT_H
#define REMOVE_USER_INPAINT_H

#include "effect_interface.h"

#include <opencv2/core/version.hpp>
#if (CV_MAJOR_VERSION > 2) || (CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION > 3)
#include <opencv2/photo/photo.hpp>
#endif

#define DILATE_KERNEL_SIZE 10

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class RemoveUserInPaint : virtual public EffectInterface{
public:
  RemoveUserInPaint() {
    dilate_kernel = cv::Mat(DILATE_KERNEL_SIZE, DILATE_KERNEL_SIZE, CV_8U, 255);
  }

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    color.copyTo(img_out);
    cv::threshold(user, mask, 0, 255, CV_THRESH_BINARY);
    cv::dilate(mask, mask, dilate_kernel);
    cv::inpaint(img_out, mask, img_out, 5, cv::INPAINT_NS);
  } // end fn();

  const char* name() const { return "RemoveUserInPaint"; }
  cv::Mat dilate_kernel;
  cv::Mat1b mask;
}; // end class RemoveUserInPaint

#endif // REMOVE_USER_INPAINT_H
