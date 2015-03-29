/*!
  \file        remove_user_inpaint_scale.h
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

\class RemoveUserInPaintScale
\brief A \a EffectInterface that removes the user by inpaint it.
It is faster by scaling the picture down.

 */

#ifndef REMOVE_USER_INPAINT_SCALE_H
#define REMOVE_USER_INPAINT_SCALE_H

#include "effect_interface.h"

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class RemoveUserInPaintScale : virtual public EffectInterface{
public:
  RemoveUserInPaintScale() {
    scale = .3f;
    dilate_kernel = cv::Mat(DILATE_KERNEL_SIZE, DILATE_KERNEL_SIZE, CV_8U, 255);
  }

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {

    cv::resize(color, img_out_scaled, cv::Size(), scale, scale, CV_INTER_NN);
    cv::dilate((user != 0), mask, dilate_kernel);
    //cv::imshow("mask", mask); cv::waitKey(10);
    cv::resize(mask, mask_scaled, cv::Size(), scale, scale, CV_INTER_NN);
    // cv::inpaint(img_out_scaled, mask_scaled, img_out_scaled, 5, cv::INPAINT_NS);
    cv::inpaint(img_out_scaled, mask_scaled, img_out_scaled, 5, cv::INPAINT_TELEA);
    cv::resize(img_out_scaled, img_out, cv::Size(), 1.f / scale, 1.f / scale,
               CV_INTER_NN);
    // restores the hi res outside of the mask
    color.copyTo(img_out, mask == 0);
  } // end fn();

  const char* name() const { return "RemoveUserInPaintScale"; }
  cv::Mat dilate_kernel;
  cv::Mat1b mask, mask_scaled;
  cv::Mat3b img_out_scaled;
  double scale;
}; // end class RemoveUserInPaintScale

#endif // REMOVE_USER_INPAINT_SCALE_H
