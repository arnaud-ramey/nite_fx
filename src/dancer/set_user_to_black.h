/*!
  \file        set_user_to_black.h
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

\class SetUserToBlack
\brief A \a EffectInterface that sets the user to a plain color.

 */

#ifndef SET_USER_TO_BLACK_H
#define SET_USER_TO_BLACK_H

#include "effect_interface.h"

#define DILATE_KERNEL_SIZE 10

//! set all pixels that are not null in user to black in img_out
inline void set_user_pixels_to_color_in_out(const cv::Mat1b & user,
                                            cv::Mat3b & img_out,
                                            const cv::Mat & dilate_kernel,
                                            cv::Mat1b &mask,
                                            const cv::Scalar& color_out) {
  // first, modify mask so as to
  // non null pixels in user become 255
  cv::threshold(user, mask, 0, 255, CV_THRESH_BINARY);

  // make the white bigger
  cv::dilate(mask, mask, dilate_kernel);
  // cv::imshow("mask", mask);

#if 0
  img_out.setTo(color_out, mask);
#else
  //cv::rectangle(img_out, cv::Rect(10, 10, 100, 100), cv::Scalar::all(0), -1);
  for (int row = 0; row < img_out.rows; ++row) {
    // get the address of row
    const uchar* mask_data = mask.ptr<uchar>(row);
    uchar* out_data = img_out.ptr<uchar>(row);
    for (int col = 0; col < img_out.cols; ++col) {
      if (*mask_data++ != 0) {
        out_data[3 * col    ] = color_out[0];
        out_data[3 * col + 1] = color_out[1];
        out_data[3 * col + 2] = color_out[2];
      }
    } // end loop col
  } // end loop row
#endif
} // end set_user_pixels_to_color_in_out();

////////////////////////////////////////////////////////////////////////////

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class SetUserToBlack : virtual public EffectInterface{
public:
  SetUserToBlack() {
    dilate_kernel = cv::Mat(DILATE_KERNEL_SIZE, DILATE_KERNEL_SIZE, CV_8U, 255);
  }

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    color.copyTo(img_out);
    set_user_pixels_to_color_in_out(user, img_out, dilate_kernel, mask,
                                    CV_RGB(255, 0, 0));
  } // end fn();

  const char* name() const { return "SetUserToBlack"; }
  cv::Mat dilate_kernel;
  cv::Mat1b mask;
}; // end class SetUserToBlack



#endif // SET_USER_TO_BLACK_H
