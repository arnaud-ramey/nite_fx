/*!
  \file        keep_only_user.h
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

\class KeepOnlyUserColorBackground
\brief A \a EffectInterface that sets the background to a solid color,
keeping color ony in the pixels indicated by user masks.

 */

#ifndef KEEP_ONLY_USER_COLOR_BACKGROUND_H
#define KEEP_ONLY_USER_COLOR_BACKGROUND_H

#include "effect_interface.h"
#include <opencv2/highgui/highgui.hpp>

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class KeepOnlyUserColorBackground : virtual public EffectInterface{
public:
  KeepOnlyUserColorBackground(cv::Vec3b bg_color) : _bg_color(bg_color) {
  }

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {

    img_out.create(user.size());
    img_out = _bg_color;
    color.copyTo(img_out, user);
    //    for (int row = 0; row < color.rows; ++row) {
    //      // get the address of row
    //      const uchar* user_data = user.ptr<uchar>(row);
    //      const uchar* in_data = color.ptr<uchar>(row);
    //      uchar* out_data = img_out.ptr<uchar>(row);
    //      for (int col = 0; col < color.cols; ++col) {
    //        if (*user_data != 0) {
    //          // maggiePrint("(%i, %i)=%i", col, row, *user_data);
    //          out_data[3 * col    ] = in_data[3 * col    ];
    //          out_data[3 * col + 1] = in_data[3 * col + 1];
    //          out_data[3 * col + 2] = in_data[3 * col + 2];
    //        }
    //        ++user_data;
    //      } // end loop col
    //    } // end loop row
  } // end fn();

  const char* name() const { return "KeepOnlyUserColorBackground"; }

  cv::Vec3b _bg_color;
}; // end class KeepOnlyUserColorBackground


#endif // KEEP_ONLY_USER_COLOR_BACKGROUND_H
