/*!
  \file        copy_color_to_out_and_user_edge.h
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

\class CopyColorToOutAndUserEdge
\brief A \a EffectInterface that copies the Kinect RGB image to the ouput image
and draws the edge of the user index.

 */

#ifndef COPY_COLOR_TO_OUT_AND_USER_EDGE_H
#define COPY_COLOR_TO_OUT_AND_USER_EDGE_H

#include "effect_interface.h"
#include "skeleton_utils.h"
#include "color_utils.h"

void draw_users_contour(const cv::Mat1b & user,
                        const uchar & min_user_idx, const uchar & max_user_idx,
                        cv::Mat3b & img_out) {
  uchar user_r, user_g, user_b;
  for (int row = 0; row < img_out.rows; ++row) {
    // get the address of row
    const uchar* user_data      = user.ptr<uchar>(row);
    const uchar* user_data_up   = (row > 0 ?
                                     user.ptr<uchar>(row - 1) : NULL);
    const uchar* user_data_down = (row < img_out.rows - 1 ?
                                     user.ptr<uchar>(row + 1) : NULL);
    // uchar* out_data = img_out.ptr<uchar>(row);
    for (int col = 0; col < img_out.cols; ++col) {
      if (user_data[col] < min_user_idx || user_data[max_user_idx] > max_user_idx)
        continue;
      if (   (col > 0               && user_data[col] != user_data[col - 1])
             || (col < img_out.cols -1 && user_data[col] != user_data[col + 1])
             || (row > 0               && user_data[col] != user_data_up[col])
             || (row < img_out.rows -1 && user_data[col] != user_data_down[col])
             ) {
        //  color_utils::indexed_color255
        //      (out_data[3 * col    ],
        //       out_data[3 * col + 1],
        //       out_data[3 * col + 2],
        //       (int) user_data[col]);
        color_utils::indexed_color255
            (user_r, user_g, user_b, (int) user_data[col]);
        cv::circle(img_out, cv::Point(col, row), 2,
                   CV_RGB(user_r, user_g, user_b), -1);
      }
    } // end loop col
  } // end loop row
} // end draw_users_contour();

////////////////////////////////////////////////////////////////////////////////

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class CopyColorToOutAndUserEdge : virtual public EffectInterface{
public:
  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    color.copyTo(img_out);
    draw_users_contour(user, 1, 255, img_out);
    skeleton_utils::draw_skeleton_list(img_out, skeleton_list, 2);
  } // end fn();

  const char* name() const { return "CopyColorToOutAndUserEdge"; }
}; // end class CopyColorToOutAndUserEdge

#endif // COPY_COLOR_TO_OUT_AND_USER_EDGE_H
