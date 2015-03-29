/*!
  \file        depth_background_remover.h
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

\class DepthBackgroundRemover
\brief A \a EffectInterface that removes the background thanks to the depth image.

 */

#ifndef DEPTH_BACKGROUND_REMOVER_H
#define DEPTH_BACKGROUND_REMOVER_H

#include "effect_interface.h"
#include "copy_user_to_out.h"
#include "copy_depth_to_out.h"
#include "keep_only_user_color_background.h"

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class DepthBackgroundRemover :
    virtual public EffectInterface,
    virtual public CopyUserToOut,
    virtual public CopyDepthToOut,
    virtual public KeepOnlyUserColorBackground
{
public:
  /*! the difference between a pixel depth and the background depth
  in meters to be considered as foreground */
  static const double FOREGROUND_MIN_DEPTH_RATIO = .9;

  //! ctor
  DepthBackgroundRemover()
    : KeepOnlyUserColorBackground(cv::Vec3b(255, 255, 255)) {}

  //////////////////////////////////////////////////////////////////////////////

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    // create a background from depth the first time
    if (background.empty())
      depth.copyTo(background);

    // foreground objects are such as background - depth > thresh
    fake_user.create(depth.size());
    fake_user = 0;
    unsigned int n_pixels = depth.cols * depth.rows;
    const float* depth_ptr = depth.ptr<float>();
    float* background_ptr = background.ptr<float>();
    uchar* fake_user_ptr = fake_user.ptr();
    for (unsigned int pixel_idx = 0; pixel_idx < n_pixels; ++pixel_idx) {
      // remove from fake_user the points were depth is not defined (0)
      if (*depth_ptr != 0
          && *depth_ptr < *background_ptr * FOREGROUND_MIN_DEPTH_RATIO)
        *fake_user_ptr = 255;
      ++depth_ptr;
      ++background_ptr;
      ++fake_user_ptr;
    } // end loop pixel_idx
    cv::morphologyEx(fake_user, fake_user, cv::MORPH_OPEN, cv::Mat(10, 10, CV_8U, 255));

    // now, some funny processing
    // do not do it as it will be done by the successive filters
    // CopyUserToOut::fn(color, depth, fake_user, img_out);
    // CopyDepthToOut::fn(color, background, user, img_out);
    // KeepOnlyUserColorBackground::fn(color, depth, fake_user, skeleton_list, img_out);

    // update background
    // cv::max(background, depth, background);
    depth_ptr = depth.ptr<float>();
    background_ptr = background.ptr<float>();
    for (unsigned int pixel_idx = 0; pixel_idx < n_pixels; ++pixel_idx) {
      if (*depth_ptr > *background_ptr)
        *background_ptr = *depth_ptr;
      ++depth_ptr;
      ++background_ptr;
    } // end loop pixel_idx
  } // end fn();

  const char* name() const { return "DepthBackgroundRemover"; }

  cv::Mat1f background;
  cv::Mat1f foreground;
  cv::Mat1b fake_user;
}; // end class DepthBackgroundRemover

#endif // DEPTH_BACKGROUND_REMOVER_H
