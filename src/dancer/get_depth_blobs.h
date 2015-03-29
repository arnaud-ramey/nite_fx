/*!
  \file        get_depth_blobs.h
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

\class GetDepthBlobs
\brief A \a EffectInterface that finds users by keeping all depth pixels that are
not undefined (NaN).

 */

#ifndef GET_DEPTH_BLOBS_H
#define GET_DEPTH_BLOBS_H

#include "effect_interface.h"
#include "disjoint_sets2.h"
#include "drawing_utils.h"
#include "copy_user_to_out.h"

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class GetDepthBlobs : virtual public EffectInterface {
public:
  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    // find connected components
    depth_mask = (depth != 0);
    cv::morphologyEx(depth_mask, depth_mask, cv::MORPH_OPEN, cv::Mat(5, 5, CV_8U, 255));
#if 0
    set.process_image(depth_mask);
    set.get_connected_components(depth.cols, components_pts, bounding_boxes);

    // paint them
    fake_user.create(user.size());
    fake_user = 0;
    for (unsigned int comp_idx = 0; comp_idx < components_pts.size(); ++comp_idx)
      image_utils::drawListOfPoints
          (fake_user, components_pts[comp_idx], (uchar) (1 + comp_idx));
#else
    depth_mask.copyTo(fake_user);
#endif
  } // end fn();

  const char* name() const { return "GetDepthBlobs"; }

  //! the fake user map is public
  cv::Mat1b fake_user;

protected:
  cv::Mat1b depth_mask;
  DisjointSets2 set;
  std::vector< std::vector<cv::Point> > components_pts;
  std::vector<cv::Rect> bounding_boxes;

}; // end class GetDepthBlobs

#endif // GET_DEPTH_BLOBS_H
