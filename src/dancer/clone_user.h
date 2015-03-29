/*!
  \file        clone_user.h
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

\todo Description of the file

 */

#ifndef CLONE_USER_H
#define CLONE_USER_H

#include "effect_interface.h"
#include "drawing_utils.h"
#include "copy_color_to_out_and_user_edge.h"

cv::Point center_of_mass(const cv::Mat1b & user, const uchar & user_idx) {
  long int sum_x = 0, sum_y = 0, sum_size = 0;
  for (int row = 0; row < user.rows; ++row) {
    const uchar* user_data = user.ptr<uchar>(row);
    for (int col = 0; col < user.cols; ++col) {
      if (*user_data++ != user_idx) {
        sum_x += col;
        sum_y += row;
        ++sum_size;
      }
    } // end loop col
  } // end loop row
  return cv::Point(1.f * sum_x / sum_size, 1.f * sum_y / sum_size);
} // end center_of_mass()

////////////////////////////////////////////////////////////////////////////////

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class CloneUser : virtual public EffectInterface{
public:
  typedef std::map< uchar, std::vector<cv::Point> > CloneMap;

  //////////////////////////////////////////////////////////////////////////////

  CloneUser() {
    reset_new_clone_data();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! return true if s1 comes before s2
  static bool pts_compare(cv::Point const& s1, cv::Point const& s2) {
    return s1.y < s2.y;
  }

  //////////////////////////////////////////////////////////////////////////////

  void first_call() {
    maggiePrint("Left click on a user to copy it, right click to paste it");
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void reset_new_clone_data() {
    new_clone_idx_pos = cv::Point(-1, -1);
    new_clone_pos = cv::Point(-1, -1);
    new_clone_idx = 0;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! consider if we need to create a new clone
  void process_new_clone_data(const cv::Mat1b & user, cv::Mat3b & img_out) {
    // determine clone index if needed
    if (new_clone_idx_pos.x >= 0 && new_clone_idx == 0) {
      new_clone_idx = user.at<uchar>(new_clone_idx_pos);
      if (new_clone_idx == 0) {
        maggiePrint("There is no user at pixel (%i, %i)",
                 new_clone_idx_pos.x, new_clone_idx_pos.y);
        reset_new_clone_data();
        return;
      }
    }

    // insert new clone if needed
    if (new_clone_idx > 0 && new_clone_pos.x > 0) {
      cv::Point clone_translation = new_clone_pos - new_clone_idx_pos;
      clones[new_clone_idx].push_back(clone_translation);
      // special case - we always want to finish with a clone
      // with no translation, because it is the cleanest
      if (clones[new_clone_idx].size() == 1)
        clones[new_clone_idx].push_back(cv::Point(0, 0));
      // sort translations by y
      std::sort(clones[new_clone_idx].begin(), clones[new_clone_idx].end(), pts_compare);
      reset_new_clone_data();
      return;
    } // end if (new_clone_idx > 0 && new_clone_pos.x > 0)

    // draw the contour of selected user if needed
    if (new_clone_idx > 0) {
      draw_users_contour(user, new_clone_idx, new_clone_idx, img_out);
      return;
    }
  } // end process_new_clone_data();

  //////////////////////////////////////////////////////////////////////////////

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    color.copyTo(img_out);

    // make the clones
    for (CloneMap::iterator it = clones.begin() ; it != clones.end() ; ++it) {
      if (it->second.size() == 0)
        continue;
      user_mask = (user == it->first);
      // smooth mask
      cv::erode(user_mask, user_mask, cv::Mat(3, 3, CV_8U, 255));
      //cv::morphologyEx(user_mask, user_mask, cv::MORPH_OPEN, cv::Mat(5, 5, CV_8U, 255));

      for (uint transl_idx = 0; transl_idx < it->second.size(); ++transl_idx) {
        cv::Point transl = it->second[transl_idx];
        image_utils::paste_img(color, img_out, transl.x, transl.y, &user_mask);
      } // end loop transl_idx
    } // end loop it

    // check if new clone needed - must be done after clones because of contours
    process_new_clone_data(user, img_out);
  } // end fn();

  //////////////////////////////////////////////////////////////////////////////

  //! custom mouse callback
  virtual void mouse_cb(int event, int x, int y) {
    maggieDebug3("mouse_cb(event:%i, x:%i, y:%i)", event, x, y);
    if (event == CV_EVENT_LBUTTONDOWN) {
      new_clone_idx_pos = cv::Point(x, y);
      new_clone_idx = 0; // force recomputation
    }
    else if (event == CV_EVENT_RBUTTONDOWN)
      new_clone_pos = cv::Point(x, y);
    else if (event == CV_EVENT_MBUTTONDOWN)
      clones.clear();
  } // end mouse_cb();


  //////////////////////////////////////////////////////////////////////////////

  const char* name() const { return "CloneUser"; }
  CloneMap clones;
  cv::Point new_clone_idx_pos, new_clone_pos;
  uchar new_clone_idx;
  cv::Mat user_mask;
}; // end class CloneUser

#endif // CLONE_USER_H
