/*!
  \file        keep_only_user_video_background.h
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

\class KeepOnlyUserVideoBackground
\brief A \a EffectInterface that sets a video file as background,
keeping color ony in the pixels indicated by user masks.


 */

#ifndef KEEP_ONLY_USER_VIDEO_BACKGROUND_H
#define KEEP_ONLY_USER_VIDEO_BACKGROUND_H

#include "effect_interface.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "border_remover.h"

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class KeepOnlyUserVideoBackground : virtual public EffectInterface {
public:
  KeepOnlyUserVideoBackground(const std::string & video_filename) :
    average_border_computed(false){
    capture.open(video_filename);
    if (!capture.isOpened())
      maggiePrint("The video file '%s' was not opened succesfully!",
             video_filename.c_str());
  }

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {

    // just set black background if no valid video
    if (!capture.isOpened()) {
      img_out.setTo(0); // set background to black
      color.copyTo(img_out, user);
      return;
    }

    // get the background video image
    if (!capture.read(background)) { // end of video => rewind
      maggieDebug2("Rewinding the background video");
      capture.set(CV_CAP_PROP_POS_MSEC, 0);
      if (!capture.read(background)) {
        maggiePrint("Impossible to rewind the video!");
        return;
      }
    }
    // copy the background to img_out
    cv::resize(background, background_resized, color.size());
    background_resized.copyTo(img_out);

    // keep user from color image
    cv::erode(user, user_mask, cv::Mat(5, 5, CV_8U, 255));
    color.copyTo(img_out, user_mask);

    // paint a black frame where no depth info
    if (!average_border_computed) {
      image_utils::evaluate_mean_border(depth, left, right, up, down);
      average_border_computed = true;
    }
    image_utils::paint_average_border_into_other_image
        (img_out, left, right, up, down, CV_RGB(0, 0, 0));

  } // end fn();

  //////////////////////////////////////////////////////////////////////////////

  void first_call() {
    maggiePrint("Left click to recompute the average border");
  }

  //////////////////////////////////////////////////////////////////////////////

  //! custom mouse callback
  virtual void mouse_cb(int event, int x, int y) {
    if (event == CV_EVENT_LBUTTONDOWN)
      average_border_computed = false;
  } // end mouse_cb();


  const char* name() const { return "KeepOnlyUserVideoBackground"; }

  cv::VideoCapture capture;
  cv::Mat3b background, background_resized;
  bool average_border_computed;
  image_utils::Coord left, right, up, down;
  cv::Mat1b user_mask;
}; // end class KeepOnlyUserVideoBackground

#endif // KEEP_ONLY_USER_VIDEO_BACKGROUND_H
