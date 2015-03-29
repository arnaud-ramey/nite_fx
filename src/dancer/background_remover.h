/*!
  \file        background_remover.h
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

\class BackgroundRemover
\brief A \a EffectInterface that removes the background thanks to
some background learning method.

No use of the depth is done.
 */

#ifndef BACKGROUND_REMOVER_H
#define BACKGROUND_REMOVER_H

#include "effect_interface.h"

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class BackgroundRemover : virtual public EffectInterface {
public:
  BackgroundRemover() : threshold(30), learningRate(0.5), need_update(false) {}

  //////////////////////////////////////////////////////////////////////////////

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    substract_background(color, img_out);
    if (need_update) {
      need_update = false;
      update_background(color);
    }
  } // end fn();

  //////////////////////////////////////////////////////////////////////////////

  void substract_background(const cv::Mat3b & frame, cv::Mat3b & frame_out) {
    if (bg.empty()) {
      frame_out.create(frame.size());
      frame_out.setTo(0);
      return;
    }
    cv::cvtColor(frame, frameBW, cv::COLOR_BGR2GRAY);
    cv::absdiff(frameBW, bg, fg);
    cv::threshold(fg, fg_thres, threshold, 255, cv::THRESH_BINARY);
    cv::morphologyEx(fg_thres, fg_thres_morph, cv::MORPH_CLOSE, cv::Mat(15, 15, CV_8U, 255));
    //cv::morphologyEx(fg_thres_morph, fg_thres_morph, cv::MORPH_OPEN, cv::Mat(15, 15, CV_8U, 255));

    frame_out.setTo(0);
    frame.copyTo(frame_out, fg_thres_morph);

    //  cv::imshow("bg", bg);
    //  cv::imshow("fg", fg);
    //  cv::imshow("fg_thres", fg_thres);
    //  cv::imshow("fg_thres_morph", fg_thres_morph);
  }

  //////////////////////////////////////////////////////////////////////////////

  // processing method
  void update_background(const cv::Mat3b & frame) {
    maggieDebug2("update_background()");
    if (bg.empty()) {
      cv::cvtColor(frame, bg, cv::COLOR_BGR2GRAY);
      bg.convertTo(bg32, CV_32F);
    }
    else {
      cv::cvtColor(frame, frameBW, cv::COLOR_BGR2GRAY);
      // dst(x, y) = (1 - alpha) * dst(x, y) + alpha * src(x, y)  , if mask(x, y) != 0
      cv::accumulateWeighted(frameBW, bg32, learningRate);
    }
    bg32.convertTo(bg, CV_8U);
  } // end update_background();

  //////////////////////////////////////////////////////////////////////////////

  void first_call() {
    maggiePrint("Left click to updated model");
  }

  //////////////////////////////////////////////////////////////////////////////

  //! custom mouse callback
  virtual void mouse_cb(int event, int x, int y) {
    if (event == CV_EVENT_LBUTTONDOWN)
      need_update = true;
  } // end mouse_cb();

  //////////////////////////////////////////////////////////////////////////////

  const char* name() const { return "BackgroundRemover"; }
  cv::Mat3b frameBW;
  cv::Mat1b bg, fg, fg_thres, fg_thres_morph;
  cv::Mat1f bg32;
  int threshold;
  double learningRate;
  bool need_update;
}; // end class BackgroundRemover

#endif // BACKGROUND_REMOVER_H
