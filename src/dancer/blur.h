/*!
  \file        blur.h
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

\class Blur
\brief A \a EffectInterface that blurs the image.
 */

#ifndef BLUR_H
#define BLUR_H

#include "effect_interface.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class Blur : virtual public EffectInterface {
public:
  Blur() {
    blur_std_dev =2;
  }

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    if (blur_std_dev == 0)
      color.copyTo(img_out);
    else {
      // kernel size must be odd
      int kernel_size = 2 * (int) (blur_std_dev / 2) + 5;
      cv::GaussianBlur(color, img_out, cv::Size(kernel_size, kernel_size),
                       blur_std_dev, blur_std_dev);
    }
  } // end fn();

  //////////////////////////////////////////////////////////////////////////////

  void first_call() {
    maggiePrint("Left click to blur more, right click to blur less");
  }

  //////////////////////////////////////////////////////////////////////////////

  virtual void mouse_cb(int event, int x, int y) {
    if (event == CV_EVENT_LBUTTONDOWN) {
      ++blur_std_dev;
      maggieDebug2("blur_std_dev:%i", blur_std_dev);
    }
    else if (event == CV_EVENT_RBUTTONDOWN) {
      if (blur_std_dev > 0)
        --blur_std_dev;
      maggieDebug2("blur_std_dev:%i", blur_std_dev);
    }
  }


  const char* name() const { return "Blur"; }

  int blur_std_dev;
}; // end class Blur

#endif // BLUR_H
