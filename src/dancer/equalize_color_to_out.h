/*!
  \file        equalize_color_to_out.h
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

\class EqualizeColorToOut
\brief A \a EffectInterface that equalizes each H, S, V channel of the Kinect RGB image
and copies these to the ouput image.

 */

#ifndef EQUALIZE_COLOR_TO_OUT_H
#define EQUALIZE_COLOR_TO_OUT_H

#include <opencv2/imgproc/imgproc.hpp>
#include "effect_interface.h"

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class EqualizeColorToOut : virtual public EffectInterface {
public:
  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    // color.copyTo(img_out);
    cv::cvtColor(color, img_out, cv::COLOR_BGR2HSV);
    cv::split(img_out, channels);
    assert(channels.size() == 3);

    // cv::equalizeHist(channels[0], channels[0]);
    cv::equalizeHist(channels[1], channels[1]);
    cv::equalizeHist(channels[2], channels[2]);

    cv::merge(channels, img_out);
    cv::cvtColor(img_out, img_out, cv::COLOR_HSV2BGR);
  } // end fn();

  const char* name() const { return "EqualizeColorToOut"; }
  std::vector<cv::Mat> channels;
}; // end class EqualizeColorToOut

#endif // EQUALIZE_COLOR_TO_OUT_H
