/*!
  \file        calibrator.h
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

\class Calibrator
\brief A \a EffectInterface that simply copies the Kinect RGB image to the ouput image.

 */

#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include "effect_interface.h"
#include "cv_conversion_float_uchar.h"
#include "skeleton_utils.h"
#include "resize_utils.h"

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class Calibrator : virtual public EffectInterface {
public:
  static const int MAX_OFFSET = 300; // pixels
  static const float MAX_SCALE = 2; // mult factor
  static const float TB_SCALE_FACTOR = 100; // mult factor

  Calibrator() : color_mode(image_utils::FULL_RGB_SCALED)  {
    win = "Calibrator";
    cv::namedWindow(win);
    xoffset_tb = yoffset_tb = MAX_OFFSET;
    yscale_tb = xscale_tb = 1.f * TB_SCALE_FACTOR;
    cv::createTrackbar("xoffset", win, &xoffset_tb, 2 * MAX_OFFSET);
    cv::createTrackbar("yoffset", win, &yoffset_tb, 2 * MAX_OFFSET);
    cv::createTrackbar("xscale", win, &xscale_tb, MAX_SCALE * TB_SCALE_FACTOR);
    cv::createTrackbar("yscale", win, &yscale_tb, MAX_SCALE * TB_SCALE_FACTOR);
  }

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    image_utils::depth_image_to_vizualisation_color_image
        (depth, img_non_calibrated, color_mode);
    skeleton_utils::draw_skeleton_list(img_non_calibrated, skeleton_list, 2);

    // img_non_calibrated.copyTo(img_out);
    float xscale = 1.f * xscale_tb / TB_SCALE_FACTOR,
        yscale = 1.f * yscale_tb / TB_SCALE_FACTOR;
    int xoffset = xoffset_tb - MAX_OFFSET,
        yoffset = yoffset_tb - MAX_OFFSET;
    image_utils::scale_img_forward_backward(img_non_calibrated, img_out,
                                            xscale, xoffset, yscale, yoffset);
    maggieDebug2("scale:(%g, %g), offset:(%i, %i)",
           xscale, yscale, xoffset, yoffset);
    //cv::imshow(win, img_non_calibrated);
  } // end fn();

  const char* name() const { return "Calibrator"; }

  image_utils::DepthViewerColorMode color_mode;
  std::string win;
  int xoffset_tb, yoffset_tb;
  int xscale_tb, yscale_tb;
  cv::Mat3b img_non_calibrated;
}; // end class Calibrator

#endif // CALIBRATOR_H
