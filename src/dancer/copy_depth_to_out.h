/*!
  \file        copy_depth_to_out.h
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

\class CopyDepthToOut
\brief A \a EffectInterface that copies the Kinect depth image to the ouput image.
The visualization mode can be changed hitting SPACE.

 */

#ifndef COPY_DEPTH_TO_OUT_H
#define COPY_DEPTH_TO_OUT_H

#include "effect_interface.h"
#include "cv_conversion_float_uchar.h"
#include <opencv2/highgui/highgui.hpp>

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class CopyDepthToOut : virtual public EffectInterface{
public:
  CopyDepthToOut() : color_mode(image_utils::FULL_RGB_STRETCHED) {}

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    // cv::cvtColor(depth, img_out, cv::COLOR_GRAY2BGR);
    image_utils::depth_image_to_vizualisation_color_image
        (depth, img_out, color_mode);
  } // end fn();

  //! custom mouse callback
  virtual void mouse_cb(int event, int x, int y) {
    maggieDebug2("mouse_cb(event:%i, x:%i, y:%i)", event, x, y);
    if (event == CV_EVENT_LBUTTONDOWN) {
     color_mode = (image_utils::DepthViewerColorMode)
                    ((color_mode + 1) % image_utils::DEPTH_VIEWER_COLOR_NMODES);
    }
  } // end mouse_cb();

  //////////////////////////////////////////////////////////////////////////////

  void first_call() {
    maggiePrint("Left click to change color model");
  }

  //////////////////////////////////////////////////////////////////////////////

  const char* name() const { return "CopyDepthToOut"; }
private:
  image_utils::DepthViewerColorMode color_mode;
}; // end class CopyDepthToOut


#endif // COPY_DEPTH_TO_OUT_H
