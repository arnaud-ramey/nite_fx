/*!
  \file        effect_interface.h
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

\class EffectInterface
\brief An interface for creating visual FX real time on a Kinect video stream.

 */

#ifndef EFFECT_INTERFACE_H
#define EFFECT_INTERFACE_H

#include <opencv2/core/core.hpp>
#ifdef NITE_FX
#include "NiteSkeletonLite.h"
#else  // not NITE_FX
#include <kinect/NiteSkeletonList.h>
#endif // not NITE_FX

class EffectInterface {
public:
  //! inherit this function to init stuff when call for the first time
  virtual void first_call() {}

  virtual void fn
  (const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
   const kinect::NiteSkeletonList & skeleton_list,
   cv::Mat3b & img_out)
  = 0;

  //! a generic mouse callback that call be inherited
  virtual void mouse_cb(int event, int x, int y) {
    // maggieDebug2("mouse_cb(event:%i, x:%i, y:%i)", event, x, y);
  }

  //! return the name of the function
  virtual const char* name() const  = 0;
}; // end class FunFunctionInterface

#endif // EFFECT_INTERFACE_H
