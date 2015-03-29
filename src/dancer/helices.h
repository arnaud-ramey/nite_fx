/*!
  \file        helices.h
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

\class Helices
\brief A \a EffectInterface that simply copies the Kinect RGB image to the ouput image.

 */

#ifndef HELICES_H
#define HELICES_H

#include <math.h>

#include <deque>
#include "timer.h"
#include "copy_color_to_out_and_user_edge.h"
#include "effect_interface.h"

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class Helices : virtual public EffectInterface {
public:
  static const float TWOPI = 2 * M_PI;

  static const float ANG_SPEED_INCR = .5;
  static const float ANG_SPEED_DECR = .5;
  static const float ANG_SPEED_MAX = 15;
  static const float ANG_SPEED_IDLE = .8;

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  class Helix {
  public:
    Helix(const cv::Point center, const float angle, const float angspeed = 0) :
      _center(center), _angle(angle), _angspeed(angspeed), _idle_color(255, 50, 50) {
      _color = _idle_color;
    }

    ////////////////////////////////////////////////////////////////////////////

    inline void set_color(const cv::Scalar & newcol) {
      // _color = newcol;
    }

    void refresh_angle(const cv::Mat1b & user,
                       const float & shared_angle_2pi) {
      // update angle
      _angle += _last_refresh.getTimeSeconds() * _angspeed;
      _last_refresh.reset();

      // change speed according to user mask
      if (user.at<uchar>(_center)) {
        _angspeed = std::min(ANG_SPEED_MAX, _angspeed + ANG_SPEED_INCR);
        set_color(CV_RGB(255, 0, 0)); // red
      }

      // otherwise (slower than ANG_SPEED_IDLE), set to ANG_SPEED_IDLE
      else if (_angspeed <= ANG_SPEED_IDLE){
        set_color(_idle_color); // blue
        _angle = shared_angle_2pi;
        _angspeed = ANG_SPEED_IDLE;
      } // end if not user

      // if slightly faster than ANG_SPEED_IDLE,
      else if (_angspeed <= ANG_SPEED_IDLE + 3 * ANG_SPEED_DECR) {
        // keep this speed till being close to shared_angle
        set_color(CV_RGB(0, 255, 0)); // green
        _angspeed = ANG_SPEED_IDLE + 2 * ANG_SPEED_DECR; // slowdown speed
        // close enough, ie _angle ~= shared_angle + n PI
        // <=> fmod( _angle - shared_angle, PI ) ~= 0
        // -> set _angle to shared_angle
        if (fmod(_angle - shared_angle_2pi, M_PI) < .3) {
          _angle = shared_angle_2pi;
          _angspeed = ANG_SPEED_IDLE;
        }
      } // end if slightly faster than ANG_SPEED_IDLE

      // if a lot faster, decrease speed
      else /*if (_angspeed > ANG_SPEED_IDLE + ANG_SPEED_DECR)*/ {
        set_color(CV_RGB(255, 255, 0)); // orange
        _angspeed = std::max((float) 0, _angspeed - ANG_SPEED_DECR);
      }
    } // end refresh_angle();

    ////////////////////////////////////////////////////////////////////////////

    void draw_on(cv::Mat3b & img) {
      float cosa = cos(_angle) * radius, sina = sin(_angle) * radius;
      cv::Point end1(_center.x + cosa, _center.y + sina),
          end2(_center.x - cosa, _center.y - sina);
      cv::line(img, end1, end2, _color, 2, CV_AA);
      // cv::line(img, end1, _center, _color, 2);
    }

    ////////////////////////////////////////////////////////////////////////////

    cv::Point _center;
    float _angle;
    static const int radius = 15;
    float _angspeed;
    Timer _last_refresh;
    cv::Scalar _color;
    cv::Scalar _idle_color;
  }; // end class Helix

  //////////////////////////////////////////////////////////////////////////////

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    int rows = color.rows, cols = color.cols;
    float shared_angle = timer.getTimeSeconds() * ANG_SPEED_IDLE;
    float shared_angle_2pi = // shared_angle - TWOPI * (int) (shared_angle / TWOPI);
        fmod(shared_angle, TWOPI);

    if (helices.empty()) {
      maggieDebug2("Creating helices");
      int rowstep = 30, colstep = 50, helixrowidx = 0;
      for (int row = 0; row < rows; row+=rowstep) {
        ++helixrowidx;
        for (int col = 0; col < cols; col+=colstep) {
          cv::Point center(col + (helixrowidx%2) * colstep/2, row);
          helices.push_back(Helix(center, shared_angle_2pi, ANG_SPEED_IDLE));
        } // end loop col
      } // end loop row
    } // end if (helices.empty())

    // user_image_to_rgb(user, img_out, 8);
    img_out.setTo(0);
    draw_users_contour(user, 1, 255, img_out);
    // draw helices
    for (unsigned int helix_idx = 0; helix_idx < helices.size(); ++helix_idx) {
      helices[helix_idx].refresh_angle(user, shared_angle_2pi);
      helices[helix_idx].draw_on(img_out);
    }
  } // end fn();

  const char* name() const { return "Helices"; }
  std::deque<Helix> helices;
  Timer timer;
}; // end class Helices

#endif // HELICES_H
