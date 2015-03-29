/*!
  \file        effect_collection.h
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

\class EffectCollection
\brief Stores a bunch of \a EffectInterface in a vector and switches between them.

Can also change the way users are detected.

 */

#ifndef EFFECT_COLLECTION_H
#define EFFECT_COLLECTION_H

#ifndef NITE_FX
#include <ros/ros.h>
#endif // not NITE_FX
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "timer.h"
#include "drawing_utils.h"

#include "effect_interface.h"
// possible effects for user detection
#include "get_depth_blobs.h"
#include "depth_background_remover.h"

class EffectCollection {
public:
  EffectCollection() {
  }

  void init() {
    // init variables
    _curr_effect_idx = 0;
    _curr_user_detection_effect = USER_DETECTION_NITE;
    _resize_scale = 1;
    window_name = "nite_foo_receiver";

    // get params
#ifdef NITE_FX
    _resize_scale = 1.5;
    DISPLAY = true;
#else // not NITE_FX
    ros::NodeHandle nh_private("~");
    nh_private.param("resize_scale", _resize_scale, _resize_scale);
    nh_private.param("DISPLAY", DISPLAY, true);
#endif // not NITE_FX
    // add all effects
    add_all_effects();
    set_effect(0); // set the first effect active (CopyToColor)

    maggiePrint("Using user_detection_effect '%s', effect %s, _resize_scale %g",
                user_detection_effect_to_string(_curr_user_detection_effect).c_str(),
                effects[_curr_effect_idx]->name(), _resize_scale);
    maggiePrint("Press SPACE or 'n' for next FX, "
                "backspace or 'p' for previous FX, "
                "'u' to change user detection algorithm");

    image_out.create(1, 1);
    cv::namedWindow(window_name);
    cvMoveWindow(window_name.c_str(), 0, 0);
    cv::setMouseCallback(window_name, mouse_cb, this);
  } // end init()

  //////////////////////////////////////////////////////////////////////////////

  void add_all_effects();

  //////////////////////////////////////////////////////////////////////////////

  ~EffectCollection() {
    // delete all effects
    for (unsigned int idx = 0; idx < effects.size(); ++idx)
      delete effects[idx];
  } // end dtor

  //////////////////////////////////////////////////////////////////////////////

  void fn(const cv::Mat3b & color,
          const cv::Mat1f & depth,
          const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list
          ) {
    maggieDebug3("fn() - user detection effect:%i, fn:%s",
                      _curr_user_detection_effect,
                      effects[_curr_effect_idx]->name());
    Timer timer;
    // user detection and call the effect
    if (_curr_user_detection_effect == USER_DETECTION_NITE) {
      effects[_curr_effect_idx]->fn(color, depth, user, skeleton_list, image_out);
    }
    else if (_curr_user_detection_effect == USER_DETECTION_DEPTH_BLOB) {
     get_depth_blobs_effect.fn
         (color, depth, user, skeleton_list, image_out);
     maggieDebug3("time for user detectop, with GetDepthBlobs: %g ms",
                       timer.getTimeMilliseconds());
      effects[_curr_effect_idx]->fn
          (color, depth, get_depth_blobs_effect.fake_user, skeleton_list,
           image_out);
    }
    else if (_curr_user_detection_effect == USER_DETECTION_DEPTH_BACKGROUND_REMOVER) {
      depth_bacground_remover_effect.fn
          (color, depth, user, skeleton_list, image_out);
      maggieDebug3("time for user detectop, with DepthBackgroundRemover: %g ms",
                        timer.getTimeMilliseconds());
      effects[_curr_effect_idx]->fn
          (color, depth, depth_bacground_remover_effect.fake_user, skeleton_list,
           image_out);
    }

    maggieDebug3("time for effect fn: %g ms", timer.getTimeMilliseconds());

    // write method
    std::ostringstream txt;
    txt << effects[_curr_effect_idx]->name()
        << " (" << _curr_user_detection_effect << ")";
    image_utils::draw_text_centered
        (image_out, txt.str(),
         cv::Point(image_out.cols / 2, image_out.rows - 50),
         CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 0, 0), 1);

    // display
    if (!DISPLAY)
      return;
    if (_resize_scale == 1) {
      cv::imshow(window_name, image_out);
    }
    else {
      cv::resize(image_out, image_out_scaled, cv::Size(),
                 _resize_scale, _resize_scale, cv::INTER_NEAREST);
      cv::imshow(window_name, image_out_scaled);
    }
    char c = cv::waitKey(5);
    // maggieDebug2("c:%i", c);
    if (c == 'n' || c == ' ') // set next effect
      set_effect((_curr_effect_idx + 1) % effects.size());
    else if (c == 'p' || c == 8) // set previous effect
      set_effect((_curr_effect_idx + effects.size() - 1) % effects.size());
    else if (c == 'u') { // set next user detection effect
      if (_curr_user_detection_effect == USER_DETECTION_NITE)
        _curr_user_detection_effect = USER_DETECTION_DEPTH_BLOB;
      else if (_curr_user_detection_effect == USER_DETECTION_DEPTH_BLOB)
        _curr_user_detection_effect = USER_DETECTION_DEPTH_BACKGROUND_REMOVER;
      else if (_curr_user_detection_effect == USER_DETECTION_DEPTH_BACKGROUND_REMOVER)
        _curr_user_detection_effect = USER_DETECTION_NITE;
      maggiePrint("Using user_detection_effect '%s'",
                  user_detection_effect_to_string(_curr_user_detection_effect).c_str());
    }
    else if ((int) c == 27)
#ifdef NITE_FX
      exit(0);
#else // not NITE_FX
      ros::shutdown();
#endif // not NITE_FX
  } // end image_callback();

  //////////////////////////////////////////////////////////////////////////////

  static void mouse_cb(int event, int x, int y, int flags, void* param) {
    EffectCollection* this_ptr = (EffectCollection*) param;
    this_ptr->effects[this_ptr->_curr_effect_idx]->mouse_cb
        (event, x / this_ptr->_resize_scale, y / this_ptr->_resize_scale);
  }

  //////////////////////////////////////////////////////////////////////////////

  //! change the current effect, by index
  inline void set_effect(int new_effect_idx) {
    _curr_effect_idx = new_effect_idx;
    maggiePrint("Using fn:%s", effects[_curr_effect_idx]->name());
    effects[_curr_effect_idx]->first_call();
  }

private:
  //! the list of all possible effects
  std::vector<EffectInterface*> effects;
  //! the index of the active effect
  int _curr_effect_idx;
  //! what to display
  cv::Mat3b image_out;
  cv::Mat3b image_out_scaled;
  double _resize_scale;
  std::string window_name;
  bool DISPLAY;

  enum UserDetectionEffect {
    USER_DETECTION_NITE = 0,
    USER_DETECTION_DEPTH_BLOB = 1,
    USER_DETECTION_DEPTH_BACKGROUND_REMOVER = 2
  };
  inline static const std::string user_detection_effect_to_string
  (const UserDetectionEffect u) {
    if (u == USER_DETECTION_NITE)
      return "NITE";
    else if (u == USER_DETECTION_DEPTH_BLOB)
      return "depth_blob";
    else if (u == USER_DETECTION_DEPTH_BACKGROUND_REMOVER)
      return "background_remover";
    else
      return "unknown";
  }

  UserDetectionEffect _curr_user_detection_effect;
  //! the list of all possible effects
  GetDepthBlobs get_depth_blobs_effect;
  DepthBackgroundRemover depth_bacground_remover_effect;
}; // end class EffectCollection

#endif // EFFECT_COLLECTION_H
