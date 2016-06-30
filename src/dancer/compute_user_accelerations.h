/*!
  \file        compute_user_accelerations.h
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

#ifndef COMPUTE_USER_ACCELERATIONS_H
#define COMPUTE_USER_ACCELERATIONS_H

#include "effect_interface.h"

//#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "geometry_utils.h"
#include "drawing_utils.h"

////////////////////////////////////////////////////////////////////////////////

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class ComputeUserAccelerations : virtual public EffectInterface {
public:
  static const unsigned int MIN_CONTOUR_SIZE = 10;
  static const double MIN_ACC_NORM = 10;
  static const double ACC_DRAWING_SCALE = 1; // pixels per meters

  ComputeUserAccelerations() :
    draw_img_flag(true) {}

  static void simplify_contour(std::vector<cv::Point> & contour) {
    for (unsigned int pt_idx = 0; pt_idx < contour.size() - 2; ++pt_idx) {
      bool need_delete = false;
      // minimum distance
      if (geometry_utils::distance_points_squared(contour[pt_idx], contour[pt_idx + 1])
          < 15 * 15)
        need_delete = true;
      else {
        // open angle
        double angle = geometry_utils::absolute_angle_between_three_points
            (contour[pt_idx], contour[pt_idx + 1], contour[pt_idx + 2]);
        if (fabs(angle - M_PI) < M_PI / 6)
          need_delete = true;
      }

      if (need_delete) { // remove pt_idx + 1
        contour.erase(contour.begin() + pt_idx + 1);
        pt_idx--; // rewind
      }
    } // end loop pt
  } // end simplify_contour();

  //////////////////////////////////////////////////////////////////////////////

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    // reset variables
    img_out.create(color.size());
    contours.clear();
    concatenated_contour.clear();
    simplified_contour.clear();
    accelerations.clear();

    // find contours of user of interest
    user_mask = (user != 0);
    cv::findContours(user_mask, contours, // a vector of contours
                     // CV_RETR_EXTERNAL, // retrieve the external contours
                     CV_RETR_LIST, // retrieve the external contours
                     // CV_CHAIN_APPROX_NONE
                     CV_CHAIN_APPROX_TC89_L1
                     ); // all pixels of each contours

    if (contours.size() == 0) { // no contours -> do nothing
      draw_img_out(img_out);
      return;
    }

    // concatenate all contours
    // first determine the size of the concatenated_contour
    int concatenated_contour_size = 0;
    for (unsigned int contour_idx = 0; contour_idx < contours.size(); ++contour_idx)
      concatenated_contour_size += contours[contour_idx].size();
    concatenated_contour.clear();
    concatenated_contour.reserve(concatenated_contour_size);
    // then really add points
    for (unsigned int contour_idx = 0; contour_idx < contours.size(); ++contour_idx) {
      // do not insert if contour to small
      if (contours[contour_idx].size() < MIN_CONTOUR_SIZE)
        continue;
      concatenated_contour.insert
          (concatenated_contour.end(),
           contours[contour_idx].begin(), contours[contour_idx].end());
    }
    maggieDebug3("concatenated_contour of size %i", concatenated_contour.size());

    // simplify contour
    //cv::approxPolyDP(cv::Mat(concatenated_contour), simplified_contour, 5, false);
    simplified_contour = concatenated_contour;
    simplify_contour(simplified_contour);
    if (simplified_contour.size() == 0) {
      maggiePrint("simplified_contour.size() == 0");
      draw_img_out(img_out);
      return;
    }
    maggieDebug3("approxPolyDP size %i -> %i",
                      concatenated_contour.size(), simplified_contour.size());

    // do not determine accelerations if previous_simplified_contour empty
    if (previous_simplified_contour.size() == 0) {
      previous_simplified_contour = simplified_contour;
      draw_img_out(img_out);
      return;
    }

    // determine accelerations
    cv::Point* curr_pt = &(simplified_contour[0]);
    for (unsigned int curr_pt_idx = 0; curr_pt_idx < simplified_contour.size(); ++curr_pt_idx) {
      double curr_depth = depth(*curr_pt);
      // find the closest point from the previous_simplified_contour
      cv::Point closest_prev_pt;
#if 0
      double min_dist_sq = std::numeric_limits<double>::max();
      cv::Point* prev_pt = &(previous_simplified_contour[0]);
      for (unsigned int prev_pt_idx = 0; prev_pt_idx < previous_simplified_contour.size(); ++prev_pt_idx) {
        double curr_dist_sq = geometry_utils::distance_points_squared
            (*curr_pt, *prev_pt);
        if (min_dist_sq > curr_dist_sq) {
          min_dist_sq = curr_dist_sq;
          closest_prev_pt = *prev_pt;
        }
        ++prev_pt;
      } // end loop prev_pt_idx
#else
      double min_dist_sq;
      geometry_utils::distance_point_polygon_squared
          (*curr_pt, previous_simplified_contour, closest_prev_pt, min_dist_sq, false);
#endif

      // now that we have the closest point from previous_simplified_contour
      // add the acceleration if it is strong enough
      double curr_acc = sqrt(min_dist_sq) * curr_depth;
      if (curr_acc > MIN_ACC_NORM) {
        Acceleration new_acc;
        new_acc.origin =  *curr_pt;
        new_acc.orien = geometry_utils::oriented_angle_of_a_vector
            (*curr_pt - closest_prev_pt);
        // (closest_prev_pt - *curr_pt);
        new_acc.norm = curr_acc;
        accelerations.push_back(new_acc);
      }

      ++curr_pt;
    } // end loop curr_pt_idx

    // store previous_simplified_contour
    previous_simplified_contour = simplified_contour;

    draw_img_out(img_out);
  } // end fn();

  //////////////////////////////////////////////////////////////////////////////

  inline void draw_img_out(cv::Mat3b & img_out) const {
    if (!draw_img_flag)
      return;

    // clear img_out
    img_out.setTo(0);

    // draw concatenated_contour
    image_utils::drawListOfPoints(img_out, concatenated_contour, cv::Vec3b(255, 255, 255));
    //    cv::drawContours(img_out, contours, -1, // draw all contours
    //                     cv::Scalar::all(255), // in white
    //                     2); // with a thickness of 2

    // draw simplified_contour
    // image_utils::drawPolygon(img_out, simplified_contour, false,  CV_RGB(255, 0, 0), 1);
    // image_utils::drawListOfPoints(img_out, simplified_contour, cv::Vec3b(0, 0, 255));
    for (unsigned int pt_idx = 0; pt_idx < simplified_contour.size(); ++pt_idx)
      image_utils::drawPoint(img_out, simplified_contour[pt_idx], CV_RGB(0, 255, 0), 3);

    // draw accelerations
    if (accelerations.size() == 0)
      return;
    const Acceleration* curr_acc = &(accelerations[0]);
    for (unsigned int acc_idx = 0; acc_idx < accelerations.size(); ++acc_idx) {
      //      cv::line(img_out, curr_acc->origin,
      //               cv::Point(curr_acc->origin.x + ACC_DRAWING_SCALE * curr_acc->norm * cos(curr_acc->orien),
      //                         curr_acc->origin.y + ACC_DRAWING_SCALE * curr_acc->norm * sin(curr_acc->orien)),
      //               CV_RGB(255, 0, 0), 2);
      image_utils::draw_arrow
          (img_out, curr_acc->origin,
           cv::Point(curr_acc->origin.x + ACC_DRAWING_SCALE * curr_acc->norm * cos(curr_acc->orien),
                     curr_acc->origin.y + ACC_DRAWING_SCALE * curr_acc->norm * sin(curr_acc->orien)),
           CV_RGB(255, 0, 0), 2);
      ++ curr_acc;
    } // end loop acc_idx
  } // end draw_img_out();

  const char* name() const { return "ComputeUserAccelerations"; }

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Point> concatenated_contour;
  std::vector<cv::Point> simplified_contour;
  std::vector<cv::Point> previous_simplified_contour;
  cv::Mat1b user_mask;

  struct Acceleration {
    cv::Point origin;
    double orien; //! radians
    double norm; //! meters
  };
  std::vector<Acceleration> accelerations;

  bool draw_img_flag;
}; // end class ComputeUserAccelerations

#endif // COMPUTE_USER_ACCELERATIONS_H
