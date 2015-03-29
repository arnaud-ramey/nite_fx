/*!
  \file        particle_thrower.h
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

\class ParticleThrower
\brief A \a EffectInterface that enables throwing small colorful particles
thanks to the motion of the users.

 */

#ifndef PARTICLE_THROWER_H
#define PARTICLE_THROWER_H

#include "compute_user_accelerations.h"
#include "timer.h"
#include "color_utils.h"

// make virtual inheritance to avoid "the diamond of death"
// http://en.wikipedia.org/wiki/Diamond_problem#The_diamond_problem
class ParticleThrower : virtual public ComputeUserAccelerations {
public:
  ParticleThrower() {
    // do not draw out iamge for ComputeUserAccelerations
    ComputeUserAccelerations::draw_img_flag = false;
  }

  //////////////////////////////////////////////////////////////////////////////

  void fn(const cv::Mat3b & color, const cv::Mat1f & depth, const cv::Mat1b & user,
          const kinect::NiteSkeletonList & skeleton_list,
          cv::Mat3b & img_out) {
    ComputeUserAccelerations::fn(color, depth, user, skeleton_list, img_out);
    // create new particles
    for (unsigned int acc_idx = 0; acc_idx < accelerations.size(); ++acc_idx) {
      Acceleration* curr_acc = &(accelerations[acc_idx]);
      // if (drand48() > .9) {
      if (curr_acc->norm / 25 > 1) {
        Particle new_particle;
        new_particle.mass = 15 + drand48() * 15;
        new_particle.accel.y = new_particle.mass * Particle::G; // inversed y
        new_particle.speed.x = 10 * curr_acc->norm * cos(curr_acc->orien);
        new_particle.speed.y = 10 * curr_acc->norm * sin(curr_acc->orien);
        new_particle.position = curr_acc->origin;
        new_particle.color = color_utils::color<cv::Scalar>(-1);
        particles.push_back(new_particle);
      }
    } // end loop acc_idx
    maggieDebug2("accelerations: size:%i, particles: size %i",
                      accelerations.size(), particles.size());

    // update all particles
    double dt_sec = last_time_update.getTimeSeconds();
    for (unsigned int part_idx = 0; part_idx < particles.size(); ++part_idx) {
      particles[part_idx].update(dt_sec);
    } // end loop part_idx
    last_time_update.reset();

    // kill dead particles
    for (unsigned int part_idx = 0; part_idx < particles.size(); ++part_idx) {
      if (particles[part_idx].position.y > img_out.rows) {
        particles.erase(particles.begin() + part_idx);
        part_idx--;
      }
    } // end loop part_idx

    // draw
    img_out.create(color.size());
    img_out.setTo(0);
    // draw simplified_contour -> put it in an empty vector
    contours.clear();
    contours.push_back(simplified_contour);
    // cv::drawContours(img_out, contours, -1, CV_RGB(50, 0, 0), -1);
    image_utils::drawListOfPoints(img_out, concatenated_contour, cv::Vec3b(255, 255, 255));

    for (unsigned int part_idx = 0; part_idx < particles.size(); ++part_idx)
      draw_particle(particles[part_idx], img_out);
  } // end fn();

  //////////////////////////////////////////////////////////////////////////////

protected:
  Timer last_time_update;

  struct Particle {
    static const double G = 9.81;
    void update(double dt_sec) {
      speed.x += dt_sec * accel.x;
      speed.y += dt_sec * accel.y;
      position.x += dt_sec * speed.x;
      position.y += dt_sec * speed.y;
    }
    double mass;
    cv::Point2f position, speed, accel;
    cv::Scalar color;
  };
  std::vector<Particle> particles;

  //////////////////////////////////////////////////////////////////////////////

  virtual void draw_particle(const Particle & p, cv::Mat3b & img_out) {
    // body
    cv::circle(img_out, cv::Point(p.position.x, p.position.y),
               2 + p.mass / 10, p.color, -1);
    // tail
    //    cv::line(img_out, cv::Point(p.position.x, p.position.y),
    //             cv::Point(p.position.x + p.speed.x, p.position.y + p.speed.y),
    //             CV_RGB(200, 0, 0), 1);
  }

  //////////////////////////////////////////////////////////////////////////////

  const char* name() const { return "ParticleThrower"; }
}; // end class ParticleThrower

#endif // PARTICLE_THROWER_H
