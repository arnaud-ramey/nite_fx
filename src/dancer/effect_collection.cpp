/*!
  \file        effect_collection.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2012/11/7

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
#include "nite_fx_path.h"

#include "effect_collection.h"

#include "background_remover.h"
#include "clone_user.h"
#include "compute_user_accelerations.h"
#include "copy_color_to_out_and_user_edge.h"
#include "copy_color_to_out.h"
#include "copy_depth_to_out.h"
#include "copy_user_to_out.h"
#include "equalize_color_to_out.h"
#include "keep_only_user_color_background.h"
#include "keep_only_user_video_background.h"
#include "particle_thrower.h"
#include "remove_user_inpaint.h"
#include "remove_user_inpaint_scale.h"
#include "remove_user_quick_fill.h"
#include "set_user_to_black.h"
#include "hue_to_out.h"
#include "blur.h"
#include "calibrator.h"
#include "helices.h"
// end of effect interfaces includes

void EffectCollection::add_all_effects() {
  // view inputs
  effects.push_back(new CopyColorToOut());
  effects.push_back(new CopyDepthToOut());
  effects.push_back(new CopyUserToOut());
  effects.push_back(new Calibrator());
  effects.push_back(new EqualizeColorToOut());
  effects.push_back(new HueToOut());
  effects.push_back(new CopyColorToOutAndUserEdge());

  // extract background
  effects.push_back(new KeepOnlyUserColorBackground(cv::Vec3b(0, 0, 0)));
  effects.push_back(new KeepOnlyUserColorBackground(cv::Vec3b(255, 255, 255)));
  // effects.push_back(new KeepOnlyUserVideoBackground(NITE_FX_PATH "video_backgrounds/blue_lines.m4v"));
  effects.push_back(new KeepOnlyUserVideoBackground(NITE_FX_PATH "video_backgrounds/news.m4v"));

  effects.push_back(new SetUserToBlack());
  effects.push_back(new RemoveUserQuickFill());
  effects.push_back(new RemoveUserInPaint());
  effects.push_back(new RemoveUserInPaintScale());
  effects.push_back(new CloneUser());
  effects.push_back(new BackgroundRemover());
  effects.push_back(new ComputeUserAccelerations());
  effects.push_back(new ParticleThrower());
  effects.push_back(new Blur());
  effects.push_back(new Helices());
  // end of effect interfaces instantiations
}
