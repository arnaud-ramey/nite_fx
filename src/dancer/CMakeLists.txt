SET(EFFECT_LIST
  effect_interface.h
  # user detection effects
  get_depth_blobs.h
  depth_background_remover.h

  # effects
  copy_color_to_out.h
  copy_depth_to_out.h
  copy_color_to_out_and_user_edge.h
  keep_only_user_color_background.h
  set_user_to_black.h
  remove_user_quick_fill.h
  remove_user_inpaint.h
  remove_user_inpaint_scale.h
  clone_user.h
  background_remover.h
  effect_collection.h
  copy_user_to_out.h
  compute_user_accelerations.h
  particle_thrower.h
  keep_only_user_video_background.h
  equalize_color_to_out.h
  hue_to_out.h
  blur.h
  calibrator.h
  helices.h
  ### end of effect interfaces
)

add_maggie_lib(effect_collection
  SOURCES      effect_collection.cpp ${EFFECT_LIST}

)
target_link_libraries(effect_collection disjoint_sets2 string_utils)

### effect_collection_nite_fx : effect_collection with no ROS
add_library(effect_collection_nite_fx effect_collection_nite_fx.cpp ${EFFECT_LIST})

### nite_fx: as a library, for compiling it (and making it visible in QtCreator),
###, but not complaining about linking problems with ROS
rosbuild_add_executable(nite_fx   nite_fx.cpp)
target_link_libraries(nite_fx     effect_collection OpenNI XnVNite disjoint_sets2)

rosbuild_add_executable(nite_effect_collection_standalone   nite_effect_collection_standalone.cpp)
target_link_libraries(nite_effect_collection_standalone  effect_collection OpenNI XnVNite)

ADD_SUBDIRECTORY(flying_game)
