INCLUDE_DIRECTORIES(dancer ${CMAKE_CURRENT_SOURCE_DIR})

ADD_LIBRARY(disjoint_sets2 disjoint_sets2.cpp disjoint_sets2.h)
ADD_LIBRARY(effect_collection_nite_fx dancer/effect_collection_nite_fx.cpp)
TARGET_LINK_LIBRARIES( effect_collection_nite_fx
                        ${OpenCV_LIBS}  ${Boost_LIBRARIES} disjoint_sets2)

ADD_EXECUTABLE( nite_fx nite_fx.cpp nite_primitive.h)
TARGET_LINK_LIBRARIES( nite_fx OpenNI XnVNite effect_collection_nite_fx)


