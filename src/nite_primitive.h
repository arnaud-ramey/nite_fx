/**
  \file        nite_primitive.h
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

\class NitePrimitiveClass
A class that enables opening a connection with a Kinect device
and publishing its video and depth image streams.

In addition, by using NITE,
it also enables the detection of the users.

The image containing these different values is also published,
along with a message enclosing their 3D position.

\section Parameters
  - \b "rate"
        [int, Hz] (default: 30)
        The wanted FPS for output.

  - \b "camera_frame_id"
        [string] (default: "openni_depth_frame")
        What frame should be put in the messages sharing acquired images.

  - \b "display_images_flag"
        [bool] (default: false)
        If true, display the acquired RGB and depth images in windows.
        Not necesary for the acquisition of these images though.

  - \b "publish_images_flag"
        [bool] (default: false)
        If true, publish RGB, depth, user images on the dedicated topics
        (see below)

  - \b "publish_skeletons_flag"
        [bool] (default: false)
        If true, publish the skeleton message on the dedicated topic
        (see below)

  - \b "publish_transforms_flag"
        [bool] (default: false)
        If true, publish the TF transforms between each joint on the "tf" topic.

\section Subscriptions
  None

\section Publications
  - \b "rgb"
        [sensor_msgs/Image]
        The RGB image,
        published only if flag \a publish_images_flag is activated

  - \b "depth"
        [sensor_msgs/Image]
        The depth image,
        published only if flag \a publish_images_flag is activated

  - \b "user"
        [sensor_msgs/Image]
        The user image,
        published only if flag \a publish_images_flag is activated

  - \b "skeletons"
        [kinect::NiteSkeletonList]
        The list of skeletons,
        published only if flag \a publish_skeletons_flag is activated

 */

#ifndef NITE_PRIMITIVE_H
#define NITE_PRIMITIVE_H

#define NITE_FX
#include <vector>
#include "effect_collection.h"
#include "nite_fx_path.h"
// NITE
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
// AD
#include "skeleton_utils.h"
#include "user_image_to_rgb.h"


////////////////////////////////////////////////////////////////////////////////

//#define DEBUG

#ifdef DEBUG
//#define DEBUG_PRINT(...)          printf(__VA_ARGS__)
#define DEBUG_PRINT(...)          printf_THROTTLE(1, __VA_ARGS__)
#define DEBUG_TIMER_INIT          Timer timer;
//#define DEBUG_TIMER_PRINT(...)    timer.printTime(__VA_ARGS__)
#define DEBUG_TIMER_PRINT(...)    printf_THROTTLE(1, "Time for %s: %g ms", __VA_ARGS__, timer.time());
#else // no DEBUG
#define DEBUG_PRINT(...)          {}
#define DEBUG_TIMER_INIT          {}
#define DEBUG_TIMER_PRINT(...)    {}
#endif

#define CHECK_RC(nRetVal, what)  \
  if (nRetVal != XN_STATUS_OK) { \
  printf("%s failed: %s", what, xnGetStatusString(nRetVal)); \
  }

////////////////////////////////////////////////////////////////////////////////

class NitePrimitiveClass  {
public:
  typedef XnUserID UserId;
  typedef XnSkeletonJoint JointId;
  static const int QUEUE_SIZE = 10;


  //  //! ctor
  //  NitePrimitiveClass() {
  //    init();
  //  }

  //////////////////////////////////////////////////////////////////////////////

  void init_nite() {
    printf("init_nite()");
    g_bNeedPose   = FALSE;
    char empty_str[]="";
    strcpy (g_strPose,empty_str);


    // get params
    // std::string configFilename = "openni_tracker.xml";
    std::string configFilename = NITE_FX_PATH "data/openni_tracker.xml";
    rate = 30;
    display_images_flag = false;
    publish_images_flag = false;
    publish_skeletons_flag = false;
    publish_transforms_flag = false;

    // init nite
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);
    CHECK_RC(nRetVal, "Find image generator");

    // hardware_registration -> align depth on image
    g_DepthGenerator.GetAlternativeViewPointCap().SetViewPoint(g_ImageGenerator);

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    if (nRetVal != XN_STATUS_OK) {
      nRetVal = g_UserGenerator.Create(g_Context);
      CHECK_RC(nRetVal, "Find user generator");
    }

    // TODO set format here
    // g_ImageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);
    // g_ImageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT);
    // g_ImageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_YUV422);

    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
      printf("Supplied user generator doesn't support skeleton");
      // return 1;
    }

    XnCallbackHandle hUserCallbacks;
    g_UserGenerator.RegisterUserCallbacks
        (User_NewUser, User_LostUser, this, hUserCallbacks);

    XnCallbackHandle hCalibrationCallbacks;
    g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks
        (UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, this, hCalibrationCallbacks);

    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
      g_bNeedPose = TRUE;
      if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
        printf("Pose required, but not supported");
        // return 1;
      }

      XnCallbackHandle hPoseCallbacks;
      g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, this, hPoseCallbacks);

      g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }

    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");

    // publishers
    effect_collection.init();
    printf("NitePrimitive: rate:%i Hz, "
           "camera_frame_id:'%s', "
           "display_images_flag:%i, "
           "publish_images_flag:%i, "
           "publish_skeletons_flag:%i, "
           "publish_transforms_flag:%i",
           rate,
           camera_frame_id.c_str(),
           display_images_flag,
           publish_images_flag,
           publish_skeletons_flag,
           publish_transforms_flag);
  } // end init_nite();

  //////////////////////////////////////////////////////////////////////////////

  //! dtor
  ~NitePrimitiveClass() {
    g_Context.Shutdown();
  }

  //////////////////////////////////////////////////////////////////////////////

  void run() {
    printf("run()");
    skeleton_utils::Rate r(rate);

    while (ros::ok()) {
      DEBUG_PRINT("run loop");
      DEBUG_TIMER_INIT;
      g_Context.WaitAndUpdateAll();
      DEBUG_TIMER_PRINT("WaitAndUpdateAll()");

      get_userjoint_data();
      generate_cv_images();
      if (display_images_flag)
        display_images();
      effect_collection.fn(bgr8, depth32f, user8, skeleton_list_msg);
      r.sleep();
    }
  } // end run();

  //////////////////////////////////////////////////////////////////////////////

  // #define COPY_DATA // comment to share data between NITE and CV matrices (faster)

  inline void generate_cv_images() {
    DEBUG_PRINT("generate_cv_images()");

    // paint rgb
    // cf http://nma.web.nitech.ac.jp/fukushima/openni/SampleMultiKinect.cpp
    g_ImageGenerator.GetMetaData(rgbMD);
    XnUInt16 rgb_cols = rgbMD.XRes(), rgb_rows = rgbMD.YRes();
    //    printf("rgb:(%ix%i), PixelFormat:%i, BytesPerPixel:%i, DataSize:%i ",
    //           rgb_cols, rgb_rows,
    //           g_ImageGenerator.GetPixelFormat(),
    //           rgbMD.BytesPerPixel(),
    //           rgbMD.DataSize());
#ifdef COPY_DATA
    rgb8.create(rgb_rows,rgb_cols);
    // it seems calling WritableData() generates a segfault
    // cf http://openni-discussions.979934.n3.nabble.com/OpenNI-dev-Periodic-partial-disappearance-of-depth-image-as-a-result-of-SetViewpoint-amp-making-depts-td2270962.html
    memcpy(rgb8.data, rgbMD.WritableData(), rgb_rows * rgb_cols * rgbMD.BytesPerPixel());
#else
    // int rows, int cols, int type, void* data, size_t step=AUTO_STEP
    rgb8 = cv::Mat(rgb_rows, rgb_cols, CV_8UC3,
                   rgbMD.WritableData());
    //    (uchar*) rgbMD.WritableRGB24Data());
    //    (uchar*) g_ImageGenerator.GetRGB24ImageMap());
#endif
    cv::cvtColor(rgb8,bgr8,CV_RGB2BGR);

    // get depth metadata
    g_DepthGenerator.GetMetaData(depthMD);
    XnUInt16 depth_cols = depthMD.XRes(), depth_rows = depthMD.YRes();
    // printf("depth:(%ix%i)", depth_cols, depth_rows);
#ifdef COPY_DATA
    depth16.create(depth_rows, depth_cols);
    // it seems calling WritableData() generates a segfault
    // cf http://openni-discussions.979934.n3.nabble.com/OpenNI-dev-Periodic-partial-disappearance-of-depth-image-as-a-result-of-SetViewpoint-amp-making-depts-td2270962.html
    memcpy(depth16.data, depthMD.WritableData(), depth_rows * depth_cols * depthMD.BytesPerPixel());
    // memcpy(depth16.data, depthMD.Data(), rows * cols * depthMD.BytesPerPixel());
    depth16.convertTo(depth32f, CV_32FC1, 1.0 / 1000.0);
#else
    // int rows, int cols, int type, void* data, size_t step=AUTO_STEP
    depth16 = cv::Mat(depth_rows, depth_cols, CV_16UC1, depthMD.WritableData());
    depth16.convertTo(depth32f, CV_32FC1, 1.0 / 1000.0);
#endif

    // user image
    g_UserGenerator.GetUserPixels(0, userMD);
    XnUInt16 user_cols = userMD.XRes(), user_rows = userMD.YRes();
    // printf("user:(%ix%i)", user_cols, user_rows);
#ifdef COPY_DATA
    user16.create(user_rows,user_cols); //,CV_16UC1);
    // it seems calling WritableData() generates a segfault
    // cf http://openni-discussions.979934.n3.nabble.com/OpenNI-dev-Periodic-partial-disappearance-of-user-image-as-a-result-of-SetViewpoint-amp-making-depts-td2270962.html
    memcpy(user16.data, userMD.WritableData(), user_rows * user_cols * userMD.BytesPerPixel());
    // memcpy(user16.data, userMD.Data(), user_rows * user_cols * userMD.BytesPerPixel());
    // convert to uchar (8 bits)
    user8.create(user_rows,user_cols); //,CV_8UC1);
#else
    // int rows, int cols, int type, void* data, size_t step=AUTO_STEP
    user16 = cv::Mat(user_rows, user_cols, CV_16UC1, userMD.WritableData());
#endif
    user16.convertTo(user8, CV_8UC1);
  } // end generate_cv_images();

  //////////////////////////////////////////////////////////////////////////////

  //! Requires calling generate_cv_images() before
  inline void display_images() {
    DEBUG_PRINT("display_images()");

    // printf(1, "cols:%i, rows:%i", cols, rows);

    // paint the depth
    double minVal, maxVal;
    cv::minMaxLoc(depth16, &minVal, &maxVal);
    // printf("min:%g, max:%g", minVal, maxVal);
    depth16.convertTo(depth8_illus, CV_8U, 32.f / 1000);

    // paint the user
    user_image_to_rgb(user8, user_illus, 8);
    skeleton_utils::draw_skeleton_list(user_illus, skeleton_list_msg, 2);

    cv::imshow("depth8_illus", depth8_illus);
    cv::imshow("user_illus", user_illus);
    cv::imshow("bgr8", bgr8);
    cv::waitKey(5);
  }

  //////////////////////////////////////////////////////////////////////////////

  static void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, UserId nId, void* pCookie) {
    printf("New User %d", nId);
    NitePrimitiveClass* this_ptr = (NitePrimitiveClass*) pCookie;

    if (this_ptr->g_bNeedPose)
      this_ptr->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection
          (this_ptr->g_strPose, nId);
    else
      this_ptr->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
  }

  //////////////////////////////////////////////////////////////////////////////

  static void XN_CALLBACK_TYPE User_LostUser
  (xn::UserGenerator& generator, UserId nId, void* pCookie) {
    printf("Lost user %d", nId);
  }

  //////////////////////////////////////////////////////////////////////////////

  static void XN_CALLBACK_TYPE UserCalibration_CalibrationStart
  (xn::SkeletonCapability& capability, UserId nId, void* pCookie) {
    printf("Calibration started for user %d", nId);
  }

  //////////////////////////////////////////////////////////////////////////////

  static void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd
  (xn::SkeletonCapability& capability, UserId nId, XnBool bSuccess, void* pCookie) {
    NitePrimitiveClass* this_ptr = (NitePrimitiveClass*) pCookie;
    if (bSuccess) {
      printf("Calibration complete, start tracking user %d", nId);
      this_ptr->g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
    else {
      printf("Calibration failed for user %d", nId);
      if (this_ptr->g_bNeedPose)
        this_ptr->g_UserGenerator.GetPoseDetectionCap().StartPoseDetection
            (this_ptr->g_strPose, nId);
      else
        this_ptr->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  static void XN_CALLBACK_TYPE UserPose_PoseDetected
  (xn::PoseDetectionCapability& capability, XnChar const* strPose, UserId nId, void* pCookie) {
    printf("Pose %s detected for user %d", strPose, nId);
    NitePrimitiveClass* this_ptr = (NitePrimitiveClass*) pCookie;
    this_ptr->g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    this_ptr->g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline bool add_userjoint_data(const UserId & user_id,
                                 const JointId & joint_id_xn)
  {
    UserJointData out;
    // build child_frame_id
    std::ostringstream frame_str;
    frame_str << joint_id_converter.direct_search(joint_id_xn) << "_" << user_id;
    out.child_frame_id = frame_str.str();

    // get position
    XnSkeletonJointPosition joint_position;
    if (g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition
        (user_id, joint_id_xn, joint_position) != XN_STATUS_OK)
      return false;
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    // get orientation (quaternion)
    XnSkeletonJointOrientation joint_orientation;
    if (g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation
        (user_id, joint_id_xn, joint_orientation) != XN_STATUS_OK)
      return false;

    // confidence as average of position and orientation
    out.position_confidence = joint_position.fConfidence;
    out.orientation_confidence = joint_orientation.fConfidence;

    // skip computing
    out.transform.translation.x = -x;
    out.transform.translation.y = -y;
    out.transform.translation.z = z;
    out.transform.rotation.x = 0;
    out.transform.rotation.y = 0;
    out.transform.rotation.z = 0;
    out.transform.rotation.w = 1;
    _userjoint_data[user_id][joint_id_xn]= out;
    return true;
  } // end get_userjoint_data_joint_transform();

  //////////////////////////////////////////////////////////////////////////////

  void get_userjoint_data() {
    DEBUG_PRINT("get_userjoint_data()");
    UserId users[15];
    XnUInt16 nusers = 15;
    g_UserGenerator.GetUsers(users, nusers);
    _userjoint_data.clear();
    std::string j_name;
    JointId j_id;

    for (int user_counter = 0; user_counter < nusers; ++user_counter) {
      UserId curr_user_id = users[user_counter];
      if (!g_UserGenerator.GetSkeletonCap().IsTracking(curr_user_id))
        continue;

      add_userjoint_data(curr_user_id, XN_SKEL_HEAD);
      add_userjoint_data(curr_user_id, XN_SKEL_NECK);
      add_userjoint_data(curr_user_id, XN_SKEL_TORSO);
      // get_userjoint_data(curr_user_id, XN_SKEL_WAIST);

      // get_userjoint_data(curr_user_id, XN_SKEL_LEFT_COLLAR);
      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_SHOULDER);
      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_ELBOW);
      // get_userjoint_data(curr_user_id, XN_SKEL_LEFT_WRIST);
      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_HAND);
      // get_userjoint_data(curr_user_id, XN_SKEL_LEFT_FINGERTIP);

      // get_userjoint_data(curr_user_id, XN_SKEL_RIGHT_COLLAR);
      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_SHOULDER);
      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_ELBOW);
      // get_userjoint_data(curr_user_id, XN_SKEL_RIGHT_WRIST);
      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_HAND);
      // get_userjoint_data(curr_user_id, XN_SKEL_RIGHT_FINGERTIP);

      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_HIP);
      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_KNEE);
      // get_userjoint_data(curr_user_id, XN_SKEL_LEFT_ANKLE);
      add_userjoint_data(curr_user_id, XN_SKEL_LEFT_FOOT);

      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_HIP);
      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_KNEE);
      // get_userjoint_data(curr_user_id, XN_SKEL_RIGHT_ANKLE);
      add_userjoint_data(curr_user_id, XN_SKEL_RIGHT_FOOT);
    } // end loop user_counter
  } // end get_userjoint_data();

  //////////////////////////////////////////////////////////////////////////////

private:
  int rate;
  std::string camera_frame_id;
  // images stuff
  bool publish_images_flag;
  xn::SceneMetaData userMD;
  xn::DepthMetaData depthMD;
  xn::ImageMetaData rgbMD;

  //! dp16: unsigned short -> unsigned: 0 to 65535 - CV_16U
  cv::Mat1w depth16;
  cv::Mat1f depth32f;
  cv::Mat1b depth8_illus;

  cv::Mat3b rgb8;
  cv::Mat3b bgr8;

  cv::Mat3b user_illus;
  cv::Mat1w user16;
  cv::Mat1b user8;

  xn::Context        g_Context;
  xn::DepthGenerator g_DepthGenerator;
  xn::ImageGenerator g_ImageGenerator;
  xn::UserGenerator  g_UserGenerator;
  XnBool g_bNeedPose;
  XnChar g_strPose[20];

  // ros::Time
  skeleton_utils::Time messages_timestamp;

  //! true for displaying input
  bool display_images_flag;

  // skeletons and TF stuff
  struct UserJointData {
    tf::Transform transform;
    double position_confidence;
    double orientation_confidence;
    std::string child_frame_id;
  };
  std::map<UserId, std::map<JointId, UserJointData> > _userjoint_data;
  //! true for publishing skeleton
  bool publish_transforms_flag;
  //! true for publishing skeleton TF (transforms)
  bool publish_skeletons_flag;
  //! the message that will be filled with skeleton
  kinect::NiteSkeletonList skeleton_list_msg;
  //! convert joint ID to string
  skeleton_utils::JointId2StringConverter joint_id_converter;
  //! the skeleton publisher

  EffectCollection effect_collection;
}; // end class NitePrimitiveClass

#endif // NITE_PRIMITIVE_H
