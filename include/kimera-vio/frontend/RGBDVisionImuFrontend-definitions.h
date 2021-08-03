/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionImuFrontend-definitions.h
 * @brief  Definitions for StereoVisionImuFrontend
 * @author Antoni Rosinol
 */

#pragma once

#include "kimera-vio/frontend/FrontendOutputPacketBase.h"
#include "kimera-vio/frontend/rgbd/RgbdFrame.h"
#include "kimera-vio/frontend/VisionImuFrontend-definitions.h"

namespace VIO {

struct RGBDFrontendOutput : public FrontendOutputPacketBase {
 public:
  KIMERA_POINTER_TYPEDEFS(RGBDFrontendOutput);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RGBDFrontendOutput);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RGBDFrontendOutput(
      const bool is_keyframe,
      // const StatusStereoMeasurementsPtr& status_stereo_measurements,
      const TrackingStatus& tracker_status,
      const gtsam::Pose3& relative_pose_body_cam,

      const RgbdFrame& rgbd_frame_lkf,
      // Use rvalue reference: FrontendOutput owns pim now.
      const ImuFrontend::PimPtr& pim,
      const ImuAccGyrS& imu_acc_gyrs,
      // const cv::Mat& feature_tracks,
      const DebugTrackerInfo& debug_tracker_info)
      : FrontendOutputPacketBase(rgbd_frame_lkf.timestamp_,
                                 is_keyframe,
                                 FrontendType::kStereoImu,
                                 pim,
                                 imu_acc_gyrs,
                                 debug_tracker_info),
        // status_stereo_measurements_(status_stereo_measurements),
        tracker_status_(tracker_status),
        relative_pose_body_cam_(relative_pose_body_cam),
        rgbd_frame_lkf_(rgbd_frame_lkf){}

  virtual ~RGBDFrontendOutput() = default;

 public:
  // const StatusStereoMeasurementsPtr status_stereo_measurements_;
  const TrackingStatus tracker_status_;
  const gtsam::Pose3 relative_pose_body_cam_;
  const RgbdFrame rgbd_frame_lkf_;
  // const cv::Mat feature_tracks_;
};

}  // namespace VIO
