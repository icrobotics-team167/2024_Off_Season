// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionPoseEstimatorIO {
  class VisionPoseEstimatorInputs implements LoggableInputs {
    VisionPoseEstimator.PoseEstimate[] poseEstimates = new VisionPoseEstimator.PoseEstimate[0];

    @Override
    public void toLog(LogTable table) {
      table.put("poseEstimates", VisionPoseEstimator.PoseEstimate.struct, poseEstimates);
    }

    @Override
    public void fromLog(LogTable table) {
      poseEstimates = table.get("poseEstimates", VisionPoseEstimator.PoseEstimate.struct);
    }
  }

  default void updateInputs(VisionPoseEstimatorInputs inputs, ChassisSpeeds speeds) {}
}
