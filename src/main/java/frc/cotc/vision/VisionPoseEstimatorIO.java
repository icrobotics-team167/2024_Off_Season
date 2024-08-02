// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface VisionPoseEstimatorIO {
  @AutoLog
  class VisionPoseEstimatorInputs {
    double[] timestamps;
    Pose2d[] poses;
    double[] translationalStDevs;
    double[] rotationalStDevs;
  }

  default void updateInputs(VisionPoseEstimatorInputs inputs) {}

  /**
   * Reliability/noise of data can be affected by robot movement, so standard deviation estimates
   * should account for that.
   *
   * @param velocities A supplier for the robot's current velocity.
   */
  default void addVelocityDataSource(Supplier<ChassisSpeeds> velocities) {}
}
