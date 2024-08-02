// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class VisionPoseEstimator {
  private final VisionPoseEstimatorIO io;
  private final VisionPoseEstimatorInputsAutoLogged inputs =
      new VisionPoseEstimatorInputsAutoLogged();

  public VisionPoseEstimator(VisionPoseEstimatorIO io, Supplier<ChassisSpeeds> speedsSupplier) {
    this.io = io;
    this.io.addVelocityDataSource(speedsSupplier);
  }

  @FunctionalInterface
  public interface EstimateConsumer {
    void useEstimate(
        Pose2d pose, double timestamp, double translationalStDevs, double angularStDevs);
  }

  /**
   * Poll the coprocessor(s) for the latest data
   *
   * @param estimateConsumer The consumer for handling that data. Takes in a pose, a timestamp, a
   *     translational standard deviation, and a rotational standard deviation, in that order.
   */
  public void poll(EstimateConsumer estimateConsumer) {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    for (int i = 0; i < inputs.timestamps.length; i++) {
      estimateConsumer.useEstimate(
          inputs.poses[i],
          inputs.timestamps[i],
          inputs.translationalStDevs[i],
          inputs.rotationalStDevs[i]);
    }
  }
}
