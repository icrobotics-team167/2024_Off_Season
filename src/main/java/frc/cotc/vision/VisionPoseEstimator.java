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
import org.littletonrobotics.junction.Logger;

public class VisionPoseEstimator {
  @FunctionalInterface
  public interface EstimateConsumer {
    void useEstimate(
        Pose2d pose, double timestamp, double translationalStDevs, double angularStDevs);
  }

  private final VisionPoseEstimatorIO io;
  private final VisionPoseEstimatorInputsAutoLogged inputs =
      new VisionPoseEstimatorInputsAutoLogged();
  private final EstimateConsumer estimateConsumer;

  public VisionPoseEstimator(
      VisionPoseEstimatorIO io,
      Supplier<ChassisSpeeds> speedsSupplier,
      EstimateConsumer estimateConsumer) {
    this.io = io;
    this.io.addVelocityDataSource(speedsSupplier);
    this.estimateConsumer = estimateConsumer;
  }

  public void poll() {
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
