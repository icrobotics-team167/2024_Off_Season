// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionPoseEstimator {
  private final VisionPoseEstimatorIO io;
  private final VisionPoseEstimatorInputsAutoLogged inputs =
      new VisionPoseEstimatorInputsAutoLogged();

  public VisionPoseEstimator(VisionPoseEstimatorIO io, Supplier<ChassisSpeeds> speedsSupplier) {
    this.io = io;
    this.io.addVelocityDataSource(speedsSupplier);
  }

  public record PoseEstimate(Pose2d pose, Vector<N3> stdDevs, double timestamp) {}

  public PoseEstimate[] poll() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    var estimates = new PoseEstimate[inputs.timestamps.length];
    for (int i = 0; i < inputs.timestamps.length; i++) {
      estimates[i] =
          new PoseEstimate(
              inputs.poses[i],
              VecBuilder.fill(
                  inputs.translationalStDevs[i],
                  inputs.translationalStDevs[i],
                  inputs.rotationalStDevs[i]),
              inputs.timestamps[i]);
    }
    return estimates;
  }
}
