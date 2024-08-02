// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionPoseEstimatorIOPhoton implements VisionPoseEstimatorIO {
  private final PhotonPoseEstimator[] poseEstimators;
  private Supplier<ChassisSpeeds> velocities;

  public VisionPoseEstimatorIOPhoton() throws IOException {
    String[] cameras = new String[] {"LeftCam", "RightCam"};

    poseEstimators = new PhotonPoseEstimator[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      poseEstimators[i] =
          new PhotonPoseEstimator(
              AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              new PhotonCamera(cameras[i]),
              new Transform3d());
      poseEstimators[i].setMultiTagFallbackStrategy(
          PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }
  }

  @Override
  public void updateInputs(VisionPoseEstimatorInputs inputs) {
    inputs.timestamps = new double[poseEstimators.length];
    inputs.poses = new Pose2d[poseEstimators.length];
    inputs.translationalStDevs = new double[poseEstimators.length];
    inputs.rotationalStDevs = new double[poseEstimators.length];

    for (int i = 0; i < poseEstimators.length; i++) {
      Optional<EstimatedRobotPose> data = poseEstimators[i].update();
      if (data.isPresent()) {
        EstimatedRobotPose poseEstimate = data.get();
        inputs.timestamps[i] = poseEstimate.timestampSeconds;
        inputs.poses[i] = poseEstimate.estimatedPose.toPose2d();

        double[] stDevs = calculateStDevs(velocities.get());
        inputs.translationalStDevs[i] = stDevs[0];
        inputs.rotationalStDevs[i] = stDevs[1];
      }
    }
  }

  @Override
  public void addVelocityDataSource(Supplier<ChassisSpeeds> velocities) {
    this.velocities = velocities;
  }

  private double[] calculateStDevs(ChassisSpeeds velocities) {
    // TODO: Implement
    return new double[] {.9, .9};
  }
}
