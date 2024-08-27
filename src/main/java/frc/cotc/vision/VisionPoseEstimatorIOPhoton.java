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
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionPoseEstimatorIOPhoton implements VisionPoseEstimatorIO {
  private final PhotonPoseEstimator[] poseEstimators;
  private Supplier<ChassisSpeeds> velocities;

  public VisionPoseEstimatorIOPhoton() {
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

    velocities = () -> null;
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

        double[] stDevs = calculateStDevs(velocities.get(), poseEstimate.targetsUsed);
        inputs.translationalStDevs[i] = stDevs[0];
        inputs.rotationalStDevs[i] = stDevs[1];
      }
    }
  }

  @Override
  public void addVelocityDataSource(Supplier<ChassisSpeeds> velocities) {
    this.velocities = velocities;
  }

  /**
   * Estimates the standard deviations of a measured robot position.
   *
   * @param velocities A ChassisSpeeds representing the robot's current speeds. Can be null.
   * @param tags The tracked tags used to calculate the position.
   * @return A double array of length 2. Index 0 is translational standard dev, 1 is rotational.
   */
  private double[] calculateStDevs(ChassisSpeeds velocities, List<PhotonTrackedTarget> tags) {
    if (velocities == null) {
      return calculateStDevs(new ChassisSpeeds(2, 0, 2), tags);
    }

    // Calculate a reliability score for each tag.
    // Lower is better.
    double[] translationalReliabilityScores = new double[tags.size()];
    double[] rotationalReliabilityScores = new double[tags.size()];
    for (int i = 0; i < tags.size(); i++) {
      PhotonTrackedTarget tag = tags.get(i);

      // Reliability score is based on 3 things:
      // 1) The square of distance. The farther it is, the smaller it appears, and thus there's
      // fewer pixels to work with
      double tagDistance = tag.getBestCameraToTarget().getTranslation().getNorm();
      // 2) The relative linear movement. The farther it is, the slower it appears when moving
      // laterally, so motion blur is less of a factor.
      // (m/s) / atan(m)
      double linearRelativeMovement =
          Math.hypot(velocities.vxMetersPerSecond, velocities.vyMetersPerSecond)
              / Math.atan(tagDistance);
      // 3) The relative angular movement. The farther it is, the faster it appears when rotating,
      // so motion blur is more of a factor.
      // m * rad/sec
      double angularRelativeMovement = tagDistance * velocities.omegaRadiansPerSecond;
      // Reliability score is separated into translational and rotational components, as rotational
      // tracking is not only dramatically less reliable than translational tracking when using
      // SolvePnP, it also tends to scale worse with movement and distance.
      // TODO: Tune weights
      translationalReliabilityScores[i] =
          .2 * Math.pow(tagDistance, 2)
              + linearRelativeMovement * .25
              + angularRelativeMovement * .3;
      rotationalReliabilityScores[i] =
          .4 * Math.pow(tagDistance, 2)
              + linearRelativeMovement * .3
              + angularRelativeMovement * .6;
    }

    // Return standard deviations based on calculated overall scores.
    return new double[] {
      getOverallScore(translationalReliabilityScores), getOverallScore(rotationalReliabilityScores)
    };
  }

  /**
   * Estimate the standard deviations of a measured robot pose based on a weighted sum of the min,
   * mean, and max reliability scores.
   *
   * @param reliabilityScores The scores for each tag
   * @return The standard deviations.
   */
  private double getOverallScore(double[] reliabilityScores) {
    double minScore = Double.POSITIVE_INFINITY;
    double meanScore = 0;
    double maxScore = Double.NEGATIVE_INFINITY;
    for (double score : reliabilityScores) {
      minScore = Math.min(minScore, score);
      meanScore += score;
      maxScore = Math.max(maxScore, score);
    }
    meanScore /= reliabilityScores.length;

    // Make an overall score based on a weighted sum of the min, max, and avg scores.
    // TODO: Also tune these weights
    double overallScore = minScore * .1 + maxScore * .25 + meanScore * .65;

    // Scale overall score inversely with tag count
    overallScore *= Math.pow(.8, reliabilityScores.length - 1);
    return overallScore;
  }
}
