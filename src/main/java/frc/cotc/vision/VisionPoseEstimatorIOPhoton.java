// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionPoseEstimatorIOPhoton implements VisionPoseEstimatorIO {
  private final PhotonCamera[] cameras;
  private final PhotonPoseEstimator[] poseEstimators;

  public VisionPoseEstimatorIOPhoton() {
    String[] cameraNames = new String[] {"LeftCam", "RightCam"};
    // TODO: Measure
    Transform3d[] cameraLocations = new Transform3d[] {new Transform3d(), new Transform3d()};

    cameras = new PhotonCamera[cameraNames.length];
    for (int i = 0; i < cameraNames.length; i++) {
      try (var camera = new PhotonCamera(cameraNames[i])) {
        cameras[i] = camera;
      } catch (Exception e) {
        throw new RuntimeException(
            "Could not initialize Photon camera \"" + cameraNames[i] + "\" with exception: " + e);
      }
    }

    poseEstimators = new PhotonPoseEstimator[cameras.length];
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    for (int i = 0; i < cameras.length; i++) {
      poseEstimators[i] =
          new PhotonPoseEstimator(
              fieldLayout,
              PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              cameraLocations[i]);
      poseEstimators[i].setMultiTagFallbackStrategy(
          PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }
  }

  @Override
  public void updateInputs(VisionPoseEstimatorInputs inputs, ChassisSpeeds speeds) {
    var poseEstimates = new ArrayList<VisionPoseEstimator.PoseEstimate>();

    for (int i = 0; i < cameras.length; i++) {
      var latestResults = cameras[i].getAllUnreadResults();
      for (var result : latestResults) {
        poseEstimators[i]
            .update(result)
            .ifPresent(
                (estimate) -> {
                  // TODO: Make something that can estimate std devs on the fly
                  poseEstimates.add(
                      new VisionPoseEstimator.PoseEstimate(
                          estimate.estimatedPose.toPose2d(),
                          .1,
                          .1,
                          .1,
                          estimate.timestampSeconds));
                });
      }
    }

    inputs.poseEstimates = poseEstimates.toArray(new VisionPoseEstimator.PoseEstimate[0]);
  }
}
