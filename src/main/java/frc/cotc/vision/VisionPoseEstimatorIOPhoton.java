// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionPoseEstimatorIOPhoton implements VisionPoseEstimatorIO {
  private static final AprilTagFieldLayout tags;

  static {
    tags = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  }

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final Transform3d cameraPos;

  public VisionPoseEstimatorIOPhoton(String name, Transform3d position) {
    camera = new PhotonCamera(name);
    poseEstimator =
        new PhotonPoseEstimator(
            tags, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, position);
    cameraPos = position;
  }

  @Override
  public void updateInputs(VisionPoseEstimatorIOInputs inputs) {
    var estimateList = new ArrayList<PoseEstimate>();

    var results = camera.getAllUnreadResults();
    for (var result : results) {
      poseEstimator
          .update(result)
          .ifPresent(
              (estimate) -> {
                var tagPoses = new Pose3d[estimate.targetsUsed.size()];
                for (int i = 0; i < tagPoses.length; i++) {
                  tagPoses[i] =
                      estimate
                          .estimatedPose
                          .plus(cameraPos)
                          .plus(estimate.targetsUsed.get(i).bestCameraToTarget);
                }
                estimateList.add(
                    new PoseEstimate(estimate.estimatedPose, estimate.timestampSeconds, tagPoses));
              });
    }

    inputs.poseEstimates = estimateList.toArray(new PoseEstimate[0]);
  }
}
