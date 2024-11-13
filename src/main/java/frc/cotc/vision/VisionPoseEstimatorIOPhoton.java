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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionPoseEstimatorIOPhoton implements VisionPoseEstimatorIO {
  private static final AprilTagFieldLayout tags;

  static {
    tags = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  }

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final Transform3d cameraPos;
  private final VisionTuningAutoLogged tuning;

  public VisionPoseEstimatorIOPhoton(int id) {
    String name;
    Transform3d cameraPosition;
    switch (id) {
      case 0 -> {
        name = "FrontLeftCamera";
        cameraPosition =
            new Transform3d(
                .1,
                .1,
                .1,
                new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(30)));
        tuning = new VisionTuningAutoLogged();
        tuning.constantValue = .005;
        tuning.relativeAreaScalar = .075;
        tuning.dotProductScalar = .3;
        tuning.tagCountExponent = 1.2;
      }
      case 1 -> {
        name = "FrontRightCamera";
        cameraPosition =
            new Transform3d(
                .1,
                -.1,
                .1,
                new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(-30)));
        tuning = new VisionTuningAutoLogged();
        tuning.constantValue = .005;
        tuning.relativeAreaScalar = .075;
        tuning.dotProductScalar = .3;
        tuning.tagCountExponent = 1.2;
      }
      case 2 -> {
        name = "BackLeftCamera";
        cameraPosition =
            new Transform3d(
                -.1,
                .1,
                .1,
                new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(150)));
        tuning = new VisionTuningAutoLogged();
        tuning.constantValue = .005;
        tuning.relativeAreaScalar = .075;
        tuning.dotProductScalar = .3;
        tuning.tagCountExponent = 1.2;
      }
      case 3 -> {
        name = "BackRightCamera";
        cameraPosition =
            new Transform3d(
                -.1,
                -.1,
                .1,
                new Rotation3d(0, Units.degreesToRadians(-45), Units.degreesToRadians(-150)));
        tuning = new VisionTuningAutoLogged();
        tuning.constantValue = .005;
        tuning.relativeAreaScalar = .075;
        tuning.dotProductScalar = .3;
        tuning.tagCountExponent = 1.2;
      }
      default -> throw new IndexOutOfBoundsException();
    }

    camera = new PhotonCamera(name);
    poseEstimator =
        new PhotonPoseEstimator(
            tags, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPosition);
    cameraPos = cameraPosition;

    if (Robot.isSimulation()) {
      var cameraProps = new SimCameraProperties();
      cameraProps.setCalibError(.25, .1);
      cameraProps.setAvgLatencyMs(5);
      cameraProps.setLatencyStdDevMs(2);
      VisionSim.getInstance().addSimCamera(new PhotonCameraSim(camera, cameraProps), cameraPos);
    }
  }

  @Override
  public void updateInputs(VisionPoseEstimatorIOInputs inputs) {
    if (Robot.isSimulation()) {
      VisionSim.getInstance().update();
    }

    var estimateList = new ArrayList<PoseEstimate>();

    var results = camera.getAllUnreadResults();
    for (var result : results) {
      poseEstimator
          .update(result)
          .ifPresent(
              (estimate) -> {
                var tagRelativePositions = new Transform3d[estimate.targetsUsed.size()];
                var tagAbsolutePoses = new Pose3d[estimate.targetsUsed.size()];
                for (int i = 0; i < tagAbsolutePoses.length; i++) {
                  tagRelativePositions[i] = estimate.targetsUsed.get(i).bestCameraToTarget;
                  tagAbsolutePoses[i] =
                      estimate.estimatedPose.plus(cameraPos.plus(tagRelativePositions[i]));
                }
                estimateList.add(
                    new PoseEstimate(
                        estimate.estimatedPose,
                        estimate.timestampSeconds,
                        tagAbsolutePoses,
                        tagRelativePositions));
              });
    }

    inputs.poseEstimates = estimateList.toArray(new PoseEstimate[0]);
  }

  @Override
  public VisionTuningAutoLogged getStdDevTuning() {
    return tuning;
  }

  private static class VisionSim {
    private static VisionSim visionSim;

    public static VisionSim getInstance() {
      if (visionSim == null) {
        visionSim = new VisionSim();
      }
      return visionSim;
    }

    final VisionSystemSim systemSim;

    public VisionSim() {
      systemSim = new VisionSystemSim("main");
      systemSim.addAprilTags(tags);
    }

    public void addSimCamera(PhotonCameraSim cameraSim, Transform3d cameraPosition) {
      cameraSim.enableProcessedStream(false);
      cameraSim.enableRawStream(false);
      systemSim.addCamera(cameraSim, cameraPosition);
    }

    public void update() {
      systemSim.update(Robot.groundTruthPoseSupplier.get());
    }
  }
}
