// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionPoseEstimatorIO {
  @AutoLog
  class VisionTuning {
    public double relativeAreaScalar = .075;
    public double dotProductScalar = .3;
    public double constantValue = .005;
    public double tagCountExponent = 1.2;
  }

  class VisionPoseEstimatorIOInputs implements LoggableInputs {
    public PoseEstimate[] poseEstimates = new PoseEstimate[0];

    @Override
    public void toLog(LogTable table) {
      for (int i = 0; i < poseEstimates.length; i++) {
        poseEstimates[i].toLog(table, i);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      poseEstimates = PoseEstimate.fromLog(table);
    }
  }

  record PoseEstimate(
      Pose3d estimatedPose,
      double timestamp,
      Pose3d[] tagsUsed,
      Transform3d[] tagRelativePositions) {
    public void toLog(LogTable table, int i) {
      table.put("PoseEstimates/" + i + "/estimatedPose", estimatedPose);
      table.put("PoseEstimates/" + i + "/timestamp", timestamp);
      table.put("PoseEstimates/" + i + "/tagsUsed", tagsUsed);
      table.put("PoseEstimates/" + i + "/tagRelativePositions", tagRelativePositions);
    }

    public static PoseEstimate[] fromLog(LogTable table) {
      var entries = new ArrayList<PoseEstimate>();

      int i = 0;
      while (true) {
        var timestamp = table.get("PoseEstimates/" + i + "/timestamp", -1.0);
        if (timestamp < 0) {
          break;
        }

        var estimatedPose = table.get("PoseEstimates/" + i + "/estimatedPose", Pose3d.kZero);
        var tagsUsed = table.get("PoseEstimates/" + i + "/tagsUsed", new Pose3d[0]);
        var tagRelativePositions =
            table.get("PoseEstimates/" + i + "/tagRelativePositions", new Transform3d[0]);

        entries.add(new PoseEstimate(estimatedPose, timestamp, tagsUsed, tagRelativePositions));
      }

      return entries.toArray(new PoseEstimate[i + 1]);
    }
  }

  default void updateInputs(VisionPoseEstimatorIOInputs inputs) {}

  default VisionTuningAutoLogged getStdDevTuning() {
    return new VisionTuningAutoLogged();
  }
}
