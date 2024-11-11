// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.cotc.vision.VisionPoseEstimatorIO.VisionPoseEstimatorInputs;
import java.nio.ByteBuffer;
import org.littletonrobotics.junction.Logger;

public class VisionPoseEstimator {
  private final VisionPoseEstimatorIO io;
  private final VisionPoseEstimatorInputs inputs = new VisionPoseEstimatorInputs();

  public VisionPoseEstimator(VisionPoseEstimatorIO io) {
    this.io = io;
  }

  public record PoseEstimate(
      Pose2d pose, double xStdDevs, double yStdDevs, double yawStdDevs, double timestamp)
      implements StructSerializable {
    public static final Struct<PoseEstimate> struct =
        new Struct<>() {
          @Override
          public Class<PoseEstimate> getTypeClass() {
            return PoseEstimate.class;
          }

          @Override
          public String getTypeName() {
            return "PoseEstimate";
          }

          @Override
          public int getSize() {
            return Pose2d.struct.getSize() + kSizeDouble * 4;
          }

          @Override
          public String getSchema() {
            return "Pose2d pose;double xStdDevs;double yStdDevs;double yawStdDevs;double timestamp";
          }

          @Override
          public PoseEstimate unpack(ByteBuffer bb) {
            var pose = Pose2d.struct.unpack(bb);
            var xStdDevs = bb.getDouble();
            var yStdDevs = bb.getDouble();
            var yawStdDevs = bb.getDouble();
            var timestamp = bb.getDouble();
            return new PoseEstimate(pose, xStdDevs, yStdDevs, yawStdDevs, timestamp);
          }

          @Override
          public void pack(ByteBuffer bb, PoseEstimate value) {
            Pose2d.struct.pack(bb, value.pose);
            bb.putDouble(value.xStdDevs);
            bb.putDouble(value.yStdDevs);
            bb.putDouble(value.yawStdDevs);
            bb.putDouble(value.timestamp);
          }

          @Override
          public Struct<?>[] getNested() {
            return new Struct<?>[] {Pose2d.struct};
          }
        };
  }

  public PoseEstimate[] poll(ChassisSpeeds currentSpeeds) {
    io.updateInputs(inputs, currentSpeeds);
    Logger.processInputs("Vision", inputs);

    return inputs.poseEstimates;
  }
}
