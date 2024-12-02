// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public final class MiscStructs {
  private MiscStructs() {}

  public static final Struct<PIDController> pidControllerStruct =
    new Struct<>() {
      @Override
      public Class<PIDController> getTypeClass() {
        return PIDController.class;
      }

      @Override
      public String getTypeName() {
        return "PIDController";
      }

      @Override
      public int getSize() {
        return kSizeDouble * 3;
      }

      @Override
      public String getSchema() {
        return "double kP;double kI;double kD";
      }

      @Override
      public PIDController unpack(ByteBuffer bb) {
        double p = bb.getDouble();
        double i = bb.getDouble();
        double d = bb.getDouble();

        return new PIDController(p, i, d);
      }

      @Override
      public void pack(ByteBuffer bb, PIDController value) {
        bb.putDouble(value.getP());
        bb.putDouble(value.getI());
        bb.putDouble(value.getD());
      }
    };
}
