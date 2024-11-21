// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface PivotIO {
  class PivotIOInputs implements LoggableInputs {
    double angleRad;
    double velRadPerSec;

    double leftAngleRad;
    double rightAngleRad;

    MotorCurrentDraws leftMotorCurrents = new MotorCurrentDraws(0, 0);
    MotorCurrentDraws rightMotorCurrents = new MotorCurrentDraws(0, 0);

    @Override
    public void toLog(LogTable table) {
      table.put("angleRad", angleRad);
      table.put("velRadPerSec", velRadPerSec);
      table.put("leftAngleRad", leftAngleRad);
      table.put("rightAngleRad", rightAngleRad);
      table.put("leftMotorCurrents", MotorCurrentDraws.struct, leftMotorCurrents);
      table.put("rightMotorCurrents", MotorCurrentDraws.struct, rightMotorCurrents);
    }

    @Override
    public void fromLog(LogTable table) {
      angleRad = table.get("angleRad", 0.0);
      velRadPerSec = table.get("velRadPerSec", 0.0);
      leftAngleRad = table.get("leftAngleRad", 0.0);
      rightAngleRad = table.get("rightAngleRad", 0.0);
      leftMotorCurrents =
          table.get("leftMotorCurrents", MotorCurrentDraws.struct, new MotorCurrentDraws(0, 0));
      rightMotorCurrents =
          table.get("rightMotorCurrents", MotorCurrentDraws.struct, new MotorCurrentDraws(0, 0));
    }
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void aimAtAngle(double angleRad) {
    aimAtAngle(angleRad, 0);
  }

  default void aimAtAngle(double angleRad, double velRadPerSec) {}

  default void runVel(double velRadPerSec) {}
}
