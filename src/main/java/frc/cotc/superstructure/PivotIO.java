// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface PivotIO {
  class PivotIOInputs implements LoggableInputs {
    double leftAngleRad;
    double leftVelRadPerSec;
    double rightAngleRad;
    double rightVelRadPerSec;

    MotorCurrentDraws leftCurrentDraws = new MotorCurrentDraws();
    MotorCurrentDraws rightCurrentDraws = new MotorCurrentDraws();

    @Override
    public void toLog(LogTable table) {
      table.put("leftAngleRad", leftAngleRad);
      table.put("leftVelRadPerSec", leftVelRadPerSec);
      table.put("rightAngleRad", rightAngleRad);
      table.put("rightVelRadPerSec", rightVelRadPerSec);
      table.put("leftMotorCurrents", MotorCurrentDraws.struct, leftCurrentDraws);
      table.put("rightMotorCurrents", MotorCurrentDraws.struct, rightCurrentDraws);
    }

    @Override
    public void fromLog(LogTable table) {
      leftAngleRad = table.get("leftAngleRad", 0.0);
      leftVelRadPerSec = table.get("leftVelRadPerSec", 0.0);
      rightAngleRad = table.get("rightAngleRad", 0.0);
      rightVelRadPerSec = table.get("rightVelRadPerSec", 0.0);
      leftCurrentDraws =
          table.get("leftMotorCurrents", MotorCurrentDraws.struct, new MotorCurrentDraws());
      rightCurrentDraws =
          table.get("rightMotorCurrents", MotorCurrentDraws.struct, new MotorCurrentDraws());
    }
  }

  @AutoLog
  class PivotIOConstants {
    double maxSpeedRadPerSec;
    double maxAngleRad;
    double minAngleRad;
  }

  default PivotIOConstantsAutoLogged getConstants() {
    return new PivotIOConstantsAutoLogged();
  }

  default void updateInputs(PivotIOInputs inputs) {}

  default void aimAtAngle(double angleRad, double velRadPerSec) {}

  default void runVel(double velRadPerSec) {}
}
