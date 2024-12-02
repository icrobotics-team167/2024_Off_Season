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

public interface FlywheelIO {
  class FlywheelIOInputs implements LoggableInputs {
    double topPosMeters;
    double topVelMetersPerSec;
    double bottomPosMeters;
    double bottomVelMetersPerSec;

    MotorCurrentDraws topCurrentDraws = new MotorCurrentDraws();
    MotorCurrentDraws bottomCurrentDraws = new MotorCurrentDraws();

    @Override
    public void toLog(LogTable table) {
      table.put("topPosMeters", topPosMeters);
      table.put("topVelMetersPerSec", topVelMetersPerSec);
      table.put("bottomPosMeters", bottomPosMeters);
      table.put("bottomVelMetersPerSec", bottomVelMetersPerSec);
      table.put("topCurrentDraws", MotorCurrentDraws.struct, topCurrentDraws);
      table.put("bottomCurrentDraws", MotorCurrentDraws.struct, bottomCurrentDraws);
    }

    @Override
    public void fromLog(LogTable table) {
      topPosMeters = table.get("topPosMeters", 0.0);
      topVelMetersPerSec = table.get("topVelMetersPerSec", 0.0);
      bottomPosMeters = table.get("bottomPosMeters", 0.0);
      bottomVelMetersPerSec = table.get("bottomVelMetersPerSec", 0.0);
      topCurrentDraws =
          table.get("topCurrentDraws", MotorCurrentDraws.struct, new MotorCurrentDraws());
      bottomCurrentDraws =
          table.get("bottomCurrentDraws", MotorCurrentDraws.struct, new MotorCurrentDraws());
    }
  }

  @AutoLog
  class FlywheelIOConstants {
    double topTolerance;
    double topKs;
    double topKv;
    double bottomTolerance;
    double bottomKs;
    double bottomKv;
  }

  default FlywheelIOConstantsAutoLogged getControllers() {
    return new FlywheelIOConstantsAutoLogged();
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void run(double topVolts, double bottomVolts) {}
}
