// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.cotc.util.MiscStructs;
import frc.cotc.util.MotorCurrentDraws;
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

  class FlywheelIOControllers implements LoggableInputs {
    SimpleMotorFeedforward topFF;
    SimpleMotorFeedforward bottomFF;

    PIDController topController;
    PIDController bottomController;

    @Override
    public void toLog(LogTable table) {
      table.put("topFF", SimpleMotorFeedforward.struct, topFF);
      table.put("bottomFF", SimpleMotorFeedforward.struct, bottomFF);
      table.put("topController", MiscStructs.pidControllerStruct, topController);
      table.put("bottomController", MiscStructs.pidControllerStruct, bottomController);
    }

    @Override
    public void fromLog(LogTable table) {
      topFF =
          table.get("topFF", SimpleMotorFeedforward.struct, new SimpleMotorFeedforward(0, 1, .1));
      bottomFF =
          table.get(
              "bottomFF", SimpleMotorFeedforward.struct, new SimpleMotorFeedforward(0, 1, .1));
      topController =
          table.get("topController", MiscStructs.pidControllerStruct, new PIDController(1, 0, 0));
      bottomController =
          table.get(
              "bottomController", MiscStructs.pidControllerStruct, new PIDController(1, 0, 0));
    }
  }

  default FlywheelIOControllers getControllers() {
    return new FlywheelIOControllers();
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void run(double topVolts, double bottomVolts) {}
}
