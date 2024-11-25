// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.cotc.util.MotorCurrentDraws;

abstract class FlywheelIOSparkFlexAbstract implements FlywheelIO {
  static final DCMotor motor = DCMotor.getNeoVortex(1);
  static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
  static final FlywheelIOConstantsAutoLogged CONSTANTS = new FlywheelIOConstantsAutoLogged();

  static {
    CONSTANTS.maxVelMetersPerSec = motor.freeSpeedRadPerSec * (WHEEL_DIAMETER_METERS / 2);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.topPosMeters = getTopPos();
    inputs.topVelMetersPerSec = getTopVel();
    inputs.bottomPosMeters = getBottomPos();
    inputs.bottomVelMetersPerSec = getBottomVel();
    updateCurrentDraws(inputs.topCurrentDraws, inputs.bottomCurrentDraws);
  }

  @Override
  public final FlywheelIOConstantsAutoLogged getConstants() {
    return CONSTANTS;
  }

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(0, 12 / CONSTANTS.maxVelMetersPerSec);
  private final PIDController topController = new PIDController(5, 0, 0);
  private final PIDController bottomController = new PIDController(5, 0, 0);

  @Override
  public final void runVel(double topVelMetersPerSec, double bottomVelMetersPerSec) {
    //noinspection removal
    runVoltage(
        feedforward.calculate(topVelMetersPerSec)
            + topController.calculate(getTopVel(), topVelMetersPerSec),
        feedforward.calculate(bottomVelMetersPerSec)
            + bottomController.calculate(getBottomVel(), bottomVelMetersPerSec));
  }

  abstract double getTopPos();

  abstract double getTopVel();

  abstract double getBottomPos();

  abstract double getBottomVel();

  abstract void runVoltage(double topVolts, double bottomVolts);

  abstract void updateCurrentDraws(MotorCurrentDraws top, MotorCurrentDraws bottom);
}
