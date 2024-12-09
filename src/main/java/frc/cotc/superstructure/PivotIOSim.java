// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.cotc.Robot;

public class PivotIOSim implements PivotIO {
  private final PivotIOConstantsAutoLogged CONSTANTS = new PivotIOConstantsAutoLogged();

  private final SingleJointedArmSim leftArmSim;
  private final SingleJointedArmSim rightArmSim;

  public PivotIOSim() {
    CONSTANTS.maxAngleRad = Units.degreesToRadians(90);
    CONSTANTS.minAngleRad = 0;
    var motor = DCMotor.getNeoVortex(1);
    double gearRatio = 400;
    CONSTANTS.maxSpeedRadPerSec = motor.freeSpeedRadPerSec / gearRatio;

    CONSTANTS.kS = 0.05;
    CONSTANTS.kG = .16;
    CONSTANTS.kV = 12 / CONSTANTS.maxSpeedRadPerSec;
    CONSTANTS.kA = .02;
    CONSTANTS.angleKp = 5;
    CONSTANTS.diffKp = 1;
    CONSTANTS.velKp = 5;

    double massKg = Units.lbsToKilograms(22);
    double comDistanceMeters = Units.inchesToMeters(15);
    double moiKgMetersSquared = SingleJointedArmSim.estimateMOI(comDistanceMeters, massKg);
    double startAngle =
        MathUtil.interpolate(CONSTANTS.minAngleRad, CONSTANTS.maxAngleRad, Math.random());
    leftArmSim =
        new SingleJointedArmSim(
            motor,
            gearRatio,
            moiKgMetersSquared,
            comDistanceMeters,
            CONSTANTS.minAngleRad,
            CONSTANTS.maxAngleRad,
            true,
            startAngle);
    rightArmSim =
        new SingleJointedArmSim(
            motor,
            gearRatio,
            moiKgMetersSquared,
            comDistanceMeters,
            CONSTANTS.minAngleRad,
            CONSTANTS.maxAngleRad,
            true,
            startAngle);
  }

  @Override
  public PivotIOConstantsAutoLogged getConstants() {
    return CONSTANTS;
  }

  private double leftSimVolts;
  private double rightSimVolts;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    var batteryVoltage = RobotController.getBatteryVoltage();
    var leftVolts = MathUtil.clamp(leftSimVolts, -batteryVoltage, batteryVoltage);
    if (MathUtil.isNear(0, leftVolts, CONSTANTS.kS)) {
      leftVolts = 0;
    } else {
      leftVolts -= CONSTANTS.kS * Math.signum(leftVolts);
    }
    leftArmSim.setInputVoltage(leftVolts);
    leftArmSim.update(Robot.defaultPeriodSecs);
    var rightVolts = MathUtil.clamp(rightSimVolts, -batteryVoltage, batteryVoltage);
    if (MathUtil.isNear(0, rightVolts, CONSTANTS.kS)) {
      rightVolts = 0;
    } else {
      rightVolts -= CONSTANTS.kS * Math.signum(rightVolts);
    }
    rightArmSim.setInputVoltage(rightVolts);
    rightArmSim.update(Robot.defaultPeriodSecs);

    inputs.leftAngleRad = leftArmSim.getAngleRads();
    inputs.rightAngleRad = rightArmSim.getAngleRads();
    inputs.leftVelRadPerSec = leftArmSim.getVelocityRadPerSec();
    inputs.rightVelRadPerSec = rightArmSim.getVelocityRadPerSec();
    inputs.leftCurrentDraws.statorCurrent = leftArmSim.getCurrentDrawAmps();
    inputs.leftCurrentDraws.supplyCurrent =
        inputs.leftCurrentDraws.statorCurrent * Math.abs(leftArmSim.getInput(0) / batteryVoltage);
    inputs.rightCurrentDraws.statorCurrent = rightArmSim.getCurrentDrawAmps();
    inputs.rightCurrentDraws.supplyCurrent =
        inputs.rightCurrentDraws.statorCurrent * Math.abs(rightArmSim.getInput(0) / batteryVoltage);
  }

  @Override
  public void run(double leftVolts, double rightVolts) {
    this.leftSimVolts = leftVolts;
    this.rightSimVolts = rightVolts;
  }
}
