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
import frc.cotc.util.MotorCurrentDraws;

public class PivotIOSparkFlexSim extends PivotIOSparkFlexAbstract {
  private final SingleJointedArmSim leftArmSim;
  private final SingleJointedArmSim rightArmSim;

  public PivotIOSparkFlexSim() {
    var motor = DCMotor.getNeoVortex(1);
    var CoMDistanceMeters = Units.inchesToMeters(15);
    var massKg = Units.lbsToKilograms(22);
    var moiKgMetersSquared = massKg * CoMDistanceMeters * CoMDistanceMeters;
    var startingAngleRad =
        MathUtil.interpolate(CONSTANTS.minAngleRad, CONSTANTS.maxAngleRad, Math.random());
    var angleDiff = Units.degreesToRadians((Math.random() * 2 - 1) * 10);
    leftArmSim =
        new SingleJointedArmSim(
            motor,
            gearRatio,
            moiKgMetersSquared,
            CoMDistanceMeters,
            CONSTANTS.minAngleRad,
            CONSTANTS.maxAngleRad,
            true,
            startingAngleRad + angleDiff);
    rightArmSim =
        new SingleJointedArmSim(
            motor,
            gearRatio,
            moiKgMetersSquared,
            CoMDistanceMeters,
            CONSTANTS.minAngleRad,
            CONSTANTS.maxAngleRad,
            true,
            startingAngleRad - angleDiff);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    leftArmSim.update(Robot.defaultPeriodSecs);
    rightArmSim.update(Robot.defaultPeriodSecs);
    super.updateInputs(inputs);
  }

  @Override
  double getLeftAngleRad() {
    return leftArmSim.getAngleRads();
  }

  @Override
  double getLeftVelRadPerSec() {
    return leftArmSim.getVelocityRadPerSec();
  }

  @Override
  double getRightAngleRad() {
    return rightArmSim.getAngleRads();
  }

  @Override
  double getRightVelRadPerSec() {
    return rightArmSim.getVelocityRadPerSec();
  }

  @Override
  void runVoltage(double leftVolts, double rightVolts) {
    var supplyVoltage = RobotController.getBatteryVoltage();
    leftArmSim.setInputVoltage(MathUtil.clamp(leftVolts, -supplyVoltage, supplyVoltage));
    rightArmSim.setInputVoltage(MathUtil.clamp(rightVolts, -supplyVoltage, supplyVoltage));
  }

  @Override
  void updateCurrents(MotorCurrentDraws left, MotorCurrentDraws right) {
    left.statorCurrent = leftArmSim.getCurrentDrawAmps();
    left.supplyCurrent = left.statorCurrent * (leftArmSim.getInput(0) / 12);
    right.statorCurrent = rightArmSim.getCurrentDrawAmps();
    right.supplyCurrent = right.statorCurrent * (rightArmSim.getInput(0) / 12);
  }
}
