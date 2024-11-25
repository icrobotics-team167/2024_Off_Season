// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.cotc.Robot;
import frc.cotc.util.MotorCurrentDraws;

public class FlywheelIOSparkFlexSim extends FlywheelIOSparkFlexAbstract {
  private final DCMotorSim topSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 1, 1), motor);
  private final DCMotorSim bottomSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 1, 1), motor);

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    topSim.update(Robot.defaultPeriodSecs);
    bottomSim.update(Robot.defaultPeriodSecs);
    super.updateInputs(inputs);
  }

  @Override
  double getTopPos() {
    return topSim.getAngularPositionRad() * WHEEL_DIAMETER_METERS / 2;
  }

  @Override
  double getTopVel() {
    return topSim.getAngularVelocityRadPerSec() * WHEEL_DIAMETER_METERS / 2;
  }

  @Override
  double getBottomPos() {
    return bottomSim.getAngularPositionRad() * WHEEL_DIAMETER_METERS / 2;
  }

  @Override
  double getBottomVel() {
    return bottomSim.getAngularVelocityRadPerSec() * WHEEL_DIAMETER_METERS / 2;
  }

  @Override
  void runVoltage(double topVolts, double bottomVolts) {
    var supplyVoltage = RobotController.getBatteryVoltage();
    topSim.setInputVoltage(MathUtil.clamp(topVolts, -supplyVoltage, supplyVoltage));
    bottomSim.setInputVoltage(MathUtil.clamp(bottomVolts, -supplyVoltage, supplyVoltage));
  }

  @Override
  void updateCurrentDraws(MotorCurrentDraws top, MotorCurrentDraws bottom) {
    top.statorCurrent = topSim.getCurrentDrawAmps();
    top.supplyCurrent = top.supplyCurrent * (topSim.getInputVoltage() / 12);
    bottom.statorCurrent = bottomSim.getCurrentDrawAmps();
    bottom.supplyCurrent = bottom.supplyCurrent * (bottomSim.getInputVoltage() / 12);
  }
}
