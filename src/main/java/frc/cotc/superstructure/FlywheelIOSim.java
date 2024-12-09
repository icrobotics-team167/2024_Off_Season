// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.cotc.Robot;

public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelIOConstantsAutoLogged CONSTANTS = new FlywheelIOConstantsAutoLogged();
  private final double wheelDiameterMeters;

  private final DCMotorSim topSim;
  private final DCMotorSim bottomSim;
  private final DCMotorSim guideSim;

  public FlywheelIOSim() {
    var motor = DCMotor.getNeoVortex(1);

    wheelDiameterMeters = Units.inchesToMeters(4);

    double kv = 12 / ((wheelDiameterMeters / 2) * motor.freeSpeedRadPerSec);
    CONSTANTS.topTolerance = 1;
    CONSTANTS.bottomTolerance = 1;
    CONSTANTS.guideTolerance = 1;
    CONSTANTS.topKs = 0.1;
    CONSTANTS.bottomKs = 0.1;
    CONSTANTS.guideKs = 0.1;
    CONSTANTS.topKv = kv;
    CONSTANTS.bottomKv = kv;
    CONSTANTS.guideKv = kv;

    topSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(12 / motor.freeSpeedRadPerSec, .01), motor);
    bottomSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(12 / motor.freeSpeedRadPerSec, .01), motor);
    guideSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(12 / motor.freeSpeedRadPerSec, .01), motor);
  }

  @Override
  public FlywheelIOConstantsAutoLogged getConstants() {
    return CONSTANTS;
  }

  private double topCommandedVolts;
  private double bottomCommandedVolts;
  private double guideCommandedVolts;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    double busVoltage = RobotController.getBatteryVoltage();

    topSim.setInputVoltage(
        MathUtil.clamp(topCommandedVolts, -busVoltage, busVoltage)
            - CONSTANTS.topKs * Math.signum(topCommandedVolts));
    topSim.update(Robot.defaultPeriodSecs);

    bottomSim.setInputVoltage(
        MathUtil.clamp(bottomCommandedVolts, -busVoltage, busVoltage)
            - CONSTANTS.bottomKs * Math.signum(bottomCommandedVolts));
    bottomSim.update(Robot.defaultPeriodSecs);

    guideSim.setInputVoltage(
        MathUtil.clamp(guideCommandedVolts, -busVoltage, busVoltage)
            - CONSTANTS.guideKs * Math.signum(guideCommandedVolts));
    guideSim.update(Robot.defaultPeriodSecs);

    inputs.topPosMeters = topSim.getAngularPositionRad() * (wheelDiameterMeters / 2);
    inputs.topVelMetersPerSec = topSim.getAngularVelocityRadPerSec() * (wheelDiameterMeters / 2);
    inputs.bottomPosMeters = bottomSim.getAngularPositionRad() * (wheelDiameterMeters / 2);
    inputs.bottomVelMetersPerSec =
        bottomSim.getAngularVelocityRadPerSec() * (wheelDiameterMeters / 2);
    inputs.guidePosMeters = guideSim.getAngularPositionRad() * (wheelDiameterMeters / 2);
    inputs.guideVelMetersPerSec =
        guideSim.getAngularVelocityRadPerSec() * (wheelDiameterMeters / 2);
  }

  @Override
  public void run(double topVolts, double bottomVolts, double guideVolts) {
    topCommandedVolts = topVolts;
    bottomCommandedVolts = bottomVolts;
    guideCommandedVolts = guideVolts;
  }
}
