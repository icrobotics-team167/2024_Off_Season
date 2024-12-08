// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import static frc.cotc.util.SparkUtils.configureSparks;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.cotc.Robot;

public class PivotIOSparkFlex implements PivotIO {
  private final PivotIOConstantsAutoLogged CONSTANTS = new PivotIOConstantsAutoLogged();

  private final SparkFlex leftMotor;
  private final RelativeEncoder leftEncoder;
  private final SparkFlex rightMotor;
  private final RelativeEncoder rightEncoder;

  private final DutyCycleEncoder angleEncoder;

  private SingleJointedArmSim leftArmSim;
  private SparkSim leftMotorSim;
  private SingleJointedArmSim rightArmSim;
  private SparkSim rightMotorSim;

  private final double startAngle;

  public PivotIOSparkFlex() {
    CONSTANTS.maxAngleRad = Units.degreesToRadians(90);
    CONSTANTS.minAngleRad = 0;
    var motor = DCMotor.getNeoVortex(1);
    double gearRatio = 400;
    CONSTANTS.maxSpeedRadPerSec = motor.freeSpeedRadPerSec / gearRatio;

    // TODO: Configure CAN IDs
    leftMotor = new SparkFlex(15, SparkLowLevel.MotorType.kBrushless);
    rightMotor = new SparkFlex(16, SparkLowLevel.MotorType.kBrushless);

    var config = new SparkFlexConfig();
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    config.smartCurrentLimit(40);
    config.voltageCompensation(12);
    config.signals.appliedOutputPeriodMs(10);
    config.encoder.positionConversionFactor(2 * Math.PI / gearRatio);
    config.encoder.velocityConversionFactor(2 * Math.PI / (gearRatio * 60));

    configureSparks(config, leftMotor, rightMotor);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    angleEncoder = new DutyCycleEncoder(0);

    if (Robot.isSimulation()) {
      double massKg = Units.lbsToKilograms(22);
      double comDistanceMeters = Units.inchesToMeters(15);
      double moiKgMetersSquared = massKg * comDistanceMeters * comDistanceMeters;
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

      leftMotorSim = new SparkSim(leftMotor, motor);
      rightMotorSim = new SparkSim(rightMotor, motor);
    }

    startAngle = getAbsoluteAngleRad();
  }

  @Override
  public PivotIOConstantsAutoLogged getConstants() {
    return CONSTANTS;
  }

  private double leftSimVolts;
  private double rightSimVolts;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (Robot.isSimulation()) {
      var batteryVoltage = RobotController.getBatteryVoltage();
      leftArmSim.setInputVoltage(MathUtil.clamp(leftSimVolts, -batteryVoltage, batteryVoltage));
      leftArmSim.update(Robot.defaultPeriodSecs);
      leftMotorSim.iterate(
          leftArmSim.getVelocityRadPerSec(), batteryVoltage, Robot.defaultPeriodSecs);
      rightArmSim.setInputVoltage(MathUtil.clamp(rightSimVolts, -batteryVoltage, batteryVoltage));
      rightArmSim.update(Robot.defaultPeriodSecs);
      rightMotorSim.iterate(
          rightArmSim.getVelocityRadPerSec(), batteryVoltage, Robot.defaultPeriodSecs);
    }

    inputs.leftAngleRad = startAngle + leftEncoder.getPosition();
    inputs.leftVelRadPerSec = leftEncoder.getVelocity();
    inputs.rightAngleRad = startAngle - rightEncoder.getPosition();
    inputs.rightVelRadPerSec = rightEncoder.getVelocity();

    inputs.leftCurrentDraws.statorCurrent = leftMotor.getOutputCurrent();
    inputs.leftCurrentDraws.supplyCurrent =
        inputs.leftCurrentDraws.statorCurrent * Math.abs(leftMotor.getAppliedOutput());
    inputs.rightCurrentDraws.statorCurrent = rightMotor.getOutputCurrent();
    inputs.rightCurrentDraws.supplyCurrent =
        inputs.rightCurrentDraws.statorCurrent * Math.abs(rightMotor.getAppliedOutput());
  }

  @Override
  public void run(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(-rightVolts);

    if (Robot.isSimulation()) {
      leftSimVolts = leftVolts;
      rightSimVolts = rightVolts;
    }
  }

  private double getAbsoluteAngleRad() {
    if (Robot.isReal()) {
      return Units.rotationsToRadians(angleEncoder.get() - (193.5 / 360));
    }
    return (leftArmSim.getAngleRads() + rightArmSim.getAngleRads()) / 2;
  }
}
