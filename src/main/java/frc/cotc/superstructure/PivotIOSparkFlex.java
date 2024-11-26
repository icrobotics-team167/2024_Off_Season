// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.cotc.Robot;

// See README.md
@SuppressWarnings("removal")
public class PivotIOSparkFlex implements PivotIO {
  static final double gearRatio = 400;
  static final DCMotor motor = DCMotor.getNeoVortex(1);
  static final PivotIOConstantsAutoLogged CONSTANTS = new PivotIOConstantsAutoLogged();

  static {
    CONSTANTS.maxAngleRad = Units.degreesToRadians(90);
    CONSTANTS.minAngleRad = 0;
    CONSTANTS.maxSpeedRadPerSec = motor.freeSpeedRadPerSec / gearRatio;
  }

  private final PIDController differentialPID;
  private final ArmFeedforward armFeedforward;
  private final ExponentialProfile profile;

  private final SparkFlex leftMotor;
  private final RelativeEncoder leftEncoder;
  private final SparkFlex rightMotor;
  private final RelativeEncoder rightEncoder;

  private final DutyCycleEncoder angleEncoder;

  private SingleJointedArmSim leftArmSim;
  private SparkSim leftMotorSim;
  private SingleJointedArmSim rightArmSim;
  private SparkSim rightMotorSim;

  public PivotIOSparkFlex() {
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

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftMotor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
    rightMotor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    angleEncoder = new DutyCycleEncoder(0);

    double kv = gearRatio / motor.KvRadPerSecPerVolt;
    double ka = .01;
    differentialPID = new PIDController(1, 0, 0);
    armFeedforward = new ArmFeedforward(0, 0.16, kv, ka);

    // Not quite 12v, need some control headroom
    profile =
        new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(11, kv, ka));

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

    leftEncoder.setPosition(getAbsoluteAngleRad());
    rightEncoder.setPosition(getAbsoluteAngleRad());
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.leftAngleRad = leftEncoder.getPosition();
    inputs.leftVelRadPerSec = leftEncoder.getVelocity();
    inputs.rightAngleRad = rightEncoder.getPosition();
    inputs.rightVelRadPerSec = rightEncoder.getVelocity();

    inputs.leftMotorCurrents.statorCurrent = leftMotor.getOutputCurrent();
    inputs.leftMotorCurrents.supplyCurrent =
        inputs.leftMotorCurrents.statorCurrent * Math.abs(leftMotor.getAppliedOutput());
    inputs.rightMotorCurrents.statorCurrent = rightMotor.getOutputCurrent();
    inputs.rightMotorCurrents.supplyCurrent =
        inputs.rightMotorCurrents.statorCurrent * Math.abs(rightMotor.getAppliedOutput());
  }

  private final PIDController leftPosController = new PIDController(5, 0, 0);
  private final PIDController rightPosController = new PIDController(5, 0, 0);

  @Override
  public void aimAtAngle(double angleRad, double velRadPerSec) {
    var leftPos = leftEncoder.getPosition();
    var rightPos = rightEncoder.getPosition();
    var leftVel = leftEncoder.getVelocity();
    var rightVel = rightEncoder.getVelocity();

    var state =
        profile.calculate(
            Robot.defaultPeriodSecs,
            new ExponentialProfile.State((leftPos + rightPos) / 2, (leftVel + rightVel) / 2),
            new ExponentialProfile.State(angleRad, velRadPerSec));

    var leftPID = leftPosController.calculate(leftPos, state.position);
    var rightPID = rightPosController.calculate(rightPos, state.position);
    var diffControl = differentialPID.calculate(leftPos - rightPos, 0);

    runLeft(leftPos, leftVel, state.velocity + leftPID + diffControl);
    runRight(rightPos, rightVel, state.velocity + rightPID - diffControl);
  }

  @Override
  public void runVel(double velRadPerSec) {
    var leftPos = leftEncoder.getPosition();
    var rightPos = rightEncoder.getPosition();

    var diffControl =
        differentialPID.calculate(leftEncoder.getPosition() - rightEncoder.getPosition(), 0);
    runLeft(leftPos, leftEncoder.getVelocity(), velRadPerSec + diffControl);
    runRight(rightPos, rightEncoder.getVelocity(), velRadPerSec - diffControl);
  }

  private final PIDController leftVelController = new PIDController(5, 0, 0);

  private void runLeft(double leftPos, double currentVelRadPerSec, double velRadPerSec) {
    double batteryVoltage = RobotController.getBatteryVoltage();
    double voltage =
        MathUtil.clamp(
            armFeedforward.calculate(
                    leftPos, currentVelRadPerSec, velRadPerSec, Robot.defaultPeriodSecs)
                + leftVelController.calculate(currentVelRadPerSec, velRadPerSec),
            -batteryVoltage,
            batteryVoltage);
    leftMotor.setVoltage(voltage);
    if (Robot.isSimulation()) {
      leftArmSim.setInputVoltage(voltage);
      leftArmSim.update(Robot.defaultPeriodSecs);
      leftMotorSim.iterate(
          leftArmSim.getVelocityRadPerSec(), batteryVoltage, Robot.defaultPeriodSecs);
    }
  }

  private final PIDController rightVelController = new PIDController(5, 0, 0);

  private void runRight(double rightPos, double currentVelRadPerSec, double velRadPerSec) {
    double batteryVoltage = RobotController.getBatteryVoltage();
    double voltage =
        MathUtil.clamp(
            armFeedforward.calculate(
                    rightPos, currentVelRadPerSec, velRadPerSec, Robot.defaultPeriodSecs)
                + rightVelController.calculate(currentVelRadPerSec, velRadPerSec),
            -batteryVoltage,
            batteryVoltage);
    rightMotor.setVoltage(voltage);
    if (Robot.isSimulation()) {
      rightArmSim.setInputVoltage(voltage);
      rightArmSim.update(Robot.defaultPeriodSecs);
      rightMotorSim.iterate(
          rightArmSim.getVelocityRadPerSec(), batteryVoltage, Robot.defaultPeriodSecs);
    }
  }

  private double getAbsoluteAngleRad() {
    if (Robot.isReal()) {
      return Units.rotationsToRadians(angleEncoder.get() - (193.5 / 360));
    }
    return (leftArmSim.getAngleRads() + rightArmSim.getAngleRads()) / 2;
  }
}
