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
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.cotc.util.MotorCurrentDraws;

public class PivotIOSparkFlexReal extends PivotIOSparkFlexAbstract {
  private final SparkFlex leftMotor;
  private final RelativeEncoder leftEncoder;
  private final SparkFlex rightMotor;
  private final RelativeEncoder rightEncoder;

  private final DutyCycleEncoder angleEncoder;

  public PivotIOSparkFlexReal() {
    // TODO: Configure CAN IDs
    leftMotor = new SparkFlex(0, SparkLowLevel.MotorType.kBrushless);
    rightMotor = new SparkFlex(0, SparkLowLevel.MotorType.kBrushless);

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
        new SparkFlexConfig(),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
    rightMotor.configure(
        new SparkFlexConfig(),
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    angleEncoder = new DutyCycleEncoder(0);
    leftEncoder.setPosition(getAbsoluteAngleRad());
    rightEncoder.setPosition(getAbsoluteAngleRad());
  }

  @Override
  double getLeftAngleRad() {
    return leftEncoder.getPosition();
  }

  @Override
  double getLeftVelRadPerSec() {
    return leftEncoder.getVelocity();
  }

  @Override
  double getRightAngleRad() {
    return rightEncoder.getPosition();
  }

  @Override
  double getRightVelRadPerSec() {
    return rightEncoder.getVelocity();
  }

  @Override
  void runVoltage(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
  }

  @Override
  void updateCurrents(MotorCurrentDraws left, MotorCurrentDraws right) {
    left.statorCurrent = leftMotor.getOutputCurrent();
    left.supplyCurrent = left.statorCurrent * leftMotor.getAppliedOutput();
    right.statorCurrent = rightMotor.getOutputCurrent();
    right.supplyCurrent = right.statorCurrent * rightMotor.getAppliedOutput();
  }

  private double getAbsoluteAngleRad() {
    return Units.rotationsToRadians(angleEncoder.get() - (193.5 / 360));
  }
}
