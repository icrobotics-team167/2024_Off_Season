// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import frc.cotc.utils.SparkUtils;
import java.util.Set;

public class ArmIOVortex implements ArmIO {
  private final CANSparkFlex leftMotor;
  private final CANSparkFlex rightMotor;

  public ArmIOVortex() {
    leftMotor = new CANSparkFlex(2, CANSparkLowLevel.MotorType.kBrushless);
    rightMotor = new CANSparkFlex(3, CANSparkLowLevel.MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
    leftMotor.enableVoltageCompensation(12);
    rightMotor.enableVoltageCompensation(12);
    leftMotor.setSmartCurrentLimit(60);
    rightMotor.setSmartCurrentLimit(60);
    SparkUtils.configureFrameStrategy(
        leftMotor, Set.of(SparkUtils.Data.POSITION, SparkUtils.Data.VELOCITY), Set.of(), false);
    SparkUtils.configureFrameStrategy(
        rightMotor, Set.of(SparkUtils.Data.POSITION, SparkUtils.Data.VELOCITY), Set.of(), false);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void goToAngle(double angleRad) {
    ArmIO.super.goToAngle(angleRad);
  }

  @Override
  public void setVelocity(double speed) {
    ArmIO.super.setVelocity(speed);
  }
}
