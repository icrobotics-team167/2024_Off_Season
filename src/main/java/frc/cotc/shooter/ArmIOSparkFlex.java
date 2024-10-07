// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import static frc.cotc.util.SparkUtils.configureSpark;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;

public class ArmIOSparkFlex implements ArmIO {
  private final CANSparkFlex left;
  private final CANSparkFlex right;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  public ArmIOSparkFlex() {
    left = new CANSparkFlex(19, CANSparkLowLevel.MotorType.kBrushless);
    right = new CANSparkFlex(20, CANSparkLowLevel.MotorType.kBrushless);

    configureSpark(left::restoreFactoryDefaults);
    configureSpark(right::restoreFactoryDefaults);
    Timer.delay(.1);

    left.setInverted(false);
    right.setInverted(true);

    configureSpark(() -> left.setSmartCurrentLimit(60));
    configureSpark(() -> right.setSmartCurrentLimit(60));
    Timer.delay(.1);

    leftEncoder = left.getEncoder();
    rightEncoder = right.getEncoder();

    // 400:1 ratio, rotations to radians -> 2pi / 400 -> pi / 200
    configureSpark(() -> leftEncoder.setPositionConversionFactor(Math.PI / 200));
    configureSpark(() -> rightEncoder.setPositionConversionFactor(Math.PI / 200));
    Timer.delay(.1);

    // radians per minute -> radians per second = / 60, pi / 200 / 60 = pi / 1200
    configureSpark(() -> leftEncoder.setVelocityConversionFactor(Math.PI / 1200));
    configureSpark(() -> rightEncoder.setVelocityConversionFactor(Math.PI / 1200));
    Timer.delay(.1);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    ArmIO.super.updateInputs(inputs);
  }

  @Override
  public void pivot(double speed) {
    ArmIO.super.pivot(speed);
  }

  @Override
  public void angleTo(double pos) {
    ArmIO.super.angleTo(pos);
  }
}
