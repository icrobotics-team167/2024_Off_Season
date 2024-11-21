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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class PivotIOSparkFlex implements PivotIO {
  private final SparkFlex leftMotor;
  private final RelativeEncoder leftEncoder;
  private final SparkFlex rightMotor;
  private final RelativeEncoder rightEncoder;

  private final DutyCycleEncoder angleEncoder;

  private final PIDController differentialPID;
  private final ArmFeedforward armFeedforward;
  private final PIDController anglePID;
  private final ExponentialProfile profile;

  public PivotIOSparkFlex() {
    // TODO: Configure CAN IDs
    leftMotor = new SparkFlex(0, SparkLowLevel.MotorType.kBrushless);
    rightMotor = new SparkFlex(0, SparkLowLevel.MotorType.kBrushless);

    double gearRatio = 400;
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

    differentialPID = new PIDController(1, 0, 0);
    armFeedforward = new ArmFeedforward(0, 0.16, 6.76, 0.01);
    anglePID = new PIDController(5, 0, 0);

    // Not quite 12v, need some control headroom
    profile =
        new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(11, 1.69, .02));
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.angleRad = getAngleRad();
    inputs.velRadPerSec = getVelRadPerSec();
    double leftMotorStatorCurrent = leftMotor.getOutputCurrent();
    double rightMotorStatorCurrent = rightMotor.getOutputCurrent();
    inputs.leftMotorCurrents.mutate(
        leftMotorStatorCurrent, leftMotorStatorCurrent * Math.abs(leftMotor.getAppliedOutput()));
    inputs.rightMotorCurrents.mutate(
        rightMotorStatorCurrent, rightMotorStatorCurrent * Math.abs(rightMotor.getAppliedOutput()));
  }

  @Override
  public void aimAtAngle(double targetAngleRad, double targetVelRadPerSec) {
    var currentAngle = getAngleRad();
    var state =
        profile.calculate(
            .2,
            new ExponentialProfile.State(currentAngle, getVelRadPerSec()),
            new ExponentialProfile.State(targetAngleRad, targetVelRadPerSec));
    state.velocity += anglePID.calculate(currentAngle, state.position);
    run(state.position, state.velocity);
  }

  @Override
  public void runVel(double velRadPerSec) {
    run(getAngleRad(), velRadPerSec);
  }

  private void run(double currentPosRad, double targetVelRadPerSec) {
    double diffControl =
        differentialPID.calculate(leftEncoder.getPosition() - rightEncoder.getPosition(), 0);

    /*
    The feedforward calculate methods that use doubles instead of the Units API are deprecated
    (See allwpilib#7280 for reasoning) but we want to avoid using the Units API at all costs.

    Unlike the C++ Units API which has 0 runtime performance costs due to being optimized out
    during compile time, the Java Units API imposes a heavy memory cost as each use of the Units
    API adds 1 object allocation, which very rapidly adds up to hundreds of allocations per tick,
    which is very bad for the extremely RAM-constrained RoboRIO.

    2027 cannot come fast enough.
    */
    //noinspection removal
    var velControl = armFeedforward.calculate(currentPosRad, targetVelRadPerSec);
    leftMotor.setVoltage(velControl + diffControl);
    rightMotor.setVoltage(velControl - diffControl);
  }

  private double getAngleRad() {
    return Units.rotationsToRadians(angleEncoder.get() - (193.5 / 360));
  }

  private double getVelRadPerSec() {
    return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
  }
}
