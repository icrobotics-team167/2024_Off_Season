// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;
import frc.cotc.util.MotorCurrentDraws;

/*
 This feels really Java-brained and dumb.
 But since the REVSim API sucks even after the 2025 library rework and the documentation is
 still bad, it's a necessity to separate sim and real unlike CTRE, so to avoid code
 duplication, this was created.
 Maybe this will get removed if REV would document the sim better.
*/
abstract class PivotIOSparkFlexAbstract implements PivotIO {
  private final PIDController differentialPID;
  private final ArmFeedforward armFeedforward;
  private final PIDController anglePID;
  private final PIDController velocityPID;
  private final ExponentialProfile profile;

  final double gearRatio = 400;

  final PivotIOConstantsAutoLogged CONSTANTS;

  public PivotIOSparkFlexAbstract() {
    DCMotor motor = DCMotor.getNeoVortex(1);
    double kv = gearRatio / motor.KvRadPerSecPerVolt;
    double ka = .01;
    differentialPID = new PIDController(1, 0, 0);
    armFeedforward = new ArmFeedforward(0, 0.16, kv, ka);
    anglePID = new PIDController(5, 0, 0);
    velocityPID = new PIDController(5, 0, 0);

    // Not quite 12v, need some control headroom
    profile =
        new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(11, kv, ka));

    CONSTANTS = new PivotIOConstantsAutoLogged();
    CONSTANTS.maxSpeedRadPerSec = motor.freeSpeedRadPerSec / gearRatio;
    CONSTANTS.maxAngleRad = Units.degreesToRadians(120);
    CONSTANTS.minAngleRad = Units.degreesToRadians(-10);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.leftAngleRad = getLeftAngleRad();
    inputs.leftVelRadPerSec = getLeftVelRadPerSec();
    inputs.rightAngleRad = getRightAngleRad();
    inputs.rightVelRadPerSec = getRightVelRadPerSec();

    updateCurrents(inputs.leftMotorCurrents, inputs.rightMotorCurrents);
  }

  @Override
  public void aimAtAngle(double angleRad, double velRadPerSec) {
    var currentAngle = getAngleRad();
    var state =
        profile.calculate(
            .2,
            new ExponentialProfile.State(currentAngle, getVelRadPerSec()),
            new ExponentialProfile.State(angleRad, velRadPerSec));
    state.velocity += anglePID.calculate(currentAngle, state.position);
    run(state.position, state.velocity);
  }

  @Override
  public void runVel(double velRadPerSec) {
    run(getAngleRad(), velRadPerSec);
  }

  private void run(double currentPosRad, double targetVelRadPerSec) {
    var leftAngleRad = getLeftAngleRad();
    var rightAngleRad = getRightAngleRad();

    double diffControl = differentialPID.calculate(leftAngleRad - rightAngleRad, 0);

    //noinspection removal
    var velControl =
        armFeedforward.calculate(
                currentPosRad, getVelRadPerSec(), targetVelRadPerSec, Robot.defaultPeriodSecs)
            + velocityPID.calculate(getVelRadPerSec(), targetVelRadPerSec);

    var leftOutput = velControl + diffControl;
    var rightOutput = velControl - diffControl;
    if ((leftAngleRad >= CONSTANTS.maxAngleRad && leftOutput > 0)
        || (leftAngleRad <= CONSTANTS.minAngleRad && leftOutput < 0)) {
      leftOutput = 0;
    }
    if ((rightAngleRad >= CONSTANTS.maxAngleRad && rightOutput > 0)
        || (rightAngleRad <= CONSTANTS.minAngleRad && rightOutput < 0)) {
      rightOutput = 0;
    }

    runVoltage(leftOutput, rightOutput);
  }

  private double getAngleRad() {
    return (getLeftAngleRad() + getRightAngleRad()) / 2;
  }

  private double getVelRadPerSec() {
    return (getLeftVelRadPerSec() + getRightVelRadPerSec()) / 2;
  }

  abstract double getLeftAngleRad();

  abstract double getLeftVelRadPerSec();

  abstract double getRightAngleRad();

  abstract double getRightVelRadPerSec();

  abstract void runVoltage(double leftVolts, double rightVolts);

  abstract void updateCurrents(MotorCurrentDraws left, MotorCurrentDraws right);
}
