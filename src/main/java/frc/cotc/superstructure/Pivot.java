// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Robot;
import org.littletonrobotics.junction.Logger;

/**
 * Compiling will throw warnings about deprecation. The feedforward calculate methods that use
 * doubles instead of the Units API are deprecated (See <a
 * href="https://github.com/wpilibsuite/allwpilib/issues/7280">allwpilib#7280</a> and <a
 * href="https://github.com/wpilibsuite/allwpilib/pull/6647#discussion_r1633591865">Tyler's comment
 * in allwpilib#6647</a>) for reasoning) but we want to avoid using the Units API at all costs.
 * Unlike the C++ Units API which has 0 runtime performance costs due to being optimized out during
 * compile time, the Java Units API imposes a heavy memory cost as each use of the Units API adds 1
 * object allocation, which very rapidly adds up to hundreds of allocations per tick, which is very
 * bad for the extremely RAM-constrained RoboRIO.
 *
 * <p>2027 cannot come fast enough.
 *
 * <p>Tada
 */
@SuppressWarnings("removal")
public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIO.PivotIOInputs inputs = new PivotIO.PivotIOInputs();
  private final PivotIOConstantsAutoLogged CONSTANTS;

  private final PIDController anglePID;
  private final PIDController diffPID;

  private final ExponentialProfile angleProfile;
  private final ArmFeedforward feedforward;

  public Pivot(PivotIO pivotIO) {
    this.io = pivotIO;
    CONSTANTS = pivotIO.getConstants();
    Logger.processInputs("Superstructure/Pivot/Constants", CONSTANTS);

    anglePID = new PIDController(CONSTANTS.angleKp, 0, CONSTANTS.angleKd);
    diffPID = new PIDController(CONSTANTS.diffKp, CONSTANTS.diffKi, CONSTANTS.diffKd);

    angleProfile =
        new ExponentialProfile(
            ExponentialProfile.Constraints.fromCharacteristics(
                12 - CONSTANTS.kG - CONSTANTS.kS, CONSTANTS.kV, CONSTANTS.kA));
    feedforward = new ArmFeedforward(CONSTANTS.kS, CONSTANTS.kG, CONSTANTS.kV, CONSTANTS.kA);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Pivot", inputs);
  }

  public Command holdAngle() {
    return run(() -> runVel(0));
  }

  public Command minAngle() {
    return aimAtAngle(CONSTANTS.minAngleRad);
  }

  public Command subwooferAim() {
    return aimAtAngle(Units.degreesToRadians(49));
  }

  public Command aimAtAngle(double angleRad) {
    return run(
        () ->
            aimAtAngle(MathUtil.clamp(angleRad, CONSTANTS.minAngleRad, CONSTANTS.maxAngleRad), 0));
  }

  private final ExponentialProfile.State currentState = new ExponentialProfile.State();
  private final ExponentialProfile.State targetState = new ExponentialProfile.State();

  private void aimAtAngle(double angleRad, double velRadPerSec) {
    currentState.position = (inputs.leftAngleRad + inputs.rightAngleRad) / 2;
    currentState.velocity = (inputs.leftVelRadPerSec + inputs.leftVelRadPerSec) / 2;
    targetState.position = angleRad;
    targetState.velocity = velRadPerSec;

    var state = angleProfile.calculate(Robot.defaultPeriodSecs, currentState, targetState);
    runVel(state.velocity + anglePID.calculate(currentState.position, state.position));
  }

  private void runVel(double velRadPerSec) {
    double diffControl = diffPID.calculate(inputs.leftAngleRad - inputs.rightAngleRad, 0);
    io.run(
        feedforward.calculate(
                inputs.leftAngleRad,
                inputs.leftVelRadPerSec,
                velRadPerSec + diffControl,
                Robot.defaultPeriodSecs)
            + ((velRadPerSec - inputs.leftVelRadPerSec) * CONSTANTS.velKp),
        feedforward.calculate(
                inputs.rightAngleRad,
                inputs.rightVelRadPerSec,
                velRadPerSec - diffControl,
                Robot.defaultPeriodSecs)
            + ((velRadPerSec - inputs.rightVelRadPerSec) * CONSTANTS.velKp));
  }
}
