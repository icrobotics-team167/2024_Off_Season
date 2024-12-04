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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Robot;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIO.PivotIOInputs inputs = new PivotIO.PivotIOInputs();
  private final PivotIOConstantsAutoLogged CONSTANTS;

  private final PIDController anglePID;
  private final PIDController diffPID;

  private final ExponentialProfile angleProfile;
  private final ArmFeedforward feedforward;

  private final InterpolatingDoubleTreeMap shotAngleLUT = new InterpolatingDoubleTreeMap();

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

  Command holdAngle() {
    return run(() -> velocityControl(0));
  }

  Command minAngle() {
    return aimAtAngle(CONSTANTS.minAngleRad);
  }

  Command subwooferAim() {
    return aimAtAngle(Units.degreesToRadians(49));
  }

  Command aimAtAngle(double angleRad) {
    return run(
        () ->
            angleControl(
                MathUtil.clamp(angleRad, CONSTANTS.minAngleRad, CONSTANTS.maxAngleRad), 0));
  }

  Command aimAtSpeaker(
      Supplier<Translation2d> positionSupplier, Supplier<ChassisSpeeds> fieldVelSupplier) {
    final Translation2d blueSpeakerPos = new Translation2d(0, 5);
    final Translation2d redSpeakerPos = new Translation2d(16.54, blueSpeakerPos.getY());
    return run(
        () -> {
          var speakerPos = Robot.isOnRed() ? redSpeakerPos : blueSpeakerPos;
          var currentPos = positionSupplier.get();
          var currentVel = fieldVelSupplier.get();
          var futurePos =
              currentPos.plus(
                  new Translation2d(
                      currentVel.vxMetersPerSecond * Robot.defaultPeriodSecs,
                      currentVel.vyMetersPerSecond * Robot.defaultPeriodSecs));

          var currentDistanceMeters = currentPos.getDistance(speakerPos);
          var futureDistanceMeters = futurePos.getDistance(speakerPos);

          var currentAngleRad = shotAngleLUT.get(currentDistanceMeters);
          var futureAngleRad = shotAngleLUT.get(futureDistanceMeters);

          angleControl(
              currentAngleRad, (futureAngleRad - currentAngleRad) / Robot.defaultPeriodSecs);
        });
  }

  private final ExponentialProfile.State currentState = new ExponentialProfile.State();
  private final ExponentialProfile.State targetState = new ExponentialProfile.State();

  private void angleControl(double angleRad, double velRadPerSec) {
    currentState.position = (inputs.leftAngleRad + inputs.rightAngleRad) / 2;
    currentState.velocity = (inputs.leftVelRadPerSec + inputs.leftVelRadPerSec) / 2;
    targetState.position = angleRad;
    targetState.velocity = velRadPerSec;

    var state = angleProfile.calculate(Robot.defaultPeriodSecs, currentState, targetState);
    velocityControl(state.velocity + anglePID.calculate(currentState.position, state.position));
  }

  /**
   * Compiling will throw warnings about deprecation. The feedforward calculate methods that use
   * doubles instead of the Units API are deprecated (See <a
   * href="https://github.com/wpilibsuite/allwpilib/issues/7280">allwpilib#7280</a> and <a
   * href="https://github.com/wpilibsuite/allwpilib/pull/6647#discussion_r1633591865">Tyler's
   * comment in allwpilib#6647</a>) for reasoning) but we want to avoid using the Units API at all
   * costs. Unlike the C++ Units API which has 0 runtime performance costs due to being optimized
   * out during compile time, the Java Units API imposes a heavy memory cost as each use of the
   * Units API adds 1 object allocation, which very rapidly adds up to hundreds of allocations per
   * tick, which is very bad for the extremely RAM-constrained RoboRIO.
   *
   * <p>2027 cannot come fast enough.
   *
   * <p>Tada
   */
  @SuppressWarnings("removal")
  private void velocityControl(double velRadPerSec) {
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
