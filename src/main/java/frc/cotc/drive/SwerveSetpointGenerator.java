// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.cotc.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Originally by FRC team 254, modified by Michael Jansen, and further modified by 167. See the
 * license files in the root directory of this project.
 *
 * <p>Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver, or from a path
 * follower), and outputs a new setpoint that respects all the kinematic constraints on module
 * rotation speed and wheel velocity/acceleration. By generating a new setpoint every iteration, the
 * robot will converge to the desired setpoint quickly while avoiding any intermediate state that is
 * kinematically infeasible (and can result in wheel slip or robot heading drift as a result).
 */
public class SwerveSetpointGenerator {
  private final SwerveDriveKinematics kinematics;
  private final Translation2d[] moduleLocations;
  private final DCMotor driveMotor;
  private final double driveCurrentLimitAmps;
  private final double maxDriveVelocity;
  private final double maxSteerSpeedRadPerSec;
  private final double massKg;
  private final double moiKgMetersSquared;
  private final double wheelRadiusMeters;
  private final double wheelFrictionForce;
  private final double maxTorqueFriction;

  public SwerveSetpointGenerator(
      final Translation2d[] moduleLocations,
      final DCMotor driveMotor,
      final double driveCurrentLimitAmps,
      final double maxSteerSpeedRadPerSec,
      final double massKg,
      final double moiKgMetersSquared,
      final double wheelDiameterMeters,
      final double wheelCoF) {

    this.driveMotor = driveMotor;
    this.driveCurrentLimitAmps = driveCurrentLimitAmps;
    this.maxSteerSpeedRadPerSec = maxSteerSpeedRadPerSec;
    kinematics = new SwerveDriveKinematics(moduleLocations);
    this.moduleLocations = moduleLocations;
    this.massKg = massKg;
    this.moiKgMetersSquared = moiKgMetersSquared;
    this.wheelRadiusMeters = wheelDiameterMeters / 2;
    this.maxDriveVelocity = driveMotor.freeSpeedRadPerSec * wheelRadiusMeters;

    wheelFrictionForce = wheelCoF * ((massKg / 4) * 9.81);
    maxTorqueFriction = this.wheelFrictionForce * wheelRadiusMeters;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Check if it would be faster to go to the opposite of the goal heading (and reverse drive
   * direction).
   *
   * @param prevToGoal The rotation from the previous state to the goal state (i.e.
   *     prev.inverse().rotateBy(goal)).
   * @return True if the shortest path to achieve this rotation involves flipping the drive
   *     direction.
   */
  private boolean flipHeading(Rotation2d prevToGoal) {
    return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
  }

  private double unwrapAngle(double ref, double angle) {
    double diff = angle - ref;
    if (diff > Math.PI) {
      return angle - 2.0 * Math.PI;
    } else if (diff < -Math.PI) {
      return angle + 2.0 * Math.PI;
    } else {
      return angle;
    }
  }

  private boolean chassisSpeedsNearZero(ChassisSpeeds speeds) {
    return MathUtil.isNear(0, speeds.vxMetersPerSecond, 1e-9)
        && MathUtil.isNear(0, speeds.vyMetersPerSecond, 1e-9)
        && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, 1e-9);
  }

  private boolean epsilonEquals(double a, double b) {
    return MathUtil.isNear(a, b, 1e-9);
  }

  @FunctionalInterface
  private interface Function2d {
    double f(double x, double y);
  }

  /**
   * A generated setpoint.
   *
   * @param chassisSpeeds Robot relative.
   * @param moduleStates
   * @param steerFeedforwards Radians per second.
   */
  public record SwerveSetpoint(
      ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates, double[] steerFeedforwards) {}

  /**
   * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
   * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
   * while being robust in ways that e.g. the Newton-Raphson method isn't.
   *
   * @param func The Function2d to take the root of.
   * @param x_0 x value of the lower bracket.
   * @param y_0 y value of the lower bracket.
   * @param f_0 value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param x_1 x value of the upper bracket.
   * @param y_1 y value of the upper bracket.
   * @param f_1 value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during
   *     recursion)
   * @param iterations_left Number of iterations of root finding left.
   * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the
   *     (approximate) root.
   */
  private double findRoot(
      Function2d func,
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      int iterations_left) {
    if (iterations_left < 0 || epsilonEquals(f_0, f_1)) {
      return 1.0;
    }
    var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
    var x_guess = (x_1 - x_0) * s_guess + x_0;
    var y_guess = (y_1 - y_0) * s_guess + y_0;
    var f_guess = func.f(x_guess, y_guess);
    if (Math.signum(f_0) == Math.signum(f_guess)) {
      // 0 and guess on same side of root, so use upper bracket.
      return s_guess
          + (1.0 - s_guess)
              * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
    } else {
      // Use lower bracket.
      return s_guess
          * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
    }
  }

  protected double findSteeringMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_deviation) {
    f_1 = unwrapAngle(f_0, f_1);
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_deviation) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_deviation;
    Function2d func = (x, y) -> unwrapAngle(f_0, Math.atan2(y, x)) - offset;
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, 8);
  }

  protected double findDriveMaxS(
      double x_0, double y_0, double f_0, double x_1, double y_1, double f_1, double max_vel_step) {
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_vel_step) {
      // Can go all the way to s=1.
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_vel_step;
    Function2d func = (x, y) -> Math.hypot(x, y) - offset;
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, 10);
  }

  public SwerveSetpoint generateSetpoint(
      final SwerveSetpoint prevSetpoint, ChassisSpeeds desiredState, double voltage) {
    return generateSetpoint(prevSetpoint, desiredState, voltage, Robot.defaultPeriodSecs);
  }

  private enum ActiveConstraint {
    STEER_VEL,
    CENTRIPETAL,
    MOTOR_DYNAMICS,
    CURRENT_LIMITS,
    TRACTION;
  }

  /**
   * Generate a new setpoint.
   *
   * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous
   *     iteration setpoint instead of the actual measured/estimated kinematic state.
   * @param desiredState The desired state of motion, such as from the driver sticks or a path
   *     following algorithm.
   * @param dt The loop time.
   * @return A setpoint that satisfies all the kinematic limits while converging to desiredState
   *     quickly.
   */
  public SwerveSetpoint generateSetpoint(
      final SwerveSetpoint prevSetpoint, ChassisSpeeds desiredState, double voltage, double dt) {
    SwerveModuleState[] desiredModuleState = kinematics.toSwerveModuleStates(desiredState);
    // Make sure desiredState respects velocity limits.
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, maxDriveVelocity);
    desiredState = kinematics.toChassisSpeeds(desiredModuleState);

    // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so
    // just use the previous angle.
    boolean need_to_steer = true;
    if (chassisSpeedsNearZero(desiredState)) {
      need_to_steer = false;
      for (int i = 0; i < 4; ++i) {
        desiredModuleState[i].angle = prevSetpoint.moduleStates()[i].angle;
        desiredModuleState[i].speedMetersPerSecond = 0.0;
      }
    }

    // For each module, compute local Vx and Vy vectors.
    double[] prev_vx = new double[4];
    double[] prev_vy = new double[4];
    Rotation2d[] prev_heading = new Rotation2d[4];
    double[] desired_vx = new double[4];
    double[] desired_vy = new double[4];
    Rotation2d[] desired_heading = new Rotation2d[4];
    boolean all_modules_should_flip = true;
    for (int i = 0; i < 4; ++i) {
      prev_vx[i] =
          prevSetpoint.moduleStates()[i].angle.getCos()
              * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
      prev_vy[i] =
          prevSetpoint.moduleStates()[i].angle.getSin()
              * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
      prev_heading[i] = prevSetpoint.moduleStates()[i].angle;
      if (prevSetpoint.moduleStates()[i].speedMetersPerSecond < 0.0) {
        prev_heading[i] = prev_heading[i].rotateBy(Rotation2d.fromRadians(Math.PI));
      }
      desired_vx[i] =
          desiredModuleState[i].angle.getCos() * desiredModuleState[i].speedMetersPerSecond;
      desired_vy[i] =
          desiredModuleState[i].angle.getSin() * desiredModuleState[i].speedMetersPerSecond;
      desired_heading[i] = desiredModuleState[i].angle;
      if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
        desired_heading[i] = desired_heading[i].rotateBy(Rotation2d.kPi);
      }
      if (all_modules_should_flip) {
        double required_rotation_rad =
            Math.abs(prev_heading[i].unaryMinus().rotateBy(desired_heading[i]).getRadians());
        if (required_rotation_rad < Math.PI / 2.0) {
          all_modules_should_flip = false;
        }
      }
    }
    if (all_modules_should_flip
        && !chassisSpeedsNearZero(prevSetpoint.chassisSpeeds())
        && !chassisSpeedsNearZero(desiredState)) {
      // It will (likely) be faster to stop the robot, rotate the modules in place to the complement
      // of the desired
      // angle, and accelerate again.
      return generateSetpoint(prevSetpoint, new ChassisSpeeds(), dt);
    }

    // Compute the deltas between start and goal. We can then interpolate from the start state to
    // the goal state; then
    // find the amount we can move from start towards goal in this cycle such that no kinematic
    // limit is exceeded.
    double dx = desiredState.vxMetersPerSecond - prevSetpoint.chassisSpeeds().vxMetersPerSecond;
    double dy = desiredState.vyMetersPerSecond - prevSetpoint.chassisSpeeds().vyMetersPerSecond;
    double dtheta =
        desiredState.omegaRadiansPerSecond - prevSetpoint.chassisSpeeds().omegaRadiansPerSecond;

    ActiveConstraint[] activeConstraints = new ActiveConstraint[4];

    // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at
    // desiredState.
    double min_s = 1.0;

    // In cases where an individual module is stopped, we want to remember the right steering angle
    // to command (since
    // inverse kinematics doesn't care about angle, we can be opportunistically lazy).
    List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(4);
    // Enforce steering velocity limits. We do this by taking the derivative of steering angle at
    // the current angle,
    // and then backing out the maximum interpolant between start and goal states. We remember the
    // minimum across all modules, since
    // that is the active constraint.
    for (int i = 0; i < 4; ++i) {
      if (!need_to_steer) {
        overrideSteering.add(Optional.of(prevSetpoint.moduleStates()[i].angle));
        continue;
      }
      double max_theta_step = dt * maxSteerSpeedRadPerSec;
      overrideSteering.add(Optional.empty());
      if (epsilonEquals(prevSetpoint.moduleStates()[i].speedMetersPerSecond, 0.0)) {
        // If module is stopped, we know that we will need to move straight to the final steering
        // angle, so limit based
        // purely on rotation in place.
        if (epsilonEquals(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
          // Goal angle doesn't matter. Just leave module at its current angle.
          overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates()[i].angle));
          continue;
        }

        var necessaryRotation =
            prevSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(desiredModuleState[i].angle);
        if (flipHeading(necessaryRotation)) {
          necessaryRotation = necessaryRotation.rotateBy(Rotation2d.fromRadians(Math.PI));
        }
        // getRadians() bounds to +/- Pi.
        final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

        if (numStepsNeeded <= 1.0) {
          // Steer directly to goal angle.
          overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
          // Don't limit the global min_s;
          continue;
        } else {
          // Adjust steering by max_theta_step.
          overrideSteering.set(
              i,
              Optional.of(
                  prevSetpoint.moduleStates()[i].angle.rotateBy(
                      Rotation2d.fromRadians(
                          Math.signum(necessaryRotation.getRadians()) * max_theta_step))));
          activeConstraints[i] = ActiveConstraint.STEER_VEL;
          min_s = 0.0;
          continue;
        }
      }
      if (min_s == 0.0) {
        // s can't get any lower. Save some CPU.
        continue;
      }

      // Enforce centripetal force limits to prevent sliding.
      // We do this by changing max_theta_step to the maximum change in heading over dt
      // that would create a large enough radius to keep the centripetal force under the
      // friction force.
      double maxHeadingChange =
          (dt * wheelFrictionForce)
              / ((massKg / 4) * Math.abs(prevSetpoint.moduleStates()[i].speedMetersPerSecond));
      max_theta_step = Math.min(max_theta_step, maxHeadingChange);

      activeConstraints[i] =
          max_theta_step == maxHeadingChange
              ? ActiveConstraint.CENTRIPETAL
              : ActiveConstraint.STEER_VEL;

      double s =
          findSteeringMaxS(
              prev_vx[i],
              prev_vy[i],
              prev_heading[i].getRadians(),
              desired_vx[i],
              desired_vy[i],
              desired_heading[i].getRadians(),
              max_theta_step);
      min_s = Math.min(min_s, s);
    }

    // Enforce drive wheel torque limits
    Translation2d chassisForceVec = new Translation2d();
    double chassisTorque = 0.0;
    for (int i = 0; i < 4; i++) {
      double prevSpeed = prevSetpoint.moduleStates()[i].speedMetersPerSecond;
      desiredModuleState[i].optimize(prevSetpoint.moduleStates()[i].angle);
      double desiredSpeed = desiredModuleState[i].speedMetersPerSecond;

      int forceSign;
      Rotation2d forceAngle = prevSetpoint.moduleStates()[i].angle;
      if (epsilonEquals(prevSpeed, 0.0)
          || (prevSpeed > 0 && desiredSpeed >= prevSpeed)
          || (prevSpeed < 0 && desiredSpeed <= prevSpeed)) {
        forceSign = 1; // Force will be applied in direction of module
        if (prevSpeed < 0) {
          forceAngle = forceAngle.plus(Rotation2d.k180deg);
        }
      } else {
        forceSign = -1; // Force will be applied in opposite direction of module
        if (prevSpeed > 0) {
          forceAngle = forceAngle.plus(Rotation2d.k180deg);
        }
      }

      double currentDraw;
      if (forceSign == 1) {
        double lastVelRadPerSec =
            prevSetpoint.moduleStates()[i].speedMetersPerSecond / wheelRadiusMeters;
        // Use the current battery voltage since we won't be able to supply 12v if the
        // battery is sagging down to 11v, which will affect the max torque output
        if (voltage > 12) {
          voltage = 12;
        }
        currentDraw =
            Math.min(
                driveMotor.getCurrent(Math.abs(lastVelRadPerSec), voltage), driveCurrentLimitAmps);
      } else {
        currentDraw = driveCurrentLimitAmps;
      }
      double moduleTorque = driveMotor.getTorque(currentDraw);

      if (currentDraw == driveCurrentLimitAmps) {
        activeConstraints[i] = ActiveConstraint.CURRENT_LIMITS;
      } else if (!epsilonEquals(0, currentDraw)) {
        activeConstraints[i] = ActiveConstraint.MOTOR_DYNAMICS;
      }

      // Limit torque to prevent wheel slip
      moduleTorque = Math.min(moduleTorque, maxTorqueFriction);

      if (moduleTorque == maxTorqueFriction) {
        activeConstraints[i] = ActiveConstraint.TRACTION;
        currentDraw = driveMotor.getCurrent(maxTorqueFriction);
      }
      Logger.recordOutput("Swerve/Setpoint Generator/Expected current draws/" + i, currentDraw);

      double forceAtCarpet = moduleTorque / wheelRadiusMeters;
      Translation2d moduleForceVec = new Translation2d(forceAtCarpet * forceSign, forceAngle);

      // Add the module force vector to the chassis force vector
      chassisForceVec = chassisForceVec.plus(moduleForceVec);

      // Calculate the torque this module will apply to the chassis
      if (!epsilonEquals(0, moduleForceVec.getNorm())) {
        Rotation2d angleToModule = moduleLocations[i].getAngle();
        Rotation2d theta = moduleForceVec.getAngle().minus(angleToModule);
        chassisTorque += forceAtCarpet * moduleLocations[i].getNorm() * theta.getSin();
      }
    }

    Translation2d chassisAccelVec = chassisForceVec.div(massKg);
    double chassisAngularAccel = chassisTorque / moiKgMetersSquared;

    // Use kinematics to convert chassis accelerations to module accelerations
    ChassisSpeeds chassisAccel =
        new ChassisSpeeds(chassisAccelVec.getX(), chassisAccelVec.getY(), chassisAngularAccel);
    var accelStates = kinematics.toSwerveModuleStates(chassisAccel);

    for (int i = 0; i < 4; i++) {
      if (min_s == 0.0) {
        // No need to carry on.
        break;
      }

      double maxVelStep = Math.abs(accelStates[i].speedMetersPerSecond * dt);

      double vx_min_s =
          min_s == 1.0 ? desired_vx[i] : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
      double vy_min_s =
          min_s == 1.0 ? desired_vy[i] : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
      // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we
      // already know we can't go faster than that.
      double s =
          findDriveMaxS(
              prev_vx[i],
              prev_vy[i],
              Math.hypot(prev_vx[i], prev_vy[i]),
              vx_min_s,
              vy_min_s,
              Math.hypot(vx_min_s, vy_min_s),
              maxVelStep);
      min_s = Math.min(min_s, s);
    }

    ChassisSpeeds retSpeeds =
        new ChassisSpeeds(
            prevSetpoint.chassisSpeeds.vxMetersPerSecond + min_s * dx,
            prevSetpoint.chassisSpeeds.vyMetersPerSecond + min_s * dy,
            prevSetpoint.chassisSpeeds.omegaRadiansPerSecond + min_s * dtheta);
    retSpeeds = ChassisSpeeds.discretize(retSpeeds, dt);
    var retStates = kinematics.toSwerveModuleStates(retSpeeds);
    var steerFeedforwards = new double[4];
    for (int i = 0; i < 4; ++i) {
      final var maybeOverride = overrideSteering.get(i);
      if (maybeOverride.isPresent()) {
        var override = maybeOverride.get();
        if (flipHeading(retStates[i].angle.unaryMinus().rotateBy(override))) {
          retStates[i].speedMetersPerSecond *= -1.0;
        }
        retStates[i].angle = override;
      }
      final var deltaRotation =
          prevSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(retStates[i].angle);
      if (flipHeading(deltaRotation)) {
        retStates[i].angle = retStates[i].angle.rotateBy(Rotation2d.kPi);
        retStates[i].speedMetersPerSecond *= -1.0;
      }

      steerFeedforwards[i] =
          retStates[i].angle.minus(prevSetpoint.moduleStates()[i].angle).getRadians() / dt;

      Logger.recordOutput(
          "Swerve/Setpoint Generator/Active constraints/" + i, activeConstraints[i]);
    }
    return new SwerveSetpoint(retSpeeds, retStates, steerFeedforwards);
  }
}
