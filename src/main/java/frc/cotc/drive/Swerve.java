// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.cotc.Robot;
import frc.cotc.vision.VisionPoseEstimator;
import frc.cotc.vision.VisionPoseEstimatorIO;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO io;
  private final SwerveIOInputsAutoLogged inputs;
  private final SwerveIOConstantsAutoLogged CONSTANTS;

  private SwerveSetpointGenerator.SwerveSetpoint lastSetpoint;
  private final SwerveSetpointGenerator setpointGenerator;
  private final SwerveSetpointGenerator.ModuleLimits limits;

  private final double MAX_LINEAR_VEL;
  private final double MAX_OMEGA;

  private final SwerveDrivePoseEstimator odometry;
  private final VisionPoseEstimator visionPoseEstimator;

  // Is 10 ms good for a timing limit? We need a limit < 20 ms to let the other parts of the bot
  // run whilst keeping the entire robot code under 20 ms loop time.
  // 50% seems fine since swerve drive is the most computationally intensive part of the bot.
  private final double timeLimit = 10; // ms
  private final Watchdog watchdog =
      new Watchdog(
          timeLimit / 1000.0,
          () ->
              DriverStation.reportWarning(
                  "Swerve.java: Loop timing limit of " + timeLimit + " ms overrun!", false));

  public Swerve(SwerveIO driveIO, VisionPoseEstimatorIO visionIO) {
    this.io = driveIO;
    inputs = new SwerveIOInputsAutoLogged();
    driveIO.updateInputs(inputs);

    // Slightly cursed way to get stats specific to each hardware implementation
    CONSTANTS = driveIO.getConstants();
    Logger.processInputs("Swerve/Constants", CONSTANTS);

    setpointGenerator =
        new SwerveSetpointGenerator(
            new Translation2d[] {
              new Translation2d(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2),
              new Translation2d(CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2),
              new Translation2d(-CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2),
              new Translation2d(-CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2)
            });
    lastSetpoint =
        new SwerveSetpointGenerator.SwerveSetpoint(
            new ChassisSpeeds(), inputs.moduleStates, new double[] {0, 0, 0, 0});

    MAX_LINEAR_VEL =
        (Units.radiansToRotations(CONSTANTS.MAX_ROTOR_VELOCITY) / CONSTANTS.DRIVE_GEAR_RATIO)
            * (CONSTANTS.WHEEL_DIAMETER * Math.PI);
    MAX_OMEGA = MAX_LINEAR_VEL / Math.hypot(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2);

    odometry =
        new SwerveDrivePoseEstimator(
            setpointGenerator.getKinematics(),
            inputs.gyroYaw,
            new SwerveModulePosition[] {
              inputs.odometryPositions[inputs.odometryPositions.length - 4],
              inputs.odometryPositions[inputs.odometryPositions.length - 3],
              inputs.odometryPositions[inputs.odometryPositions.length - 2],
              inputs.odometryPositions[inputs.odometryPositions.length - 1],
            },
            new Pose2d());

    limits =
        new SwerveSetpointGenerator.ModuleLimits(
            MAX_LINEAR_VEL,
            CONSTANTS.MAX_ACCEL,
            CONSTANTS.MAX_ROTOR_VELOCITY / CONSTANTS.STEER_GEAR_RATIO);

    visionPoseEstimator = new VisionPoseEstimator(visionIO, this::getVelocity);
  }

  @Override
  public void periodic() {
    watchdog.reset();

    io.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    watchdog.addEpoch("Periodic/updating IO inputs");

    // Update odometry
    // Parse data from odometry thread
    for (int i = 0; i < inputs.odometryTimestamps.length; i++) {
      odometry.updateWithTime(
          inputs.odometryTimestamps[i],
          inputs.odometryYaws[i],
          new SwerveModulePosition[] {
            inputs.odometryPositions[i],
            inputs.odometryPositions[i + 1],
            inputs.odometryPositions[i + 2],
            inputs.odometryPositions[i + 3],
          });
    }
    watchdog.addEpoch("Periodic/parsing odometry thread data");
    // Parse data from vision coprocessor
    visionPoseEstimator.poll(
        (Pose2d pose, double timestamp, double translationalStDevs, double rotationalStDevs) ->
            odometry.addVisionMeasurement(
                pose,
                timestamp,
                VecBuilder.fill(translationalStDevs, translationalStDevs, rotationalStDevs)));
    watchdog.addEpoch("Periodic/parsing vision data");

    Logger.recordOutput("Swerve/Odometry position", odometry.getEstimatedPosition());

    watchdog.disable();

    if (watchdog.isExpired()) {
      watchdog.printEpochs();
    }
  }

  public void drive(ChassisSpeeds speed) {
    var setpoint =
        setpointGenerator.generateSetpoint(limits, lastSetpoint, speed, Robot.defaultPeriodSecs);

    Logger.recordOutput("Swerve/Drive Setpoint", setpoint.moduleStates());
    io.drive(setpoint.moduleStates(), setpoint.steerFeedforward());

    lastSetpoint = setpoint;
  }

  public Command stopInX() {
    return run(
        () -> {
          var setpoint =
              new Rotation2d[] {
                new Rotation2d(CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2),
                new Rotation2d(-CONSTANTS.TRACK_WIDTH / 2, CONSTANTS.TRACK_LENGTH / 2),
                new Rotation2d(CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2),
                new Rotation2d(-CONSTANTS.TRACK_WIDTH / 2, -CONSTANTS.TRACK_LENGTH / 2),
              };
          io.stopWithAngles(setpoint);

          lastSetpoint =
              new SwerveSetpointGenerator.SwerveSetpoint(
                  new ChassisSpeeds(),
                  new SwerveModuleState[] {
                    new SwerveModuleState(0, setpoint[0]),
                    new SwerveModuleState(0, setpoint[1]),
                    new SwerveModuleState(0, setpoint[2]),
                    new SwerveModuleState(0, setpoint[3]),
                  },
                  new double[4]);
        });
  }

  public ChassisSpeeds getVelocity() {
    return setpointGenerator.getKinematics().toChassisSpeeds(inputs.moduleStates);
  }

  /** Command for controlling the drivebase from driver controls. */
  public Command teleopDrive(
      DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rotInput) {
    return run(() -> {
          double xVel = xInput.getAsDouble() * MAX_LINEAR_VEL;
          double yVel = yInput.getAsDouble() * MAX_LINEAR_VEL;
          double omega = rotInput.getAsDouble() * MAX_OMEGA;

          double translationalVel = Math.hypot(xVel, yVel);
          if (translationalVel > MAX_LINEAR_VEL) {
            xVel /= (translationalVel / MAX_LINEAR_VEL);
            yVel /= (translationalVel / MAX_LINEAR_VEL);
          }

          drive(
              ChassisSpeeds.discretize(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(xVel, yVel, omega), inputs.gyroYaw),
                  Robot.defaultPeriodSecs));
        })
        .withName("Swerve teleop drive");
  }

  public Command getSteerCharacterization() {
    SysIdRoutine characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> SignalLogger.writeString("SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.steerCharacterization(voltage.in(Volts)), null, this));
    return sequence(
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kForward),
        stop().withTimeout(1),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
        stop().withTimeout(1),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        stop().withTimeout(1),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
  }

  public Command choreoPathFollower(ChoreoTrajectory trajectory) {
    Timer timer = new Timer();
    return runOnce(timer::start)
        .andThen(
            run(() -> {
                  ChoreoTrajectoryState sample =
                      trajectory.sample(
                          timer.get(),
                          DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

                  SwerveSetpointGenerator.SwerveSetpoint setpoint =
                      setpointGenerator.generateSetpoint(
                          limits, lastSetpoint, sample.getChassisSpeeds(), Robot.defaultPeriodSecs);

                  double[] forceFeedforwards = new double[4];
                  for (int i = 0; i < 4; i++) {
                    Translation2d force =
                        new Translation2d(sample.moduleForcesX[i], sample.moduleForcesY[i])
                            .rotateBy(Rotation2d.fromRadians(sample.heading));
                    forceFeedforwards[i] = force.getNorm();
                  }

                  io.drive(setpoint.moduleStates(), setpoint.steerFeedforward(), forceFeedforwards);
                })
                .until(() -> timer.hasElapsed(trajectory.getTotalTime())));
  }

  public Command stop() {
    return run(
        () -> {
          io.stop();
          lastSetpoint =
              new SwerveSetpointGenerator.SwerveSetpoint(
                  new ChassisSpeeds(), inputs.moduleStates, new double[] {0, 0, 0, 0});
        });
  }
}
