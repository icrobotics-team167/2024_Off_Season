// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.cotc.drive.Swerve;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final AutoChooser autoChooser;
  private final Swerve swerve;

  public Autos(Swerve swerve) {
    this.swerve = swerve;

    AutoFactory factory =
        Choreo.createAutoFactory(
            swerve,
            swerve::getPose,
            swerve::followTrajectory,
            () ->
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
            new AutoFactory.AutoBindings(),
            (pose, starting) -> Logger.recordOutput("Choreo/Trajectory", pose.getPoses()));
    autoChooser = new AutoChooser(factory, "Auto Selector");

    autoChooser.addAutoRoutine("Test", this::test);
  }

  private Command test(AutoFactory factory) {
    final var loop = factory.newLoop("Test");

    final var path = factory.trajectory("Test", loop);

    loop.enabled().onTrue(sequence(resetPose(path, loop), path.cmd()));

    return loop.cmd();
  }

  public void update() {
    autoChooser.update();
  }

  /**
   * Gets the auto command.
   *
   * <p>The command is deferred, meaning that it is only created on schedule.
   */
  public Command getAutoDeferred() {
    return defer(autoChooser::getSelectedAutoRoutine, Set.of(swerve));
  }

  private Command resetPose(AutoTrajectory path, AutoLoop loop) {
    return runOnce(
        () ->
            swerve.resetPose(
                path.getInitialPose()
                    .orElseGet(
                        () -> {
                          loop.kill();
                          return new Pose2d();
                        })));
  }
}
