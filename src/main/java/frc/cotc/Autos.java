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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.drive.Swerve;
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
            Robot::isOnRed,
            new AutoFactory.AutoBindings(),
            (pose, starting) -> Logger.recordOutput("Choreo/Trajectory", pose.getPoses()));
    autoChooser = new AutoChooser(factory, "Auto Selector");

    autoChooser.addAutoRoutine("Test", this::test);
    autoChooser.addAutoRoutine("Four Note", this::fourNoteAuto);
  }

  private Command test(AutoFactory factory) {
    final var loop = factory.newLoop("Test");

    final var path = factory.trajectory("Test", loop);

    loop.enabled().onTrue(resetPose(path, loop).andThen(path.cmd()));

    return loop.cmd();
  }

  private Command fourNoteAuto(AutoFactory factory) {
    final var loop = factory.newLoop("Four Note Auto");

    final var speakerToC1 = factory.trajectory("speakerToC1", loop);
    final var C1ToSpeaker = factory.trajectory("C1ToSpeaker", loop);
    final var C1ToC2 = factory.trajectory("C1ToC2", loop);
    final var speakerToC2 = factory.trajectory("speakerToC2", loop);
    final var C2ToSpeaker = factory.trajectory("C2ToSpeaker", loop);
    final var C2ToC3 = factory.trajectory("C2ToC3", loop);
    final var speakerToC3 = factory.trajectory("speakerToC3", loop);
    final var C3ToSpeaker = factory.trajectory("C3ToSpeaker", loop);

    // Score and then go to C1
    loop.enabled().onTrue(resetPose(speakerToC1, loop).andThen(speakerToC1.cmd()));

    // Go score if it has C1, skip to C2 if it doesn't
    speakerToC1.done().and(hasGamePiece(loop)).onTrue(C1ToSpeaker.cmd());
    speakerToC1.done().and(noGamePiece(loop)).onTrue(C1ToC2.cmd());

    // Once it scores C1, go to C2
    C1ToSpeaker.done().onTrue(speakerToC2.cmd());

    // Go score if it has C2, skip to C3 if it doesn't
    speakerToC2.done().and(hasGamePiece(loop)).onTrue(C2ToSpeaker.cmd());
    speakerToC2.done().and(noGamePiece(loop)).onTrue(C2ToC3.cmd());
    C1ToC2.done().and(hasGamePiece(loop)).onTrue(C2ToSpeaker.cmd());
    C1ToC2.done().and(noGamePiece(loop)).onTrue(C2ToC3.cmd());

    // Once it scores C2, go to C3
    C2ToSpeaker.done().onTrue(speakerToC3.cmd());

    // Go score regardless of status
    speakerToC3.done().onTrue(C3ToSpeaker.cmd());
    C2ToC3.done().onTrue(C3ToSpeaker.cmd());

    return loop.cmd();
  }

  private Trigger hasGamePiece(AutoLoop loop) {
    return new Trigger(loop.getLoop(), () -> true);
  }

  private Trigger noGamePiece(AutoLoop loop) {
    return new Trigger(loop.getLoop(), () -> false);
  }

  public void update() {
    autoChooser.update();
  }

  public Command getAuto() {
    return autoChooser.getSelectedAutoRoutine();
  }

  private Command resetPose(AutoTrajectory path, AutoLoop loop) {
    return runOnce(
        () ->
            swerve.resetForAuto(
                path.getInitialPose()
                    .orElseGet(
                        () -> {
                          loop.kill();
                          return new Pose2d();
                        })));
  }
}
