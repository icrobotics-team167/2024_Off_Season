// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.Choreo;
import choreo.auto.AutoChooser.AutoRoutineGenerator;
import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.drive.Swerve;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final Swerve swerve;

  private final AutoFactory factory;

  private final LoggedDashboardChooser<String> selector;
  private String lastAutoRoutineName;
  private Command lastAutoRoutine;

  private final String NONE_NAME = "Do Nothing";
  private final HashMap<String, AutoRoutineGenerator> autoRoutines =
      new HashMap<>(Map.of(NONE_NAME, AutoRoutineGenerator.NONE));

  public Autos(Swerve swerve) {
    this.swerve = swerve;

    factory =
        Choreo.createAutoFactory(
            swerve,
            swerve::getPose,
            swerve::followTrajectory,
            Robot::isOnRed,
            new AutoFactory.AutoBindings(),
            (pose, starting) -> Logger.recordOutput("Choreo/Trajectory", pose.getPoses()));

    selector = new LoggedDashboardChooser<>("Auto Chooser");
    selector.addDefaultOption(NONE_NAME, NONE_NAME);

    lastAutoRoutineName = NONE_NAME;
    lastAutoRoutine = AutoRoutineGenerator.NONE.apply(factory);

    addAuto("Four Note Auto", this::fourNoteAuto);
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

  private void addAuto(
      @SuppressWarnings("SameParameterValue") String name, AutoRoutineGenerator generator) {
    autoRoutines.put(name, generator);
    selector.addOption(name, name);
  }

  public void update() {
    if (DriverStation.isDisabled() || IterativeRobotBase.isSimulation()) {
      String selectStr = selector.get();
      if (selectStr.equals(lastAutoRoutineName)) return;
      if (!autoRoutines.containsKey(selectStr)) {
        selectStr = NONE_NAME;
        DriverStation.reportError(
            "Selected an auto that isn't an option, falling back to \"" + NONE_NAME + "\"", false);
      }
      lastAutoRoutineName = selectStr;
      lastAutoRoutine = autoRoutines.get(lastAutoRoutineName).apply(factory);
    }
  }

  public Command getAuto() {
    return lastAutoRoutine;
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
