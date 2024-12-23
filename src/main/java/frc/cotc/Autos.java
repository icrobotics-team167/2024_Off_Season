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
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.drive.Swerve;
import java.util.HashMap;
import java.util.Map;
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
            swerve::getPose,
            swerve::followTrajectory,
            Robot::isOnRed,
            swerve,
            new AutoFactory.AutoBindings());

    selector = new LoggedDashboardChooser<>("Auto Chooser");
    selector.addDefaultOption(NONE_NAME, NONE_NAME);

    lastAutoRoutineName = NONE_NAME;
    lastAutoRoutine = AutoRoutineGenerator.NONE.apply(factory).cmd();

    addAuto("Four Note Auto", this::fourNoteAuto);
  }

  private AutoRoutine fourNoteAuto(AutoFactory factory) {
    final var routine = factory.newRoutine("Four Note Auto");

    final var speakerToC1 = routine.trajectory("speakerToC1");
    final var C1ToSpeaker = routine.trajectory("C1ToSpeaker");
    final var C1ToC2 = routine.trajectory("C1ToC2");
    final var speakerToC2 = routine.trajectory("speakerToC2");
    final var C2ToSpeaker = routine.trajectory("C2ToSpeaker");
    final var C2ToC3 = routine.trajectory("C2ToC3");
    final var speakerToC3 = routine.trajectory("speakerToC3");
    final var C3ToSpeaker = routine.trajectory("C3ToSpeaker");

    // Score and then go to C1
    routine.running().onTrue(resetPose(speakerToC1, routine).andThen(speakerToC1.cmd()));

    // Go score if it has C1, skip to C2 if it doesn't
    speakerToC1.done().and(hasGamePiece(routine)).onTrue(C1ToSpeaker.cmd());
    speakerToC1.done().and(noGamePiece(routine)).onTrue(C1ToC2.cmd());

    // Once it scores C1, go to C2
    C1ToSpeaker.done().onTrue(speakerToC2.cmd());

    // Go score if it has C2, skip to C3 if it doesn't
    speakerToC2.done().and(hasGamePiece(routine)).onTrue(C2ToSpeaker.cmd());
    speakerToC2.done().and(noGamePiece(routine)).onTrue(C2ToC3.cmd());
    C1ToC2.done().and(hasGamePiece(routine)).onTrue(C2ToSpeaker.cmd());
    C1ToC2.done().and(noGamePiece(routine)).onTrue(C2ToC3.cmd());

    // Once it scores C2, go to C3
    C2ToSpeaker.done().onTrue(speakerToC3.cmd());

    // Go score regardless of status
    speakerToC3.done().onTrue(C3ToSpeaker.cmd());
    C2ToC3.done().onTrue(C3ToSpeaker.cmd());

    return routine;
  }

  private Trigger hasGamePiece(AutoRoutine routine) {
    return routine.observe(() -> true);
  }

  private Trigger noGamePiece(AutoRoutine routine) {
    return routine.observe(() -> false);
  }

  private void addAuto(
      @SuppressWarnings("SameParameterValue") String name, AutoRoutineGenerator generator) {
    autoRoutines.put(name, generator);
    selector.addOption(name, name);
  }

  private final Alert invalidAutoWarning =
      new Alert(
          "Selected an auto that isn't an option, falling back to \"" + NONE_NAME + "\"",
          Alert.AlertType.kWarning);

  public void update() {
    if (DriverStation.getAlliance().isPresent()) {
      String selectStr = selector.get();
      if (selectStr.equals(lastAutoRoutineName)) return;
      if (!autoRoutines.containsKey(selectStr)) {
        selectStr = NONE_NAME;
        invalidAutoWarning.set(true);
      } else {
        invalidAutoWarning.set(false);
      }
      lastAutoRoutineName = selectStr;
      lastAutoRoutine = autoRoutines.get(lastAutoRoutineName).apply(factory).cmd();
    }
  }

  public Command getAuto() {
    return lastAutoRoutine;
  }

  private Command resetPose(AutoTrajectory path, AutoRoutine loop) {
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
