// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.Robot;
import frc.cotc.util.Supersystem;
import java.util.function.DoubleSupplier;

public class Superstructure extends Supersystem {
  private final Pivot pivot;
  private final Flywheel flywheel;

  public Superstructure(PivotIO pivotIO, FlywheelIO flywheelIO, DoubleSupplier xPosSupplier) {
    this.pivot = new Pivot(pivotIO);
    this.flywheel = new Flywheel(flywheelIO);

    new Trigger(
            () -> {
              double fieldWidthMeters = 16.54;
              if (Robot.isOnRed()) {
                return xPosSupplier.getAsDouble() > (fieldWidthMeters / 2);
              } else {
                return xPosSupplier.getAsDouble() < (fieldWidthMeters / 2);
              }
            })
        .whileTrue(speakerShot());
  }

  public Command rest() {
    return expose(pivot.holdAngle());
  }

  private Command speakerShot() {
    return flywheel.spinUp(17.25, 21.25, 21.25);
  }
}
