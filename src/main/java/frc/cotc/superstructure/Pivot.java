// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
  private final PivotIO pivotIO;

  public Pivot(PivotIO pivotIO) {
    this.pivotIO = pivotIO;
  }

  public Command holdAngle() {
    return run(() -> pivotIO.runVel(0));
  }

  public Command subwooferAim() {
    return run(() -> pivotIO.aimAtAngle(Units.degreesToRadians(49)));
  }

  public Command aimAtAngle(double angleRad) {
    return run(() -> pivotIO.aimAtAngle(angleRad));
  }
}
