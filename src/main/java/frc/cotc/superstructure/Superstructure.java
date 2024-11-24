// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.cotc.util.Supersystem;

public class Superstructure extends Supersystem {
  private final Pivot pivot;

  public Superstructure(PivotIO pivotIO) {
    this.pivot = new Pivot(pivotIO);
  }

  public Command rest() {
    return expose(pivot.holdAngle());
  }
}
