// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIO.PivotIOInputs inputs = new PivotIO.PivotIOInputs();
  private final PivotIOConstantsAutoLogged CONSTANTS;

  public Pivot(PivotIO pivotIO) {
    this.pivotIO = pivotIO;
    CONSTANTS = pivotIO.getConstants();
    Logger.processInputs("Superstructure/Pivot/Constants", CONSTANTS);
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(inputs);
    Logger.processInputs("Superstructure/Pivot", inputs);
  }

  public Command holdAngle() {
    return run(() -> pivotIO.runVel(0));
  }

  public Command subwooferAim() {
    return aimAtAngle(Units.degreesToRadians(49));
  }

  public Command aimAtAngle(double angleRad) {
    return run(
        () ->
            pivotIO.aimAtAngle(
                MathUtil.clamp(angleRad, CONSTANTS.minAngleRad, CONSTANTS.maxAngleRad), 0));
  }
}
