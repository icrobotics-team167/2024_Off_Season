// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final ArmIO io;

  public Arm(ArmIO io) {
    this.io = io;
  }

  public Command goToPosition(double angleRad) {
    return run(() -> io.goToAngle(angleRad));
  }

  public Command setVelocity(double speed) {
    return run(() -> io.setVelocity(speed));
  }
}
