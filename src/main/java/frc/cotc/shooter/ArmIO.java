// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  public static final double MIN_ANGLE = 5;
  public static final double MAX_ANGLE = 105;

  @AutoLog
  class ArmIOInputs {
    double angleRad;
    double velRadPerSec;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  default void pivot(double speed) {}

  default void angleTo(double pos) {}
}
