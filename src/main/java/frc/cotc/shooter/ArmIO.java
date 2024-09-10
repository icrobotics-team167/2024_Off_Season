// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.shooter;

public interface ArmIO {
  default void goToAngle(double angleRad) {}

  default void setVelocity(double speed) {}
}
