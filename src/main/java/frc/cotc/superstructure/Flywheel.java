// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIO.FlywheelIOInputs inputs = new FlywheelIO.FlywheelIOInputs();
  private final FlywheelIOConstantsAutoLogged CONSTANTS;

  public Flywheel(FlywheelIO io) {
    this.io = io;
    CONSTANTS = io.getControllers();
    Logger.processInputs("Superstructure/Flywheels/Constants", CONSTANTS);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Flywheels", inputs);
  }

  private void runVel(double topVelMetersPerSec, double bottomVelMetersPerSec) {
    topVelMetersPerSec =
        MathUtil.clamp(topVelMetersPerSec, -12 / CONSTANTS.topKv, 12 / CONSTANTS.topKv);
    bottomVelMetersPerSec =
        MathUtil.clamp(bottomVelMetersPerSec, -12 / CONSTANTS.bottomKv, 12 / CONSTANTS.bottomKv);

    io.run(
        calculate(
            topVelMetersPerSec,
            inputs.topVelMetersPerSec,
            CONSTANTS.topTolerance,
            CONSTANTS.topKs,
            CONSTANTS.topKv),
        calculate(
            bottomVelMetersPerSec,
            inputs.bottomVelMetersPerSec,
            CONSTANTS.bottomTolerance,
            CONSTANTS.bottomKs,
            CONSTANTS.bottomKv));
  }

  /**
   * A bang-bang controller is mathematically optimal assuming continuous control, but we have
   * discrete control. This is a hybrid controller, using bang-bang when outside a tolerance and
   * lerping to a steady state feedforward when within the tolerance.
   */
  private double calculate(double target, double current, double tolerance, double ks, double kv) {
    if (target > current) {
      // The t value in MathUtil.interpolate is clamped to [0,1]
      return MathUtil.interpolate(target * kv, 12, (target - current) / tolerance) + ks;
    } else {
      return MathUtil.interpolate(target * kv, -12, (current - target) / tolerance) - ks;
    }
  }
}
