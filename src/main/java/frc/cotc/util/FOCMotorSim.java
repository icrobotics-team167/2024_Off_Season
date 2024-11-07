// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * A class to run a simple linear physics sim with motor current as input.
 *
 * <p>WPILib's DCMotorSim takes in voltage as an input, which works well if you're running standard
 * voltage control, but breaks with current control (It assumes 0 volts = brake, but we want coast)+
 */
public class FOCMotorSim {
  private final DCMotor motor;
  private final double moi;

  public FOCMotorSim(DCMotor motor, double moiKgMetersSquared) {
    this.motor = motor;
    this.moi = moiKgMetersSquared;
  }

  private double pos = 0;
  private double vel = 0;
  private double accel = 0;

  public void tick(double current, double dt) {
    double maxCurrentDraw;
    // If accelerating (aka current and vel have the same sign), then limit current based on
    // current vel
    if (Math.signum(current) == Math.signum(vel)) {
      // Clamp current draw to the max possible draw based on current vel
      maxCurrentDraw =
          MathUtil.interpolate(
              motor.stallCurrentAmps,
              0,
              MathUtil.inverseInterpolate(0, motor.freeSpeedRadPerSec, Math.abs(vel)));

    } else {
      // Otherwise, clamp based on max stall current
      maxCurrentDraw = motor.stallCurrentAmps;
    }
    current = MathUtil.clamp(current, -maxCurrentDraw, maxCurrentDraw);

    // Torque / moi = acceleration
    accel = (current * motor.KtNMPerAmp) / moi;
    // pos = p_0 + v_0 * dt + (1/2 * a * dt^2)
    pos += vel * dt + (accel * dt * dt / 2);
    vel += accel * dt;
  }

  /**
   * @return The position in radians.
   */
  public double getPos() {
    return pos;
  }

  /**
   * @return The velocity in radians per second.
   */
  public double getVel() {
    return vel;
  }

  /**
   * @return The velocity in radians per second squared.
   */
  public double getAccel() {
    return accel;
  }
}
