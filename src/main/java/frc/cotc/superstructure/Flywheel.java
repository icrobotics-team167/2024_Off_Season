// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIO.FlywheelIOInputs inputs = new FlywheelIO.FlywheelIOInputs();
  private final FlywheelIO.FlywheelIOControllers controllers;

  public Flywheel(FlywheelIO io) {
    this.io = io;
    controllers = io.getControllers();
    Logger.processInputs("Superstructure/Flywheels/Constants", controllers);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Flywheels", inputs);
  }

  private void runVel(double topVelMetersPerSec, double bottomVelMetersPerSec) {
    
  }
}
