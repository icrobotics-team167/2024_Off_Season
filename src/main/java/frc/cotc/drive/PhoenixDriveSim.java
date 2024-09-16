// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class PhoenixDriveSim {
  private final PhoenixSimModule[] simModules;

  protected PhoenixDriveSim(
      TalonFX[] driveMotors,
      TalonFX[] steerMotors,
      CANcoder[] steerEncoders,
      double driveGearRatio,
      double steerGearRatio,
      boolean[] driveMotorInversions,
      boolean steerMotorInverted) {
    simModules =
        new PhoenixSimModule[] {
          new PhoenixSimModule(
              driveMotors[0],
              steerMotors[0],
              steerEncoders[0],
              driveMotorInversions[0],
              steerMotorInverted,
              driveGearRatio,
              steerGearRatio),
          new PhoenixSimModule(
              driveMotors[1],
              steerMotors[1],
              steerEncoders[1],
              driveMotorInversions[1],
              steerMotorInverted,
              driveGearRatio,
              steerGearRatio),
          new PhoenixSimModule(
              driveMotors[2],
              steerMotors[2],
              steerEncoders[2],
              driveMotorInversions[2],
              steerMotorInverted,
              driveGearRatio,
              steerGearRatio),
          new PhoenixSimModule(
              driveMotors[3],
              steerMotors[3],
              steerEncoders[3],
              driveMotorInversions[3],
              steerMotorInverted,
              driveGearRatio,
              steerGearRatio)
        };
  }

  private void run() {
    double startTime = Logger.getRealTimestamp();
    for (PhoenixSimModule module : simModules) {
      module.run();
    }
    double endTime = Logger.getRealTimestamp();
    SignalLogger.writeDouble("SimThread/Time", endTime - startTime, "Microseconds");
  }

  private final Notifier notifier = new Notifier(this::run);

  public void start(double frequency) {
    notifier.startPeriodic(1.0 / frequency);
  }

  private static class PhoenixSimModule {
    private final TalonFXSimState driveMotorSim;
    private final TalonFXSimState steerMotorSim;
    private final CANcoderSimState encoderSim;

    private final DCMotorSim driveSim;
    private final DCMotorSim steerSim;

    private final double driveGearRatio;
    private final double steerGearRatio;

    protected PhoenixSimModule(
        TalonFX driveMotor,
        TalonFX steerMotor,
        CANcoder encoder,
        boolean driveMotorInverted,
        boolean steerMotorInverted,
        double driveGearRatio,
        double steerGearRatio) {
      driveMotorSim = driveMotor.getSimState();
      steerMotorSim = steerMotor.getSimState();
      encoderSim = encoder.getSimState();

      this.driveGearRatio = driveGearRatio;
      this.steerGearRatio = steerGearRatio;

      driveSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), driveGearRatio, .05);
      steerSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1), steerGearRatio, .005);

      driveMotorSim.Orientation =
          driveMotorInverted
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;
      steerMotorSim.Orientation =
          steerMotorInverted
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;
    }

    private double lastTimestamp = -1;
    private double lastDriveRotorVel = 0;
    private double lastSteerRotorVel = 0;

    protected void run() {
      double timestamp = Logger.getRealTimestamp() / 1e6;

      if (lastTimestamp < 0) {
        steerSim.setState(2 * Math.PI * (Math.random() - .5), 0);
        steerMotorSim.setRawRotorPosition(steerSim.getAngularPositionRotations() * steerGearRatio);
      } else {
        double dt = timestamp - lastTimestamp;

        driveSim.setInputVoltage(driveMotorSim.getMotorVoltage());
        steerSim.setInputVoltage(steerMotorSim.getMotorVoltage());

        driveSim.update(timestamp - lastTimestamp);
        steerSim.update(timestamp - lastTimestamp);

        double driveRotorVel = driveSim.getAngularVelocityRPM() / 60.0 * driveGearRatio;
        double steerRotorVel = steerSim.getAngularVelocityRPM() / 60.0 * steerGearRatio;

        driveMotorSim.setRawRotorPosition(driveSim.getAngularPositionRotations() * driveGearRatio);
        driveMotorSim.setRotorVelocity(driveRotorVel);
        steerMotorSim.setRawRotorPosition(steerSim.getAngularPositionRotations() * steerGearRatio);
        steerMotorSim.setRotorVelocity(steerRotorVel);
        encoderSim.setRawPosition(steerSim.getAngularPositionRotations());
        encoderSim.setVelocity(steerSim.getAngularVelocityRPM() / 60.0);

        driveMotorSim.setRotorAcceleration((lastDriveRotorVel - driveRotorVel) / dt);
        steerMotorSim.setRotorAcceleration((lastSteerRotorVel - steerRotorVel) / dt);

        lastDriveRotorVel = driveRotorVel;
        lastSteerRotorVel = steerRotorVel;
      }

      lastTimestamp = timestamp;
    }
  }
}
