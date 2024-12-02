// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.cotc.Robot;

public class FlywheelIOSparkFlex implements FlywheelIO {
  private final FlywheelIOConstantsAutoLogged CONSTANTS;
  private final double wheelDiameterMeters;

  private final SparkFlex top;
  private final SparkFlex bottom;
  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  private SparkSim topMotorSim;
  private DCMotorSim topSim;
  private SparkSim bottomMotorSim;
  private DCMotorSim bottomSim;

  public FlywheelIOSparkFlex() {
    var motor = DCMotor.getNeoVortex(1);

    top = new SparkFlex(17, SparkLowLevel.MotorType.kBrushless);
    bottom = new SparkFlex(18, SparkLowLevel.MotorType.kBrushless);

    wheelDiameterMeters = Units.inchesToMeters(4);

    var config = new SparkFlexConfig();
    config.signals.appliedOutputPeriodMs(10);
    config.smartCurrentLimit(40);
    config.voltageCompensation(12);
    config.encoder.uvwMeasurementPeriod(16);
    config.encoder.uvwAverageDepth(2);
    config.encoder.positionConversionFactor(wheelDiameterMeters * Math.PI);
    config.encoder.velocityConversionFactor(wheelDiameterMeters * Math.PI / 60);
    top.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);
    bottom.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kNoPersistParameters);

    topEncoder = top.getEncoder();
    bottomEncoder = bottom.getEncoder();

    CONSTANTS = new FlywheelIOConstantsAutoLogged();
    CONSTANTS.topTolerance = .1;
    CONSTANTS.bottomTolerance = .1;
    CONSTANTS.topKv = 12 / ((wheelDiameterMeters / 2) * motor.freeSpeedRadPerSec);
    CONSTANTS.bottomKv = 12 / ((wheelDiameterMeters / 2) * motor.freeSpeedRadPerSec);

    if (Robot.isSimulation()) {
      topMotorSim = new SparkSim(top, motor);
      bottomMotorSim = new SparkSim(bottom, motor);

      topSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 1, 1), motor);
      bottomSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 1, 1), motor);
    }
  }

  @Override
  public FlywheelIOConstantsAutoLogged getControllers() {
    return CONSTANTS;
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.topPosMeters = topEncoder.getPosition();
    inputs.bottomPosMeters = bottomEncoder.getPosition();
    inputs.topVelMetersPerSec = topEncoder.getVelocity();
    inputs.bottomVelMetersPerSec = bottomEncoder.getVelocity();

    inputs.topCurrentDraws.statorCurrent = top.getOutputCurrent();
    inputs.topCurrentDraws.supplyCurrent =
        inputs.topCurrentDraws.statorCurrent * Math.abs(top.getAppliedOutput());
    inputs.bottomCurrentDraws.statorCurrent = bottom.getOutputCurrent();
    inputs.bottomCurrentDraws.supplyCurrent =
        inputs.bottomCurrentDraws.statorCurrent * Math.abs(bottom.getAppliedOutput());
  }

  @Override
  public void run(double topVolts, double bottomVolts) {
    top.setVoltage(topVolts);
    bottom.setVoltage(bottomVolts);

    if (Robot.isSimulation()) {
      double voltage = RobotController.getBatteryVoltage();

      topSim.setInputVoltage(MathUtil.clamp(topVolts, -voltage, voltage));
      topSim.update(Robot.defaultPeriodSecs);
      topMotorSim.iterate(
          topSim.getAngularVelocityRadPerSec() * wheelDiameterMeters / 2,
          voltage,
          Robot.defaultPeriodSecs);

      bottomSim.setInputVoltage(MathUtil.clamp(topVolts, -voltage, voltage));
      bottomSim.update(Robot.defaultPeriodSecs);
      bottomMotorSim.iterate(
          bottomSim.getAngularVelocityRadPerSec() * wheelDiameterMeters / 2,
          voltage,
          Robot.defaultPeriodSecs);
    }
  }
}
