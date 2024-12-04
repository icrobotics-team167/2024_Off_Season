// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import static frc.cotc.util.SparkUtils.configureSparks;

import com.revrobotics.RelativeEncoder;
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
  private final SparkFlex guide;
  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;
  private final RelativeEncoder guideEncoder;

  private SparkSim topMotorSim;
  private DCMotorSim topSim;
  private SparkSim bottomMotorSim;
  private DCMotorSim bottomSim;
  private SparkSim guideMotorSim;
  private DCMotorSim guideSim;

  public FlywheelIOSparkFlex() {
    var motor = DCMotor.getNeoVortex(1);

    top = new SparkFlex(22, SparkLowLevel.MotorType.kBrushless);
    bottom = new SparkFlex(26, SparkLowLevel.MotorType.kBrushless);
    guide = new SparkFlex(21, SparkLowLevel.MotorType.kBrushless);

    wheelDiameterMeters = Units.inchesToMeters(4);

    var config = new SparkFlexConfig();
    config.signals.appliedOutputPeriodMs(10);
    config.smartCurrentLimit(40);
    config.voltageCompensation(12);
    config.encoder.uvwMeasurementPeriod(16);
    config.encoder.uvwAverageDepth(2);
    config.encoder.positionConversionFactor(wheelDiameterMeters * Math.PI);
    config.encoder.velocityConversionFactor(wheelDiameterMeters * Math.PI / 60);
    configureSparks(config, top, bottom, guide);

    topEncoder = top.getEncoder();
    bottomEncoder = bottom.getEncoder();
    guideEncoder = guide.getEncoder();

    double kv = 12 / ((wheelDiameterMeters / 2) * motor.freeSpeedRadPerSec);
    CONSTANTS = new FlywheelIOConstantsAutoLogged();
    CONSTANTS.topTolerance = .1;
    CONSTANTS.bottomTolerance = .1;
    CONSTANTS.guideTolerance = .1;
    CONSTANTS.topKs = 0;
    CONSTANTS.bottomKs = 0;
    CONSTANTS.guideKs = 0;
    CONSTANTS.topKv = kv;
    CONSTANTS.bottomKv = kv;
    CONSTANTS.guideKv = kv;

    if (Robot.isSimulation()) {
      topMotorSim = new SparkSim(top, motor);
      bottomMotorSim = new SparkSim(bottom, motor);
      guideMotorSim = new SparkSim(guide, motor);

      topSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(kv, .01), motor);
      bottomSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(kv, .01), motor);
      guideSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(kv, .01), motor);
    }
  }

  @Override
  public FlywheelIOConstantsAutoLogged getConstants() {
    return CONSTANTS;
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.topPosMeters = topEncoder.getPosition();
    inputs.bottomPosMeters = bottomEncoder.getPosition();
    inputs.topVelMetersPerSec = topEncoder.getVelocity();
    inputs.bottomVelMetersPerSec = bottomEncoder.getVelocity();
    inputs.guidePosMeters = guideEncoder.getPosition();
    inputs.guideVelMetersPerSec = guideEncoder.getVelocity();

    inputs.topCurrentDraws.statorCurrent = top.getOutputCurrent();
    inputs.topCurrentDraws.supplyCurrent =
        inputs.topCurrentDraws.statorCurrent * Math.abs(top.getAppliedOutput());
    inputs.bottomCurrentDraws.statorCurrent = bottom.getOutputCurrent();
    inputs.bottomCurrentDraws.supplyCurrent =
        inputs.bottomCurrentDraws.statorCurrent * Math.abs(bottom.getAppliedOutput());
    inputs.guideCurrentDraws.statorCurrent = guide.getOutputCurrent();
    inputs.guideCurrentDraws.supplyCurrent =
        inputs.guideCurrentDraws.statorCurrent * Math.abs(guide.getAppliedOutput());
  }

  @Override
  public void run(double topVolts, double bottomVolts, double guideVolts) {
    top.setVoltage(topVolts);
    bottom.setVoltage(bottomVolts);
    guide.setVoltage(guideVolts);

    if (Robot.isSimulation()) {
      double voltage = RobotController.getBatteryVoltage();

      topSim.setInputVoltage(MathUtil.clamp(topVolts, -voltage, voltage));
      topSim.update(Robot.defaultPeriodSecs);
      topMotorSim.iterate(
          topSim.getAngularVelocityRadPerSec() * wheelDiameterMeters / 2,
          voltage,
          Robot.defaultPeriodSecs);

      bottomSim.setInputVoltage(MathUtil.clamp(bottomVolts, -voltage, voltage));
      bottomSim.update(Robot.defaultPeriodSecs);
      bottomMotorSim.iterate(
          bottomSim.getAngularVelocityRadPerSec() * wheelDiameterMeters / 2,
          voltage,
          Robot.defaultPeriodSecs);

      guideSim.setInputVoltage(MathUtil.clamp(guideVolts, -voltage, voltage));
      guideSim.update(Robot.defaultPeriodSecs);
      guideMotorSim.iterate(
          guideSim.getAngularVelocityRadPerSec() * wheelDiameterMeters / 2,
          voltage,
          Robot.defaultPeriodSecs);
    }
  }
}
