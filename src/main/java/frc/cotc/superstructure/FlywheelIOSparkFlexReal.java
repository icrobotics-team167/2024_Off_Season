package frc.cotc.superstructure;

import frc.cotc.util.MotorCurrentDraws;

public class FlywheelIOSparkFlexReal extends FlywheelIOSparkFlexAbstract {
  public FlywheelIOSparkFlexReal() {
    // TODO: Implement
    throw new UnsupportedOperationException("This class is unimplemented!");
  }
  @Override
  double getTopPos() {
    return 0;
  }

  @Override
  double getTopVel() {
    return 0;
  }

  @Override
  double getBottomPos() {
    return 0;
  }

  @Override
  double getBottomVel() {
    return 0;
  }

  @Override
  void runVoltage(double topVolts, double bottomVolts) {

  }

  @Override
  void updateCurrentDraws(MotorCurrentDraws top, MotorCurrentDraws bottom) {

  }
}
