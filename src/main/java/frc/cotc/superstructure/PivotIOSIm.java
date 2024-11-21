// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.cotc.Robot;

public class PivotIOSIm implements PivotIO {
  private final SingleJointedArmSim leftArmSim;
  private final SingleJointedArmSim rightArmSim;

  private final PIDController differentialPID;
  private final ArmFeedforward armFeedforward;
  private final PIDController anglePID;
  private final PIDController velocityPID;
  private final ExponentialProfile profile;

  public PivotIOSIm() {
    var motor = DCMotor.getNeoVortex(1);
    var gearRatio = 400;
    var CoMDistanceMeters = Units.inchesToMeters(15);
    var massKg = Units.lbsToKilograms(22);
    var moiKgMetersSquared = massKg * CoMDistanceMeters * CoMDistanceMeters;
    var minAngleRad = 0.0;
    var maxAngleRad = Units.degreesToRadians(90);
    var startingAngleRad = MathUtil.interpolate(minAngleRad, maxAngleRad, Math.random());
    var angleDiff = Units.degreesToRadians(Math.random() * 10);
    leftArmSim =
        new SingleJointedArmSim(
            motor,
            gearRatio,
            moiKgMetersSquared,
            CoMDistanceMeters,
            minAngleRad,
            maxAngleRad,
            true,
            startingAngleRad + angleDiff);
    rightArmSim =
        new SingleJointedArmSim(
            motor,
            gearRatio,
            moiKgMetersSquared,
            CoMDistanceMeters,
            minAngleRad,
            maxAngleRad,
            true,
            startingAngleRad - angleDiff);

    differentialPID = new PIDController(1, 0, 0);
    armFeedforward = new ArmFeedforward(0, 0.16, 6.76, 0.01);
    anglePID = new PIDController(5, 0, 0);
    velocityPID = new PIDController(5, 0, 0);

    // Not quite 12v, need some control headroom
    profile =
        new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(11, 1.69, .02));
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    leftArmSim.update(Robot.defaultPeriodSecs);
    rightArmSim.update(Robot.defaultPeriodSecs);

    inputs.angleRad = getAngleRad();
    inputs.velRadPerSec = getVelRadPerSec();

    inputs.leftAngleRad = leftArmSim.getAngleRads();
    inputs.rightAngleRad = rightArmSim.getAngleRads();

    double leftMotorStatorCurrent = leftArmSim.getCurrentDrawAmps();
    double rightMotorStatorCurrent = rightArmSim.getCurrentDrawAmps();
    inputs.leftMotorCurrents.mutate(
        leftMotorStatorCurrent,
        leftMotorStatorCurrent
            * Math.abs(leftArmSim.getInput(0) / RobotController.getBatteryVoltage()));
    inputs.rightMotorCurrents.mutate(
        rightMotorStatorCurrent,
        rightMotorStatorCurrent
            * Math.abs(rightArmSim.getInput(0) / RobotController.getBatteryVoltage()));
  }

  @Override
  public void aimAtAngle(double targetAngleRad, double targetVelRadPerSec) {
    var currentAngle = getAngleRad();
    var state =
        profile.calculate(
            .2,
            new ExponentialProfile.State(currentAngle, getVelRadPerSec()),
            new ExponentialProfile.State(targetAngleRad, targetVelRadPerSec));
    state.velocity += anglePID.calculate(currentAngle, state.position);
    run(state.position, state.velocity);
  }

  @Override
  public void runVel(double velRadPerSec) {
    run(getAngleRad(), velRadPerSec);
  }

  private void run(double currentPosRad, double targetVelRadPerSec) {
    double diffControl =
        differentialPID.calculate(leftArmSim.getAngleRads() - rightArmSim.getAngleRads(), 0);

    /*
    The feedforward calculate methods that use doubles instead of the Units API are deprecated
    (See allwpilib#7280 for reasoning) but we want to avoid using the Units API at all costs.

    Unlike the C++ Units API which has 0 runtime performance costs due to being optimized out
    during compile time, the Java Units API imposes a heavy memory cost as each use of the Units
    API adds 1 object allocation, which very rapidly adds up to hundreds of allocations per tick,
    which is very bad for the extremely RAM-constrained RoboRIO.

    2027 cannot come fast enough.
    */
    //noinspection removal
    var velControl =
        armFeedforward.calculate(currentPosRad, targetVelRadPerSec)
            + velocityPID.calculate(getVelRadPerSec(), targetVelRadPerSec);
    leftArmSim.setInputVoltage(MathUtil.clamp(velControl + diffControl, -12, 12));
    rightArmSim.setInputVoltage(MathUtil.clamp(velControl - diffControl, -12, 12));
  }

  private double getAngleRad() {
    return (leftArmSim.getAngleRads() + rightArmSim.getAngleRads()) / 2;
  }

  private double getVelRadPerSec() {
    return (leftArmSim.getVelocityRadPerSec() + rightArmSim.getVelocityRadPerSec()) / 2;
  }
}
