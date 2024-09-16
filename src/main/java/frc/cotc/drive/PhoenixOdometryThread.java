// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.wpilibj.Threads;
import frc.cotc.Robot;
import java.util.concurrent.locks.ReentrantLock;

public class PhoenixOdometryThread extends Thread {
  private final int FREQUENCY = 250;
  private final int BUFFER_SIZE = (int) Math.ceil(2 * (FREQUENCY * Robot.defaultPeriodSecs));

  private final ModuleSignals[] moduleSignals;
  private final StatusSignal<Double> yawSignal;
  private final StatusSignal<Double> yawVelocity;

  private final BaseStatusSignal[] allSignals;

  private final DoubleCircularBuffer timestampBuffer = new DoubleCircularBuffer(BUFFER_SIZE);
  private final CircularBuffer<SwerveModulePosition> positionsBuffer =
      new CircularBuffer<>(4 * BUFFER_SIZE);
  private final CircularBuffer<Rotation2d> yawsBuffer = new CircularBuffer<>(BUFFER_SIZE);

  private final double WHEEL_CIRCUMFERENCE;

  protected PhoenixOdometryThread(
      ModuleSignals[] moduleSignals, Pigeon2 gyro, double wheelDiameter) {
    this.moduleSignals = moduleSignals;
    yawSignal = gyro.getYaw();
    yawVelocity = gyro.getAngularVelocityZWorld();

    allSignals = new BaseStatusSignal[18];
    for (int i = 0; i < 4; i++) {
      allSignals[i * 4] = moduleSignals[i].drivePosition;
      allSignals[i * 4 + 1] = moduleSignals[i].driveVelocity;
      allSignals[i * 4 + 2] = moduleSignals[i].steerPosition;
      allSignals[i * 4 + 3] = moduleSignals[i].steerVelocity;
    }
    allSignals[16] = yawSignal;
    allSignals[17] = yawVelocity;

    WHEEL_CIRCUMFERENCE = wheelDiameter * Math.PI;

    BaseStatusSignal.setUpdateFrequencyForAll(FREQUENCY, allSignals);

    setName("Phoenix Odometry Thread");
    Threads.setCurrentThreadPriority(true, 10);
    setDaemon(true);
  }

  private final ReentrantLock lock = new ReentrantLock();

  protected OdometryFrame getData() {
    lock.lock();
    try {
      double[] retTimestamps = new double[timestampBuffer.size()];
      for (int i = 0; i < retTimestamps.length; i++) {
        retTimestamps[i] = timestampBuffer.get(i);
      }
      timestampBuffer.clear();

      SwerveModulePosition[] retPositions = new SwerveModulePosition[positionsBuffer.size()];
      for (int i = 0; i < retPositions.length; i++) {
        retPositions[i] = positionsBuffer.get(i);
      }
      positionsBuffer.clear();

      Rotation2d[] retYaws = new Rotation2d[yawsBuffer.size()];
      for (int i = 0; i < yawsBuffer.size(); i++) {
        retYaws[i] = yawsBuffer.get(i);
      }
      yawsBuffer.clear();

      return new OdometryFrame(retTimestamps, retPositions, retYaws);
    } finally {
      lock.unlock();
    }
  }

  @Override
  public void run() {
    //noinspection InfiniteLoopStatement
    while (true) {
      BaseStatusSignal.waitForAll(2.0 / FREQUENCY, allSignals);

      lock.lock();
      try {
        double avgTimestamp = 0;

        for (int i = 0; i < 4; i++) {
          double rawDrivePosition =
              BaseStatusSignal.getLatencyCompensatedValue(
                  moduleSignals[i].drivePosition, moduleSignals[i].driveVelocity);
          double rawSteerPosition =
              BaseStatusSignal.getLatencyCompensatedValue(
                  moduleSignals[i].steerPosition, moduleSignals[i].steerVelocity);

          positionsBuffer.addLast(
              new SwerveModulePosition(
                  rawDrivePosition * WHEEL_CIRCUMFERENCE,
                  Rotation2d.fromRotations(rawSteerPosition)));

          avgTimestamp += moduleSignals[i].drivePosition.getTimestamp().getTime();
          avgTimestamp += moduleSignals[i].driveVelocity.getTimestamp().getTime();
          avgTimestamp += moduleSignals[i].steerPosition.getTimestamp().getTime();
          avgTimestamp += moduleSignals[i].steerVelocity.getTimestamp().getTime();
        }

        yawsBuffer.addLast(
            Rotation2d.fromDegrees(
                BaseStatusSignal.getLatencyCompensatedValue(yawSignal, yawVelocity)));

        avgTimestamp += yawSignal.getTimestamp().getTime();
        avgTimestamp += yawVelocity.getTimestamp().getTime();

        avgTimestamp /= 18;
        timestampBuffer.addLast(avgTimestamp);
      } finally {
        lock.unlock();
      }
    }
  }

  protected record ModuleSignals(
      StatusSignal<Double> drivePosition,
      StatusSignal<Double> driveVelocity,
      StatusSignal<Double> steerPosition,
      StatusSignal<Double> steerVelocity) {}

  protected record OdometryFrame(
      double[] timestamps, SwerveModulePosition[] modulePositions, Rotation2d[] yaws) {}
}
