// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveIOPhoenix implements SwerveIO {
  private static class Module {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder encoder;

    public Module(int id) {
      driveMotor = new TalonFX(id * 3, "Croppenheimer");
      steerMotor = new TalonFX(id * 3 + 1, "Croppenheimer");
      encoder = new CANcoder(id * 3 + 2, "Croppenheimer");
    }

    public void run(SwerveModuleState state, double forceFeedforward) {}
  }

  private static class SimThread extends Thread {
    private static class SimModule {
      private final TalonFXSimState driveMotorSim;
      private final TalonFXSimState steerMotorSim;
      private final CANcoderSimState encoderSim;

      public SimModule(Module module) {
        driveMotorSim = module.driveMotor.getSimState();
        steerMotorSim = module.steerMotor.getSimState();
        encoderSim = module.encoder.getSimState();
      }
    }
  }
}
