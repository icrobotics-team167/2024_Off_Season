// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

public class FlywheelIOSparkFlex implements FlywheelIO {
  private final SparkFlex top;
  private final SparkFlex bottom;

  public FlywheelIOSparkFlex() {
    // TODO: Add CAN IDs
    top = new SparkFlex(17, SparkLowLevel.MotorType.kBrushless);
    bottom = new SparkFlex(18, SparkLowLevel.MotorType.kBrushless);
  }
}
