// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AKitIO<Constants extends LoggableInputs, Inputs extends LoggableInputs> {
  /** Gets the {@link Constants}. */
  Constants getConstants();

  /**
   * Updates the {@link Inputs} to feed in new data from the drivebase.
   *
   * @param inputs The SwerveIOInputs object. Will be mutated.
   */
  void updateInputs(Inputs inputs);
}
