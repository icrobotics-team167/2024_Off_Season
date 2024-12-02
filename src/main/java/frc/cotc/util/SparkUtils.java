// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public final class SparkUtils {
  private SparkUtils() {}

  /**
   * If an attempt to set configurations on a Spark max fails, spam retries until it succeeds. Gives
   * up if it fails after 5 attempts.
   *
   * <p>The 2025 library updates should in theory make this unnecessary, but TBH IDK how much I
   * trust that especially since the last attempt at fixing the "configs doesn't actually apply"
   * problem didn't work.
   */
  public static void configureSpark(Supplier<REVLibError> config) {
    for (int i = 0; i < 5; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
    }
    DriverStation.reportWarning("Failed to configure spark!", true);
  }
}
