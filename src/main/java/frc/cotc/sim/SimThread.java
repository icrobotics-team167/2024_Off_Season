// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.sim;

import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;

public class SimThread {
  private static final Notifier thread = new Notifier(SimThread::run);

  public static void start(double frequency) {
    thread.setName("Sim Thread");
    thread.startPeriodic(1.0 / frequency);
  }

  private static final ArrayList<Runnable> simMethods = new ArrayList<>();

  public static void addSim(Runnable sim) {
    simMethods.add(sim);
  }

  private static void run() {
    for (Runnable simMethod : simMethods) {
      simMethod.run();
    }
  }
}
