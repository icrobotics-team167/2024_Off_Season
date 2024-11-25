// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import edu.wpi.first.wpilibj2.command.*;

/**
 * A class that allows for the nesting of {@link Subsystem}s.
 *
 * <p>The main use for this is to allow for more complex behavior involving default commands and
 * command compositions, giving the flexibility and mutexing of having multiple split subsystems but
 * the simplicity of having only 1 subsystem to deal with.
 *
 * <p>By wrapping a command with {@link Supersystem#expose(Command internalCommand)}, the {@link
 * CommandScheduler} can know the hierarchy of nested resources, making sure that if a {@link
 * Command} uses a nested Subsystem, that Subsystem and the parent Mechanism can be reserved for
 * mutexing, while the other Subsystems in the nest are free to do their own things.
 *
 * <p><b><i><u>NESTED SUBSYSTEMS MUST BE PRIVATE FIELDS. PUBLICLY ACCESSIBLE SUBSYSTEMS HAVE
 * ENTIRELY UNDEFINED BEHAVIOR.
 */
public abstract class Supersystem extends SubsystemBase {
  /**
   * Wraps a {@link Command} such that from the outside, it only looks like a Command that only
   * requires this {@link Supersystem}. This allows commands and command compositions involving
   * nested {@link Subsystem}s to be accessible safely.
   *
   * <p><b><i><u>ALL PUBLIC FACING COMMANDS MUST BE RUN THROUGH THIS METHOD. PUBLICLY ACCESSIBLE
   * COMMANDS FROM A NESTED SUBSYSTEM NOT WRAPPED BY THIS METHOD IS ENTIRELY UNDEFINED
   * BEHAVIOR.</b></i></u>
   *
   * @param internalCommand The command to wrap.
   * @return The wrapped command, now safe to expose to the public.
   */
  protected ProxyCommand expose(Command internalCommand) {
    var exposedCommand = internalCommand.asProxy();
    exposedCommand.addRequirements(this);
    return exposedCommand;
  }
}
