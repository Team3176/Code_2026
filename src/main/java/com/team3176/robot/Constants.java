// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.team3176.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /** CAN and mechanism constants. */
  public static final class CAN {
    // Turret Falcon/TalonFX motor CAN ID (placeholder - change to real ID as needed)
    public static final int TURRET_MOTOR = 11;
    // Add other CAN IDs here as needed
  }

  /** Mechanism ratios and indices. */
  public static final class Mechanism {
    // Motor rotations per turret rotation (motor spins 10 times for 1 turret rotation)
    public static final double TURRET_MOTOR_TO_TURRET_GEAR_RATIO = 10.0;

    // Shooter camera index in Vision (0-based). We append the shooter camera as the last camera.
    public static final int SHOOTER_CAMERA_INDEX = 4;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }
}
