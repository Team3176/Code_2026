
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.ShooterControl;//Name of the folder the stuff is in 

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface ShooterControlIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ShooterControlIOInputs {
    public double shooterPositionDeg = 0.0;
    public double shooterPositionRot = 0.0;
    public double shooterPositionRotREAL =  0.0;
    public double shooterAbsolutePositionDegrees = 0.0;
    public double shooterVelocityRadPerSec = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterAmpsStator = 0.0;
    public double shooterAmpsSupply = 0.0;
    public double shooterTempCelcius = 0.0;
    public double shooter_pos_offset = 0.0;
    public double shooterVelocityRot = 0.0;

    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

    // constructor if needed for some inputs
    ShooterControlIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterControlIOInputs inputs) {}

  public default void setDualShooterSpeedVelocity(double speed_RPS) {};

  public default void setDualShooterSpeedBrakeMode(boolean enable) {};

}
