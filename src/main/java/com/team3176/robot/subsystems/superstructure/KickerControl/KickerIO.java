
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.KickerControl;//Name of the folder the stuff is in 

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface KickerIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class kickerIOInputs {
    public double kickerVelocityRadPerSec = 0.0;
    public double kickerAppliedVolts = 0.0;
    public double kickerAmpsStator = 0.0;
    public double kickerAmpsSupply = 0.0;
    public double kickerTempCelcius = 0.0;

    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

    // constructor if needed for some inputs
    kickerIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(kickerIOInputs inputs) {}

  public default void setkickerVolts(double volts) {}

  public default void setCoastMode(boolean isCoastMode) {}

  public default void setkickerBrakeMode(boolean enable) {};

  public default void setkickerCurrent(double current) {};

  public default void setkickerSpeedVelocity(double speed_RPS) {};

  public default void setkickerSpeedBrakeMode(boolean enable) {};

}
