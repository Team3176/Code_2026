
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
  public static class KickerIOInputs {
    public double KickerVelocityRadPerSec = 0.0;
    public double KickerAppliedVolts = 0.0;
    public double KickerAmpsStator = 0.0;
    public double KickerAmpsSupply = 0.0;
    public double KickerTempCelcius = 0.0;

    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

    // constructor if needed for some inputs
    KickerIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(KickerIOInputs inputs) {}

  public default void setKickerVolts(double volts) {}

  public default void setCoastMode(boolean isCoastMode) {}

  public default void setKickerBrakeMode(boolean enable) {};

  public default void setKickerCurrent(double current) {};

  public default void setKickerSpeedVelocity(double speed_RPS) {};

  public default void setKickerSpeedBrakeMode(boolean enable) {};

}
