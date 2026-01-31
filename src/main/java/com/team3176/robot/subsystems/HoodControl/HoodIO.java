
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.HoodControl;//Name of the folder the stuff is in 

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface HoodIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class HoodIOInputs {
    public double HoodPositionDeg = 0.0;
    public double HoodPositionRot = 0.0;
    public double HoodPositionRotREAL =  0.0;
    public double HoodAbsolutePositionDegrees = 0.0;
    public double HoodVelocityRadPerSec = 0.0;
    public double HoodAppliedVolts = 0.0;
    public double HoodAmpsStator = 0.0;
    public double HoodAmpsSupply = 0.0;
    public double HoodTempCelcius = 0.0;
    public double Hood_pos_offset = 0.0;

    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

    // constructor if needed for some inputs
    HoodIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setHoodVolts(double volts) {}

  public default void setHoodPIDPosition(double position) {}

  public default void setCoastMode(boolean isCoastMode) {}

  public default void setHoodVoltagePos(double position) {}

  public default void setHoodBrakeMode(boolean enable) {};

  public default void setHoodCurrent(double current) {};


}
