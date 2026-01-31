
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.GenericTalonControl;//Name of the folder the stuff is in 

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface GenericTalonIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class GenericTalonIOInputs {
    public double genericTalonPositionDeg = 0.0;
    public double genericTalonPositionRot = 0.0;
    public double genericTalonPositionRotREAL =  0.0;
    public double genericTalonAbsolutePositionDegrees = 0.0;
    public double genericTalonVelocityRadPerSec = 0.0;
    public double genericTalonAppliedVolts = 0.0;
    public double genericTalonAmpsStator = 0.0;
    public double genericTalonAmpsSupply = 0.0;
    public double genericTalonTempCelcius = 0.0;
    public double genericTalon_pos_offset = 0.0;

    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

    // constructor if needed for some inputs
    GenericTalonIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GenericTalonIOInputs inputs) {}

  public default void setGenericTalonVolts(double volts) {}

  public default void setGenericTalonPIDPosition(double position) {}

  public default void setCoastMode(boolean isCoastMode) {}

  public default void setGenericTalonVoltagePos(double position) {}

  public default void setGenericTalonBrakeMode(boolean enable) {};

  public default void setGenericTalonCurrent(double current) {};

  public default void setGenericTalonSpeedVelocity(double speed_RPS) {};

  public default void setGenericTalonSpeedBrakeMode(boolean enable) {};

}
