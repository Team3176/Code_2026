
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.Spindexer; //folder the stuff is in 

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface SpindexerIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class SpindexerIOInputs {
    public double SpindexerPositionDeg = 0.0;
    public double SpindexerPositionRot = 0.0;
    public double SpindexerPositionRotREAL =  0.0;
    public double SpindexerAbsolutePositionDegrees = 0.0;
    public double SpindexerVelocityRadPerSec = 0.0;
    public double SpindexerAppliedVolts = 0.0;
    public double SpindexerAmpsStator = 0.0;
    public double SpindexerAmpsSupply = 0.0;
    public double SpindexerTempCelcius = 0.0;
    public double Spindexer_pos_offset = 0.0;

    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

    // constructor if needed for some inputs
    SpindexerIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void setSpindexerVolts(double volts) {}
  
  public default void setSpindexerSpeedVelocity(double speed_RPS) {};

  
}
