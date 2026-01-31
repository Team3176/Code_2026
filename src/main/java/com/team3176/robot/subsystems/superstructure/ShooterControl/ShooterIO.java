
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.ShooterControl;//Name of the folder the stuff is in 

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ShooterIOInputs {
    public double ShooterPositionDeg = 0.0;
    public double ShooterPositionRot = 0.0;
    public double ShooterPositionRotREAL =  0.0;
    public double ShooterAbsolutePositionDegrees = 0.0;
    public double ShooterVelocityRadPerSec = 0.0;
    public double ShooterAppliedVolts = 0.0;
    public double ShooterAmpsStator = 0.0;
    public double ShooterAmpsSupply = 0.0;
    public double ShooterTempCelcius = 0.0;
    public double Shooter_pos_offset = 0.0;

    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

    // constructor if needed for some inputs
    ShooterIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVolts(double volts) {}

  public default void setShooterPIDPosition(double position) {}

  public default void setCoastMode(boolean isCoastMode) {}

  public default void setShooterVoltagePos(double position) {}

  public default void setShooterBrakeMode(boolean enable) {};

  public default void setShooterCurrent(double current) {};

  public default void setShooterSpeedVelocity(double speed_RPS) {};

  public default void setShooterSpeedBrakeMode(boolean enable) {};

}
