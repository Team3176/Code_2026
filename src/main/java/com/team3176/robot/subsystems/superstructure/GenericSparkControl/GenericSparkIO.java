
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.GenericSparkControl;//Name of the folder the stuff is in 

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface GenericSparkIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class GenericSparkIOInputs {
    public double genericSparkPositionDeg = 0.0;
    public double genericSparkPositionRot = 0.0;
    public double genericSparkPositionRotREAL =  0.0;
    public double genericSparkAbsolutePositionDegrees = 0.0;
    public double genericSparkVelocityRadPerSec = 0.0;
    public double genericSparkAppliedVolts = 0.0;
    public double genericSparkAmpsStator = 0.0;
    public double genericSparkAmpsSupply = 0.0;
    public double genericSparkTempCelcius = 0.0;
    public double genericSpark_pos_offset = 0.0;

    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

    // constructor if needed for some inputs
    GenericSparkIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GenericSparkIOInputs inputs) {}

  public default void setGenericSparkVolts(double volts) {}

  public default void setGenericSparkPIDPosition(double position) {}

  public default void setCoastMode(boolean isCoastMode) {}

  public default void setGenericSparkPosition(double position) {}

  public default void setGenericSparkBrakeMode(boolean enable) {};

  public default void setGenericSparkCurrent(double current) {};

  public default void setGenericSparkSpeedVelocity(double speed_RPS) {};

  public default void setGenericSparkSpeedBrakeMode(boolean enable) {};

  public default void setGenericSparkDualSpeedVelocity(double speed_RPS) {};

  public default void setGenericSparkDualSpeedBrakeMode(boolean enable) {};

}
