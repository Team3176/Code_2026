
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.ClimbControl;//Name of the folder the stuff is in 

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface ClimbControlIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimbControlIOInputs {
    public double ClimbPositionDeg = 0.0;
    public double ClimbPositionRot = 0.0;
    
    public double ClimbPositionRotREAL =  0.0;
    public double ClimbAbsolutePositionDegrees = 0.0;
    public double ClimbVelocityRadPerSec = 0.0;
    public double ClimbAppliedVolts = 0.0;
    public double ClimbAmpsStator = 0.0;
    public double ClimbAmpsSupply = 0.0;
    public double ClimbTempCelcius = 0.0;
    public double Climb_pos_offset = 0.0;

    public boolean upperlimitswitchRightClimb = false;
    public boolean upperlimitswitchLeftClimb = false;
    public boolean lowerlimitswitchRightClimb  = false;
    public boolean lowerlimitswitchLeftClimb = false;
    
    // constructor if needed for some inputs
    ClimbControlIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbControlIOInputs inputs) {}

  public default void setClimbVolts(double volts) {}

  public default void setClimbBothPos(double position) {}

  public default void setClimbLeftPos(double position) {}

  public default void setLeftVoltage(double voltage) {}

  public default void setRightVoltage(double voltage) {}

  public default void setVoltage(double voltage) {}



}
