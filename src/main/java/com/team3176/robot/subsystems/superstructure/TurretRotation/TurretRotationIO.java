
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.TurretRotation;//Name of the folder the stuff is in 

import org.littletonrobotics.junction.AutoLog;


/** Template hardware interface for a closed loop subsystem. */
public interface TurretRotationIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class TurretRotationIOInputs {
    public double turretRotationPositionDeg = 0.0;
    public double turretRotationPositionRot = 0.0;
    public double turretRotationPositionRotREAL =  0.0;
    public double turretRotationAbsolutePositionDegrees = 0.0;
    public double turretRotationVelocityRadPerSec = 0.0;
    public double turretRotationAppliedVolts = 0.0;
    public double turretRotationAmpsStator = 0.0;
    public double turretRotationAmpsSupply = 0.0;
    public double turretRotationTempCelcius = 0.0;
    public double turretRotation_pos_offset = 0.0;

    public boolean turretClockwiselimitswitch = true;
    public boolean turretCounterclockwiselimitswitch = true;

    // constructor if needed for some inputs
    TurretRotationIOInputs() {}
  }



  /** Updates the set of loggable inputs. */
  public default void updateInputs(TurretRotationIOInputs inputs) {}

  public default void setTurretRotationVolts(double volts) {}

  public default void setTurretRotationVoltagePos(double position) {}

  //Use This one with limit switches
  public default void setTurretRotationVoltage(double voltage) {}

  //TrackPostionFrom Error only if vision can see targets 
  public default void setTurretRotationError(double position, boolean isVisionLocked) {} 

 }
