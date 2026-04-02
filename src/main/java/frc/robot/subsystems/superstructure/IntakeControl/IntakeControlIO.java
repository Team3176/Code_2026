
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.IntakeControl;//Name of the folder the stuff is in 

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface IntakeControlIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class IntakeControlIOInputs {
    public double IntakePositionDeg = 0.0;
    public double IntakePositionRot = 0.0;
    public double IntakePositionRotREAL =  0.0;
    public double IntakeAbsolutePositionDegrees = 0.0;
    public double IntakeVelocityRadPerSec = 0.0;
    public double IntakeAppliedVolts = 0.0;
    public double IntakeAmpsStator = 0.0;
    public double IntakeAmpsSupply = 0.0;
    public double IntakeTempCelcius = 0.0;
    public double Intake_pos_offset = 0.0;

    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

    // constructor if needed for some inputs
    IntakeControlIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeControlIOInputs inputs) {}

  public default void setIntakePositionVolts(double volts) {}

  public default void setIntakePIDPosition(double position) {}

  //public default void setIntakePostionVoltagePos(double position) {}


  public  default void setIntakePositionVoltagePosSlot0(double position) {}
  public  default void setIntakePositionVoltagePosSlot1(double position) {}

  public default void setIntakeRollerVelocity(double percentDutyCycle) {}

}
