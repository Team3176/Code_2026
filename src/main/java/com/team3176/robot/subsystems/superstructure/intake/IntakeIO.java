package com.team3176.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class IntakeIOInputs {

   

    // constructor if needed for some inputs
    IntakeIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVolts(double volts) {}

  public default void setPivotVolts(double volts) {}

  public default void setPivotPIDPosition(double position) {}

  public default void setVelocityVoltage(double volts) {}

  public default void setIntakePIDPosition(double rotations) {}

  public default void setIntakeVoltage(double voltage) {}


}

