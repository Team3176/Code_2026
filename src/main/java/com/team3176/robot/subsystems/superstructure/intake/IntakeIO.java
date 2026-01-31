package com.team3176.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

/** Template hardware interface for a closed loop subsystem. */
public interface IntakeIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class IntakeIOInputs {
    public double intakePositionDeg = 0.0;
    public double intakePositionRot = 0.0;
    public double intakePositionRotREAL =  0.0;
    public double intakeAbsolutePositionDegrees = 0.0;
    public double intakeVelocityRadPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeAmpsStator = 0.0;
    public double intakeAmpsSupply = 0.0;
    public double intakeTempCelcius = 0.0;
    public double intake_pos_offset = 0.0;

    public boolean upperlimitswitch = false;
    public boolean lowerlimitswitch = false;

   

    // constructor if needed for some inputs
    IntakeIOInputs() {}
  }

  /*   public default Boolean getRollerLinebreak() {} */

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

   public default void setIntakeVolts(double volts) {}

  public default void setIntakePIDPosition(double position) {}

  public default void setCoastMode(boolean isCoastMode) {}

  public default void setIntakeVoltagePos(double position) {}

  public default void setIntakeBrakeMode(boolean enable) {};

  public default void seIntakeCurrent(double current) {};

  public default void setIntakeRollerVelocity(double speed_RPS) {};

  public default void setIntakeRollerBrakeMode(boolean enable) {};
}

