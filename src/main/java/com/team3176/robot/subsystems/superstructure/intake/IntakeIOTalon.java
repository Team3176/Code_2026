package com.team3176.robot.subsystems.superstructure.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team3176.robot.constants.Hardwaremap;
import com.team3176.robot.util.TalonUtils;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

    /** Template hardware interface for a closed loop subsystem. */
public class IntakeIOTalon implements IntakeIO {
  private TalonFX rollerController;
  private TalonFX pivotController;
  VelocityVoltage voltVelocity;
  private final PositionVoltage positionVoltage;
  private final VelocityVoltage velocityVoltage;
  VoltageOut rollerVolts = new VoltageOut(0.0);
  VoltageOut pivotVolts = new VoltageOut(0.0);
  PositionVoltage voltPosition;

    public IntakeIOTalon() {

    TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

    positionVoltage = new PositionVoltage(0).withSlot(0);
    velocityVoltage = new VelocityVoltage(0).withSlot(0);


    
  }

  
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
  }

  



  public void setRollerVolts(double volts) {
    rollerController.setControl(rollerVolts.withOutput(volts));
  }

  @Override
  public void setPivotPIDPosition(double position) {
    double desiredRotations = position * 10; // Go for plus/minus 10 rotations
    System.out.println(desiredRotations);
    rollerController.setControl(positionVoltage.withPosition(desiredRotations));
  }

  @Override
  public void setPivotVolts(double volts) {
    pivotController.setControl(pivotVolts.withOutput(volts * 12));
    System.out.println(volts);
  }

  public void setVelocityVoltage(double volts) {
    double desiredRotations = volts * 10; // Go for plus/minus 10 rotations
    System.out.println(desiredRotations);
    pivotController.setControl(velocityVoltage.withVelocity(desiredRotations));
  }
}


