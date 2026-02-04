// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.ShooterControl;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import com.team3176.robot.constants.Hardwaremap;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.util.TalonUtils;


/** Template hardware interface for a closed loop subsystem. */
public class ShooterIOTalon implements ShooterIO {

  private TalonFX ShooterController;
  private TalonFX ShooterSpeedController;
  private CANcoder ShooterEncoder;
  VelocityVoltage voltVelocity = new VelocityVoltage(0);
  VoltageOut ShooterVolts = new VoltageOut(0.0);
  PositionVoltage voltPosition = new PositionVoltage(0);
  private Rotation2d encoderOffset; 
  private double Shooter_pos_offset = 0;
  
  DigitalInput ShooterLinebreak;

  private final StatusSignal<Voltage> ShooterAppliedVolts;
  private final StatusSignal<Current> ShooterCurrentAmpsStator;
  private final StatusSignal<Current> ShooterCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> ShooterVelocity;
  private final StatusSignal<Angle> ShooterPosition;
  private final StatusSignal<Angle> ShooterAbsolutePosition;
  private final StatusSignal<Temperature> ShooterTemp;


  public ShooterIOTalon() {

 
    TalonFXConfiguration ShooterConfigs = new TalonFXConfiguration();
    TalonFXConfiguration ShooterSpeedConfigs = new TalonFXConfiguration();
 
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    ShooterController = new TalonFX(Hardwaremap.Shooter_CID, Hardwaremap.Shooter_CBN);
    ShooterSpeedController = new TalonFX(Hardwaremap.ShooterSpeed_CID, Hardwaremap.ShooterSpeed_CBN);

    ShooterEncoder = new CANcoder(Hardwaremap.ShooterCancoder_CID, Hardwaremap.Shooter_CBN);
 

 
    //var ShooterEncoderConfig = new CANcoderConfiguration();
    encoderOffset = Rotation2d.fromDegrees(SuperStructureConstants.Shooter_ENCODER_OFFSET);
    //ShooterEncoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    

    //armPivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    /* ShooterConfigs.Slot0.kP = 3; // An error of 1 rotation results in 2.4 V output
    ShooterConfigs.Slot0.kI = 0.1; // No output for integrated error
    ShooterConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // set max output voltage limits speed - 14V is max output available 
    ShooterConfigs.Voltage.PeakForwardVoltage = SuperStructureConstants.Shooter_MAX_OUTPUT_VOLTS; 
    ShooterConfigs.Voltage.PeakReverseVoltage = SuperStructureConstants.Shooter_MAXNeg_OUTPUT_VOLTS;

    ShooterConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //TODO if position from Cancoder define which CanCoder / remote sensor to use for position feedback
    //ShooterConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.ShooterCancoder_CID;
    //ShooterConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    //ShooterConfigs.Feedback.SensorToMechanismRatio = 1.0;

    ShooterConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    ShooterConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    ShooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    ShooterConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.6;
    ShooterConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    ShooterConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    ShooterConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

    TalonUtils.applyTalonFxConfigs(ShooterController, ShooterConfigs); */
    //ShooterController.setPosition(0, 0);


    ShooterConfigs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    ShooterConfigs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    ShooterConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    ShooterConfigs.Slot0.kI = 0; // No output for integrated error
    ShooterConfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    ShooterConfigs.Voltage.withPeakForwardVoltage (SuperStructureConstants.ShooterSpeed_MAX_OUTPUT_VOLTS)
      .withPeakReverseVoltage(SuperStructureConstants.ShooterSpeed_MAXNeg_OUTPUT_VOLTS);

    TalonUtils.applyTalonFxConfigs(ShooterController, ShooterConfigs);
    // Shooter Controller

    //SETUP SPEED CONTROL CONFIGS
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    ShooterSpeedConfigs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    ShooterSpeedConfigs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    ShooterSpeedConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    ShooterSpeedConfigs.Slot0.kI = 0; // No output for integrated error
    ShooterSpeedConfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    ShooterSpeedConfigs.Voltage.withPeakForwardVoltage (SuperStructureConstants.ShooterSpeed_MAX_OUTPUT_VOLTS)
      .withPeakReverseVoltage(SuperStructureConstants.ShooterSpeed_MAXNeg_OUTPUT_VOLTS);

    TalonUtils.applyTalonFxConfigs(ShooterSpeedController, ShooterSpeedConfigs);


    ShooterAppliedVolts = ShooterController.getMotorVoltage();
    ShooterCurrentAmpsStator = ShooterController.getStatorCurrent();
    ShooterCurrentAmpsSupply = ShooterController.getSupplyCurrent();
    ShooterVelocity = ShooterController.getVelocity();
    ShooterPosition = ShooterController.getPosition();
    
    //If you want to use a cancode use this definition 
    //ShooterPosition = ShooterEncoder.getPositionSinceBoot();
    ShooterAbsolutePosition = ShooterEncoder.getAbsolutePosition();
    ShooterTemp = ShooterController.getDeviceTemp();

    Shooter_pos_offset = ShooterEncoder.getPosition().getValueAsDouble();


    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        ShooterAppliedVolts,
        ShooterCurrentAmpsStator,
        ShooterVelocity,
        ShooterPosition,
        ShooterTemp,
        ShooterCurrentAmpsSupply);

    ShooterController.optimizeBusUtilization();
    ShooterSpeedController.optimizeBusUtilization();
  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        ShooterAppliedVolts,
        ShooterCurrentAmpsStator,
        ShooterVelocity,
        ShooterPosition,
        ShooterTemp,
        ShooterCurrentAmpsSupply
        );


    inputs.ShooterAppliedVolts = ShooterAppliedVolts.getValueAsDouble();
    inputs.ShooterAmpsStator = ShooterCurrentAmpsStator.getValueAsDouble();
    inputs.ShooterAmpsSupply = ShooterCurrentAmpsSupply.getValueAsDouble();
    inputs.ShooterTempCelcius = ShooterTemp.getValueAsDouble();
    inputs.ShooterPositionDeg = Units.rotationsToDegrees(ShooterPosition.getValueAsDouble());
    inputs.Shooter_pos_offset = Shooter_pos_offset;
    inputs.ShooterPositionRot = ShooterController.getPosition().getValueAsDouble();
    //Use if using cancoder
    //inputs.ShooterPositionRot = ShooterEncoder.getPosition().getValueAsDouble() - Shooter_pos_offset;
    inputs.ShooterPositionRotREAL = ShooterEncoder.getPosition().getValueAsDouble(); 
    inputs.ShooterVelocityRadPerSec = Units.rotationsToRadians(ShooterVelocity.getValueAsDouble());

    inputs.ShooterAbsolutePositionDegrees =
        MathUtil.inputModulus(
            Rotation2d.fromRotations(ShooterEncoder.getAbsolutePosition().getValueAsDouble())
                .minus(encoderOffset)
                .getDegrees(),
            -180,
            180);

  }


  //Use this to provide a speed based on voltage - it is not "controlling to speed"
  @Override
  public void setShooterVolts(double volts) {
    ShooterController.setControl(ShooterVolts.withOutput(volts));
  }

  //Offset would be used when we need 
  @Override
  public void setShooterVoltagePos(double speed_RPS) {
    ShooterController.setControl(voltVelocity.withVelocity(speed_RPS));
  }


  @Override
  public void setShooterBrakeMode(boolean enable) {
    if (enable) {
      ShooterController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      ShooterController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void setShooterSpeedBrakeMode(boolean enable) {
    if (enable) {
      ShooterSpeedController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      ShooterSpeedController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

    //Offset would be used when we need 
  @Override
  public void setShooterSpeedVelocity(double speed_RPS) {
    ShooterSpeedController.setControl(voltVelocity.withVelocity(speed_RPS));
  }

}
