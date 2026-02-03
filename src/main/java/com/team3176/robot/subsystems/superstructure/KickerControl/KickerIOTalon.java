// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.KickerControl;

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
public class KickerIOTalon implements KickerIO {

  private TalonFX kickerController;
  private TalonFX kickerSpeedController;
  private CANcoder kickerEncoder;
  VelocityVoltage voltVelocity = new VelocityVoltage(0);
  VoltageOut kickerVolts = new VoltageOut(0.0);
  private Rotation2d encoderOffset; 

  DigitalInput kickerLinebreak;

  private final StatusSignal<Voltage> kickerAppliedVolts;
  private final StatusSignal<Current> kickerCurrentAmpsStator;
  private final StatusSignal<Current> kickerCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> kickerVelocity;
  private final StatusSignal<Temperature> kickerTemp;


  public KickerIOTalon() {

 
    TalonFXConfiguration kickerConfigs = new TalonFXConfiguration();
    TalonFXConfiguration kickerSpeedConfigs = new TalonFXConfiguration();
 
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    kickerController = new TalonFX(Hardwaremap.Kicker_CID, Hardwaremap.Kicker_CBN);
    kickerSpeedController = new TalonFX(Hardwaremap.KickerSpeed_CID, Hardwaremap.KickerSpeed_CBN);

    kickerEncoder = new CANcoder(Hardwaremap.KickerCancoder_CID, Hardwaremap.Kicker_CBN);
 

 
    //var kickerEncoderConfig = new CANcoderConfiguration();
    //kickerEncoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    

    //armPivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    kickerConfigs.Slot0.kP = 3; // An error of 1 rotation results in 2.4 V output
    kickerConfigs.Slot0.kI = 0.1; // No output for integrated error
    kickerConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // set max output voltage limits speed - 14V is max output available 
    kickerConfigs.Voltage.PeakForwardVoltage = SuperStructureConstants.Kicker_MAX_OUTPUT_VOLTS; 
    kickerConfigs.Voltage.PeakReverseVoltage = SuperStructureConstants.Kicker_MAXNeg_OUTPUT_VOLTS;

    kickerConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //TODO if position from Cancoder define which CanCoder / remote sensor to use for position feedback
    //kickerConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.kickerCancoder_CID;
    //kickerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    //kickerConfigs.Feedback.SensorToMechanismRatio = 1.0;

    kickerConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    kickerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    kickerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.6;
    kickerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    kickerConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    kickerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

    TalonUtils.applyTalonFxConfigs(kickerController, kickerConfigs);
    //kickerController.setPosition(0, 0);

    //SETUP SPEED CONTROL CONFIGS
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    kickerSpeedConfigs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    kickerSpeedConfigs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    kickerSpeedConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    kickerSpeedConfigs.Slot0.kI = 0; // No output for integrated error
    kickerSpeedConfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    kickerSpeedConfigs.Voltage.withPeakForwardVoltage (SuperStructureConstants.KickerSpeed_MAX_OUTPUT_VOLTS)
      .withPeakReverseVoltage(SuperStructureConstants.KickerSpeed_MAXNeg_OUTPUT_VOLTS);

    TalonUtils.applyTalonFxConfigs(kickerSpeedController, kickerSpeedConfigs);


    kickerAppliedVolts = kickerController.getMotorVoltage();
    kickerCurrentAmpsStator = kickerController.getStatorCurrent();
    kickerCurrentAmpsSupply = kickerController.getSupplyCurrent();
    kickerVelocity = kickerController.getVelocity();
    
    //If you want to use a cancode use this definition 
    //kickerPosition = kickerEncoder.getPositionSinceBoot();
    kickerTemp = kickerController.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        kickerAppliedVolts,
        kickerCurrentAmpsStator,
        kickerVelocity,
        kickerTemp,
        kickerCurrentAmpsSupply);

    kickerController.optimizeBusUtilization();
    kickerSpeedController.optimizeBusUtilization();
  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(kickerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        kickerAppliedVolts,
        kickerCurrentAmpsStator,
        kickerVelocity,
        kickerTemp,
        kickerCurrentAmpsSupply
        );


    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerAmpsStator = kickerCurrentAmpsStator.getValueAsDouble();
    inputs.kickerAmpsSupply = kickerCurrentAmpsSupply.getValueAsDouble();
    inputs.kickerTempCelcius = kickerTemp.getValueAsDouble();
    //Use if using cancoder
    //inputs.kickerPositionRot = kickerEncoder.getPosition().getValueAsDouble() - kicker_pos_offset;
    inputs.kickerVelocityRadPerSec = Units.rotationsToRadians(kickerVelocity.getValueAsDouble());
  }


  //Use this to provide a speed based on voltage - it is not "controlling to speed"
  @Override
  public void setkickerVolts(double volts) {
    kickerController.setControl(kickerVolts.withOutput(volts));
  }

  @Override
  public void setkickerBrakeMode(boolean enable) {
    if (enable) {
      kickerController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      kickerController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void setkickerSpeedBrakeMode(boolean enable) {
    if (enable) {
      kickerSpeedController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      kickerSpeedController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

    //Offset would be used when we need 
  @Override
  public void setkickerSpeedVelocity(double speed_RPS) {
    kickerSpeedController.setControl(voltVelocity.withVelocity(speed_RPS));
  }

}
