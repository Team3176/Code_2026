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

  private TalonFX KickerController;
  private TalonFX KickerSpeedController;
  private CANcoder KickerEncoder;
  VelocityVoltage voltVelocity = new VelocityVoltage(0);
  VoltageOut KickerVolts = new VoltageOut(0.0);
  private Rotation2d encoderOffset; 

  DigitalInput KickerLinebreak;

  private final StatusSignal<Voltage> KickerAppliedVolts;
  private final StatusSignal<Current> KickerCurrentAmpsStator;
  private final StatusSignal<Current> KickerCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> KickerVelocity;
  private final StatusSignal<Temperature> KickerTemp;


  public KickerIOTalon() {

 
    TalonFXConfiguration KickerConfigs = new TalonFXConfiguration();
    TalonFXConfiguration KickerSpeedConfigs = new TalonFXConfiguration();
 
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    KickerController = new TalonFX(Hardwaremap.Kicker_CID, Hardwaremap.Kicker_CBN);
    KickerSpeedController = new TalonFX(Hardwaremap.KickerSpeed_CID, Hardwaremap.KickerSpeed_CBN);

    KickerEncoder = new CANcoder(Hardwaremap.KickerCancoder_CID, Hardwaremap.Kicker_CBN);
 

 
    //var KickerEncoderConfig = new CANcoderConfiguration();
    //KickerEncoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    

    //armPivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    KickerConfigs.Slot0.kP = 3; // An error of 1 rotation results in 2.4 V output
    KickerConfigs.Slot0.kI = 0.1; // No output for integrated error
    KickerConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // set max output voltage limits speed - 14V is max output available 
    KickerConfigs.Voltage.PeakForwardVoltage = SuperStructureConstants.Kicker_MAX_OUTPUT_VOLTS; 
    KickerConfigs.Voltage.PeakReverseVoltage = SuperStructureConstants.Kicker_MAXNeg_OUTPUT_VOLTS;

    KickerConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //TODO if position from Cancoder define which CanCoder / remote sensor to use for position feedback
    //KickerConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.KickerCancoder_CID;
    //KickerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    //KickerConfigs.Feedback.SensorToMechanismRatio = 1.0;

    KickerConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    KickerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    KickerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    KickerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.6;
    KickerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    KickerConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    KickerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

    TalonUtils.applyTalonFxConfigs(KickerController, KickerConfigs);
    //KickerController.setPosition(0, 0);

    //SETUP SPEED CONTROL CONFIGS
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    KickerSpeedConfigs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    KickerSpeedConfigs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    KickerSpeedConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    KickerSpeedConfigs.Slot0.kI = 0; // No output for integrated error
    KickerSpeedConfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    KickerSpeedConfigs.Voltage.withPeakForwardVoltage (SuperStructureConstants.KickerSpeed_MAX_OUTPUT_VOLTS)
      .withPeakReverseVoltage(SuperStructureConstants.KickerSpeed_MAXNeg_OUTPUT_VOLTS);

    TalonUtils.applyTalonFxConfigs(KickerSpeedController, KickerSpeedConfigs);


    KickerAppliedVolts = KickerController.getMotorVoltage();
    KickerCurrentAmpsStator = KickerController.getStatorCurrent();
    KickerCurrentAmpsSupply = KickerController.getSupplyCurrent();
    KickerVelocity = KickerController.getVelocity();
    
    //If you want to use a cancode use this definition 
    //KickerPosition = KickerEncoder.getPositionSinceBoot();
    KickerTemp = KickerController.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        KickerAppliedVolts,
        KickerCurrentAmpsStator,
        KickerVelocity,
        KickerTemp,
        KickerCurrentAmpsSupply);

    KickerController.optimizeBusUtilization();
    KickerSpeedController.optimizeBusUtilization();
  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(KickerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        KickerAppliedVolts,
        KickerCurrentAmpsStator,
        KickerVelocity,
        KickerTemp,
        KickerCurrentAmpsSupply
        );


    inputs.KickerAppliedVolts = KickerAppliedVolts.getValueAsDouble();
    inputs.KickerAmpsStator = KickerCurrentAmpsStator.getValueAsDouble();
    inputs.KickerAmpsSupply = KickerCurrentAmpsSupply.getValueAsDouble();
    inputs.KickerTempCelcius = KickerTemp.getValueAsDouble();
    //Use if using cancoder
    //inputs.KickerPositionRot = KickerEncoder.getPosition().getValueAsDouble() - Kicker_pos_offset;
    inputs.KickerVelocityRadPerSec = Units.rotationsToRadians(KickerVelocity.getValueAsDouble());
  }


  //Use this to provide a speed based on voltage - it is not "controlling to speed"
  @Override
  public void setKickerVolts(double volts) {
    KickerController.setControl(KickerVolts.withOutput(volts));
  }

  @Override
  public void setKickerBrakeMode(boolean enable) {
    if (enable) {
      KickerController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      KickerController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void setKickerSpeedBrakeMode(boolean enable) {
    if (enable) {
      KickerSpeedController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      KickerSpeedController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

    //Offset would be used when we need 
  @Override
  public void setKickerSpeedVelocity(double speed_RPS) {
    KickerSpeedController.setControl(voltVelocity.withVelocity(speed_RPS));
  }

}
