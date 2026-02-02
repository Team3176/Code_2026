// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.GenericTalonControl;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
public class GenericTalonIOTalon implements GenericTalonIO {

  private TalonFX genericTalonController;
  private TalonFX genericTalonSpeedController;

  //Dual Motor Leader Follower Speed Control
  private TalonFX genericTalonLeaderSpeedController;
  private TalonFX genericTalonFollowerSpeedController;


  private CANcoder genericTalonEncoder;
  VelocityVoltage voltVelocity = new VelocityVoltage(0);
  VoltageOut genericTalonVolts = new VoltageOut(0.0);
  PositionVoltage voltPosition = new PositionVoltage(0);
  private Rotation2d encoderOffset; 
  private double genericTalon_pos_offset = 0;
  
  DigitalInput genericTalonLinebreak;

  private final StatusSignal<Voltage> genericTalonAppliedVolts;
  private final StatusSignal<Current> genericTalonCurrentAmpsStator;
  private final StatusSignal<Current> genericTalonCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> genericTalonVelocity;
  private final StatusSignal<Angle> genericTalonPosition;
  private final StatusSignal<Angle> genericTalonAbsolutePosition;
  private final StatusSignal<Temperature> genericTalonTemp;


  public GenericTalonIOTalon() {

 
    TalonFXConfiguration genericTalonConfigs = new TalonFXConfiguration();
    TalonFXConfiguration genericTalonSpeedConfigs = new TalonFXConfiguration();
    
    TalonFXConfiguration genericTalonDualSpeedConfigs = new TalonFXConfiguration();
 
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    genericTalonController = new TalonFX(Hardwaremap.genericTalon_CID, Hardwaremap.genericTalon_CBN);
    genericTalonSpeedController = new TalonFX(Hardwaremap.genericTalonSpeed_CID, Hardwaremap.genericTalonSpeed_CBN);

    genericTalonLeaderSpeedController = new TalonFX(Hardwaremap.genericTalonLeaderSpeed_CID, Hardwaremap.genericTalonDualSpeed_CBN);
    genericTalonFollowerSpeedController = new TalonFX(Hardwaremap.genericTalonFollowerSpeed_CID, Hardwaremap.genericTalonDualSpeed_CBN);


    genericTalonEncoder = new CANcoder(Hardwaremap.genericTalonCancoder_CID, Hardwaremap.genericTalon_CBN);
 

 
    //var genericTalonEncoderConfig = new CANcoderConfiguration();
    encoderOffset = Rotation2d.fromDegrees(SuperStructureConstants.GenericTalon_ENCODER_OFFSET);
    //genericTalonEncoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    

    //armPivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    genericTalonConfigs.Slot0.kP = 3; // An error of 1 rotation results in 2.4 V output
    genericTalonConfigs.Slot0.kI = 0.1; // No output for integrated error
    genericTalonConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // set max output voltage limits speed - 14V is max output available 
    genericTalonConfigs.Voltage.PeakForwardVoltage = SuperStructureConstants.GenericTalon_MAX_OUTPUT_VOLTS; 
    genericTalonConfigs.Voltage.PeakReverseVoltage = SuperStructureConstants.GenericTalon_MAXNeg_OUTPUT_VOLTS;

    genericTalonConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //TODO if position from Cancoder define which CanCoder / remote sensor to use for position feedback
    //genericTalonConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.genericTalonCancoder_CID;
    //genericTalonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    //genericTalonConfigs.Feedback.SensorToMechanismRatio = 1.0;

    genericTalonConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    genericTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    genericTalonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    genericTalonConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.6;
    genericTalonConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    genericTalonConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    genericTalonConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

    TalonUtils.applyTalonFxConfigs(genericTalonController, genericTalonConfigs);
    //genericTalonController.setPosition(0, 0);

    //SETUP SPEED CONTROL CONFIGS
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    genericTalonSpeedConfigs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    genericTalonSpeedConfigs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    genericTalonSpeedConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    genericTalonSpeedConfigs.Slot0.kI = 0; // No output for integrated error
    genericTalonSpeedConfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    genericTalonSpeedConfigs.Voltage.withPeakForwardVoltage (SuperStructureConstants.GenericTalonSpeed_MAX_OUTPUT_VOLTS)
      .withPeakReverseVoltage(SuperStructureConstants.GenericTalonSpeed_MAXNeg_OUTPUT_VOLTS);

    TalonUtils.applyTalonFxConfigs(genericTalonSpeedController, genericTalonSpeedConfigs);

    //Setup SPEED CONTROL FOR Dual Motor Leader / Follower - this definition assumes that the motors need to spin opposite directions for the mechanisim. 
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    genericTalonDualSpeedConfigs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    genericTalonDualSpeedConfigs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    genericTalonDualSpeedConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    genericTalonDualSpeedConfigs.Slot0.kI = 0; // No output for integrated error
    genericTalonDualSpeedConfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    genericTalonDualSpeedConfigs.Voltage.withPeakForwardVoltage (SuperStructureConstants.GenericTalonDualSpeed_MAX_OUTPUT_VOLTS)
      .withPeakReverseVoltage(SuperStructureConstants.GenericTalonDualSpeed_MAXNeg_OUTPUT_VOLTS);

    TalonUtils.applyTalonFxConfigs(genericTalonLeaderSpeedController, genericTalonDualSpeedConfigs);
    TalonUtils.applyTalonFxConfigs(genericTalonFollowerSpeedController, genericTalonDualSpeedConfigs);
    genericTalonFollowerSpeedController.setControl(new Follower(genericTalonLeaderSpeedController.getDeviceID(), MotorAlignmentValue.Opposed)); 



    // Set variables for viewing. - TODO select the group that makes the most sense for your mechanism 
    genericTalonAppliedVolts = genericTalonController.getMotorVoltage();
    genericTalonCurrentAmpsStator = genericTalonController.getStatorCurrent();
    genericTalonCurrentAmpsSupply = genericTalonController.getSupplyCurrent();
    genericTalonVelocity = genericTalonController.getVelocity();
    genericTalonPosition = genericTalonController.getPosition();
    
    //If you want to use a cancode use this definition 
    //genericTalonPosition = genericTalonEncoder.getPositionSinceBoot();

    genericTalonAbsolutePosition = genericTalonEncoder.getAbsolutePosition();
    genericTalonTemp = genericTalonController.getDeviceTemp();

    genericTalon_pos_offset = genericTalonEncoder.getPosition().getValueAsDouble();


    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        genericTalonAppliedVolts,
        genericTalonCurrentAmpsStator,
        genericTalonVelocity,
        genericTalonPosition,
        genericTalonTemp,
        genericTalonCurrentAmpsSupply);

    genericTalonController.optimizeBusUtilization();
    genericTalonSpeedController.optimizeBusUtilization();

    genericTalonLeaderSpeedController.optimizeBusUtilization();
    genericTalonFollowerSpeedController.optimizeBusUtilization();


  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(GenericTalonIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        genericTalonAppliedVolts,
        genericTalonCurrentAmpsStator,
        genericTalonVelocity,
        genericTalonPosition,
        genericTalonTemp,
        genericTalonCurrentAmpsSupply
        );


    inputs.genericTalonAppliedVolts = genericTalonAppliedVolts.getValueAsDouble();
    inputs.genericTalonAmpsStator = genericTalonCurrentAmpsStator.getValueAsDouble();
    inputs.genericTalonAmpsSupply = genericTalonCurrentAmpsSupply.getValueAsDouble();
    inputs.genericTalonTempCelcius = genericTalonTemp.getValueAsDouble();
    inputs.genericTalonPositionDeg = Units.rotationsToDegrees(genericTalonPosition.getValueAsDouble());
    inputs.genericTalon_pos_offset = genericTalon_pos_offset;
    inputs.genericTalonPositionRot = genericTalonController.getPosition().getValueAsDouble();
    //Use if using cancoder
    //inputs.genericTalonPositionRot = genericTalonEncoder.getPosition().getValueAsDouble() - genericTalon_pos_offset;
    inputs.genericTalonPositionRotREAL = genericTalonEncoder.getPosition().getValueAsDouble(); 
    inputs.genericTalonVelocityRadPerSec = Units.rotationsToRadians(genericTalonVelocity.getValueAsDouble());

    inputs.genericTalonAbsolutePositionDegrees =
        MathUtil.inputModulus(
            Rotation2d.fromRotations(genericTalonEncoder.getAbsolutePosition().getValueAsDouble())
                .minus(encoderOffset)
                .getDegrees(),
            -180,
            180);

  }


  //Use this to provide a speed based on voltage - it is not "controlling to speed"
  @Override
  public void setGenericTalonVolts(double volts) {
    genericTalonController.setControl(genericTalonVolts.withOutput(volts));
  }

  //Offset would be used when we need 
  @Override
  public void setGenericTalonVoltagePos(double position) {
    genericTalonController.setControl(voltPosition.withPosition(position + genericTalon_pos_offset));
  }


  @Override
  public void setGenericTalonBrakeMode(boolean enable) {
    if (enable) {
      genericTalonController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      genericTalonController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void setGenericTalonSpeedBrakeMode(boolean enable) {
    if (enable) {
      genericTalonSpeedController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      genericTalonSpeedController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

    //Offset would be used when we need 
  @Override
  public void setGenericTalonSpeedVelocity(double speed_RPS) {
    genericTalonSpeedController.setControl(voltVelocity.withVelocity(speed_RPS));
  }


  @Override
  public void setGenericTalonDualSpeedBrakeMode(boolean enable) {
    if (enable) {
      genericTalonLeaderSpeedController.setNeutralMode(NeutralModeValue.Brake);
      genericTalonFollowerSpeedController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      genericTalonLeaderSpeedController.setNeutralMode(NeutralModeValue.Coast);
      genericTalonFollowerSpeedController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

    //Offset would be used when we need 
  @Override
  public void setGenericTalonDualSpeedVelocity(double speed_RPS) {
    genericTalonLeaderSpeedController.setControl(voltVelocity.withVelocity(speed_RPS));
  }


}
