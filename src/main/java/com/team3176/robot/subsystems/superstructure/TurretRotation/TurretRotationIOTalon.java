// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project

package com.team3176.robot.subsystems.superstructure.TurretRotation;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import java.io.PrintStream;

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
public class TurretRotationIOTalon implements TurretRotationIO {

  private TalonFX turretRotationController;



  private CANcoder turretRotationEncoder;
  VelocityVoltage voltVelocity = new VelocityVoltage(0);
  VoltageOut turretRotationVolts = new VoltageOut(0.0);
  PositionVoltage voltPosition = new PositionVoltage(0);
  private Rotation2d encoderOffset; 
  private double turretRotation_pos_offset = 0;
  
  DigitalInput clockwiselimitswitch, counterclockwiselimitswitch ;

  private final StatusSignal<Voltage> turretRotationAppliedVolts;
  private final StatusSignal<Current> turretRotationCurrentAmpsStator;
  private final StatusSignal<Current> turretRotationCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> turretRotationVelocity;
  private final StatusSignal<Angle> turretRotationPosition;
  //private final StatusSignal<Double> turretError_Vision;
  private final StatusSignal<Angle> turretRotationAbsolutePosition;
  private final StatusSignal<Temperature> turretRotationTemp;


  public TurretRotationIOTalon() {

 
    TalonFXConfiguration turretRotationConfigs = new TalonFXConfiguration();
  
    clockwiselimitswitch = new DigitalInput(Hardwaremap.turretClockwiseLimitSwitch_DIO);
    counterclockwiselimitswitch = new DigitalInput(Hardwaremap.turretCounterClockwiseLimitSwitch_DIO);
 
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    turretRotationController = new TalonFX(Hardwaremap.turretRotation_CID, Hardwaremap.turretRotation_CBN);
    

    turretRotationEncoder = new CANcoder(Hardwaremap.turretRotationCancoder_CID, Hardwaremap.turretRotation_CBN);
 

 
    //var turretRotationEncoderConfig = new CANcoderConfiguration();
    encoderOffset = Rotation2d.fromDegrees(SuperStructureConstants.TurretRotation_ENCODER_OFFSET);
    //turretRotationEncoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    

    //armPivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    turretRotationConfigs.Slot0.kP = 3; // An error of 1 rotation results in 2.4 V output
    turretRotationConfigs.Slot0.kI = 0.1; // No output for integrated error
    turretRotationConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // set max output voltage limits speed - 14V is max output available 
    turretRotationConfigs.Voltage.PeakForwardVoltage = SuperStructureConstants.TurretRotation_MAX_OUTPUT_VOLTS; 
    turretRotationConfigs.Voltage.PeakReverseVoltage = SuperStructureConstants.TurretRotation_MAXNeg_OUTPUT_VOLTS;

    turretRotationConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //TODO if position from Cancoder define which CanCoder / remote sensor to use for position feedback
    //turretRotationConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.turretRotationCancoder_CID;
    //turretRotationConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    //turretRotationConfigs.Feedback.SensorToMechanismRatio = 1.0;

    turretRotationConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    turretRotationConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    turretRotationConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    turretRotationConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.6;
    turretRotationConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    turretRotationConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    turretRotationConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

    TalonUtils.applyTalonFxConfigs(turretRotationController, turretRotationConfigs);
    //turretRotationController.setPosition(0, 0);

  

    // Set variables for viewing. - TODO select the group that makes the most sense for your mechanism 
    turretRotationAppliedVolts = turretRotationController.getMotorVoltage();
    turretRotationCurrentAmpsStator = turretRotationController.getStatorCurrent();
    turretRotationCurrentAmpsSupply = turretRotationController.getSupplyCurrent();
    turretRotationVelocity = turretRotationController.getVelocity();
    turretRotationPosition = turretRotationController.getPosition();
   
    
    //If you want to use a cancode use this definition 
    //turretRotationPosition = turretRotationEncoder.getPositionSinceBoot();

    turretRotationAbsolutePosition = turretRotationEncoder.getAbsolutePosition();
    turretRotationTemp = turretRotationController.getDeviceTemp();

    turretRotation_pos_offset = turretRotationEncoder.getPosition().getValueAsDouble();


    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        turretRotationAppliedVolts,
        turretRotationCurrentAmpsStator,
        turretRotationVelocity,
        turretRotationPosition,
        turretRotationTemp,
        turretRotationCurrentAmpsSupply);

    turretRotationController.optimizeBusUtilization();

  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(TurretRotationIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        turretRotationAppliedVolts,
        turretRotationCurrentAmpsStator,
        turretRotationVelocity,
        turretRotationPosition,
        turretRotationTemp,
        turretRotationCurrentAmpsSupply
        );

    inputs.turretClockwiselimitswitch = (!clockwiselimitswitch.get());
    inputs.turretCounterclockwiselimitswitch = (!counterclockwiselimitswitch.get());
    
    inputs.turretRotationAppliedVolts = turretRotationAppliedVolts.getValueAsDouble();
    inputs.turretRotationAmpsStator = turretRotationCurrentAmpsStator.getValueAsDouble();
    inputs.turretRotationAmpsSupply = turretRotationCurrentAmpsSupply.getValueAsDouble();
    inputs.turretRotationTempCelcius = turretRotationTemp.getValueAsDouble();
    inputs.turretRotationPositionDeg = Units.rotationsToDegrees(turretRotationPosition.getValueAsDouble());
    inputs.turretRotation_pos_offset = turretRotation_pos_offset;
    inputs.turretRotationPositionRot = turretRotationController.getPosition().getValueAsDouble();
    //Use if using cancoder
    //inputs.turretRotationPositionRot = turretRotationEncoder.getPosition().getValueAsDouble() - turretRotation_pos_offset;
    inputs.turretRotationPositionRotREAL = turretRotationEncoder.getPosition().getValueAsDouble(); 
    inputs.turretRotationVelocityRadPerSec = Units.rotationsToRadians(turretRotationVelocity.getValueAsDouble());

    inputs.turretRotationAbsolutePositionDegrees =
        MathUtil.inputModulus(
            Rotation2d.fromRotations(turretRotationEncoder.getAbsolutePosition().getValueAsDouble())
                .minus(encoderOffset)
                .getDegrees(),
            -180,
            180);

  }


  //Use this to provide a speed based on voltage - it is not "controlling to speed"
  @Override
  public void setTurretRotationVolts(double volts) {
    turretRotationController.setControl(turretRotationVolts.withOutput(volts));
  }

  //Offset would be used when we need 
  @Override
  public void setTurretRotationVoltagePos(double position) {
    turretRotationController.setControl(voltPosition.withPosition(position + turretRotation_pos_offset));
  }


  @Override
  public void setTurretRotationVoltage(double voltage) {
    if (voltage > 0 && clockwiselimitswitch.get()){
      turretRotationController.setVoltage(voltage);
    }
    else if(voltage < 0 && counterclockwiselimitswitch.get()){
      turretRotationController.setVoltage(voltage);
    }
    else {
    
      turretRotationController.setVoltage(0);
    }
  }

  // only update the position of the turret vased on valid inputs from vision
  
  @Override
  public void  setTurretRotationError(double position, boolean isVisionLocked) {
   

    if(true){//isVisionLocked
      turretRotationController.setControl(voltPosition.withPosition(position));
    }
  } 




}
