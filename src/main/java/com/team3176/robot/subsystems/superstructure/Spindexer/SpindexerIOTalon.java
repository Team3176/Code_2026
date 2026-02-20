// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.Spindexer;

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
public class SpindexerIOTalon implements SpindexerIO {

  private TalonFX SpindexerController;
  private TalonFX SpindexerSpeedController;
  private CANcoder SpindexerEncoder;
  VelocityVoltage voltVelocity = new VelocityVoltage(0);
  VoltageOut SpindexerVolts = new VoltageOut(0.0);
  PositionVoltage voltPosition = new PositionVoltage(0);
  private Rotation2d encoderOffset; 
  private double Spindexer_pos_offset = 0;
  
  DigitalInput SpindexerLinebreak;

  private final StatusSignal<Voltage> SpindexerAppliedVolts;
  private final StatusSignal<Current> SpindexerCurrentAmpsStator;
  private final StatusSignal<Current> SpindexerCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> SpindexerVelocity;
  //private final StatusSignal<Angle> SpindexerPosition;
  private final StatusSignal<Angle> SpindexerAbsolutePosition;
  private final StatusSignal<Temperature> SpindexerTemp;


  public SpindexerIOTalon() {

 
    
    TalonFXConfiguration SpindexerSpeedConfigs = new TalonFXConfiguration();
 
    SpindexerSpeedController = new TalonFX(Hardwaremap.Spindexer_CID, Hardwaremap.Spindexer_CBN);

   
    //SETUP SPEED CONTROL CONFIGS
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    SpindexerSpeedConfigs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    SpindexerSpeedConfigs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    SpindexerSpeedConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    SpindexerSpeedConfigs.Slot0.kI = 0; // No output for integrated error
    SpindexerSpeedConfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    SpindexerSpeedConfigs.Voltage.withPeakForwardVoltage (SuperStructureConstants.SpindexerSpeed_MAX_OUTPUT_VOLTS)
      .withPeakReverseVoltage(SuperStructureConstants.SpindexerSpeed_MAXNeg_OUTPUT_VOLTS);

    TalonUtils.applyTalonFxConfigs(SpindexerSpeedController, SpindexerSpeedConfigs);


    SpindexerAppliedVolts = SpindexerSpeedController.getMotorVoltage();
    SpindexerCurrentAmpsStator = SpindexerSpeedController.getStatorCurrent();
    SpindexerCurrentAmpsSupply = SpindexerSpeedController.getSupplyCurrent();
    SpindexerVelocity = SpindexerSpeedController.getVelocity();
    //SpindexerPosition = SpindexerSpeedController.getPosition();
    
    //If you want to use a cancode use this definition 
    //SpindexerPosition = SpindexerEncoder.getPositionSinceBoot();
    SpindexerAbsolutePosition = SpindexerSpeedController.getPosition();
    SpindexerTemp = SpindexerSpeedController.getDeviceTemp();

    Spindexer_pos_offset = SpindexerSpeedController.getPosition().getValueAsDouble();


    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        SpindexerAppliedVolts,
        SpindexerCurrentAmpsStator,
        SpindexerVelocity,
       // SpindexerPosition,
        SpindexerTemp,
        SpindexerCurrentAmpsSupply);

    
    SpindexerSpeedController.optimizeBusUtilization();
  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        SpindexerAppliedVolts,
        SpindexerCurrentAmpsStator,
        SpindexerVelocity,
      //  SpindexerPosition,
        SpindexerTemp,
        SpindexerCurrentAmpsSupply
        );


    inputs.SpindexerAppliedVolts = SpindexerAppliedVolts.getValueAsDouble();
    inputs.SpindexerAmpsStator = SpindexerCurrentAmpsStator.getValueAsDouble();
    inputs.SpindexerAmpsSupply = SpindexerCurrentAmpsSupply.getValueAsDouble();
    inputs.SpindexerTempCelcius = SpindexerTemp.getValueAsDouble();
    //inputs.SpindexerPositionDeg = Units.rotationsToDegrees(SpindexerPosition.getValueAsDouble());
    inputs.Spindexer_pos_offset = Spindexer_pos_offset;
    inputs.SpindexerPositionRot = SpindexerSpeedController.getPosition().getValueAsDouble();
    //Use if using cancoder
    //inputs.SpindexerPositionRot = SpindexerEncoder.getPosition().getValueAsDouble() - Spindexer_pos_offset;
    inputs.SpindexerPositionRotREAL = SpindexerSpeedController.getPosition().getValueAsDouble(); 
    inputs.SpindexerVelocityRadPerSec = Units.rotationsToRadians(SpindexerVelocity.getValueAsDouble());
    inputs.SpindexerVelocity = (SpindexerVelocity.getValueAsDouble());
   

  }


  //Use this to provide a speed based on voltage - it is not "controlling to speed"
  @Override
  public void setSpindexerVolts(double volts) {
    SpindexerController.setControl(SpindexerVolts.withOutput(volts));
  }

  
  //Offset would be used when we need 
  @Override
  public void setSpindexerSpeedVelocity(double speed_RPS) {
    SpindexerSpeedController.setControl(voltVelocity.withVelocity(speed_RPS));
  }

}
