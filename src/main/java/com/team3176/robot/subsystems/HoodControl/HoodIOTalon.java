// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.HoodControl;

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
public class HoodIOTalon implements HoodIO {

  private TalonFX HoodController;
  
  private CANcoder HoodEncoder;
  VelocityVoltage voltVelocity = new VelocityVoltage(0);
  VoltageOut HoodVolts = new VoltageOut(0.0);
  PositionVoltage voltPosition = new PositionVoltage(0);
  private Rotation2d encoderOffset; 
  private double Hood_pos_offset = 0;
  
  DigitalInput HoodLinebreak;

  private final StatusSignal<Voltage> HoodAppliedVolts;
  private final StatusSignal<Current> HoodCurrentAmpsStator;
  private final StatusSignal<Current> HoodCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> HoodVelocity;
  private final StatusSignal<Angle> HoodPosition;
  private final StatusSignal<Angle> HoodAbsolutePosition;
  private final StatusSignal<Temperature> HoodTemp;


  public HoodIOTalon() {

 
    TalonFXConfiguration HoodConfigs = new TalonFXConfiguration();
    
 
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    HoodController = new TalonFX(Hardwaremap.Hood_CID, Hardwaremap.Hood_CBN);
    
    HoodEncoder = new CANcoder(Hardwaremap.HoodCancoder_CID, Hardwaremap.Hood_CBN);
 

 
    //var HoodEncoderConfig = new CANcoderConfiguration();
    encoderOffset = Rotation2d.fromDegrees(SuperStructureConstants.Hood_ENCODER_OFFSET);
    //HoodEncoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    

    //armPivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    HoodConfigs.Slot0.kP = 3; // An error of 1 rotation results in 2.4 V output
    HoodConfigs.Slot0.kI = 0.1; // No output for integrated error
    HoodConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // set max output voltage limits speed - 14V is max output available 
    HoodConfigs.Voltage.PeakForwardVoltage = SuperStructureConstants.Hood_MAX_OUTPUT_VOLTS; 
    HoodConfigs.Voltage.PeakReverseVoltage = SuperStructureConstants.Hood_MAXNeg_OUTPUT_VOLTS;

    HoodConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //TODO if position from Cancoder define which CanCoder / remote sensor to use for position feedback
    //HoodConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.HoodCancoder_CID;
    //HoodConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    //HoodConfigs.Feedback.SensorToMechanismRatio = 1.0;

    HoodConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    HoodConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    HoodConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    HoodConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.6;
    HoodConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    HoodConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    HoodConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

    TalonUtils.applyTalonFxConfigs(HoodController, HoodConfigs);
    //HoodController.setPosition(0, 0);

    


    HoodAppliedVolts = HoodController.getMotorVoltage();
    HoodCurrentAmpsStator = HoodController.getStatorCurrent();
    HoodCurrentAmpsSupply = HoodController.getSupplyCurrent();
    HoodVelocity = HoodController.getVelocity();
    HoodPosition = HoodController.getPosition();
    
    //If you want to use a cancode use this definition 
    //HoodPosition = HoodEncoder.getPositionSinceBoot();
    HoodAbsolutePosition = HoodEncoder.getAbsolutePosition();
    HoodTemp = HoodController.getDeviceTemp();

    Hood_pos_offset = HoodEncoder.getPosition().getValueAsDouble();


    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        HoodAppliedVolts,
        HoodCurrentAmpsStator,
        HoodVelocity,
        HoodPosition,
        HoodTemp,
        HoodCurrentAmpsSupply);

    HoodController.optimizeBusUtilization();
    
  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        HoodAppliedVolts,
        HoodCurrentAmpsStator,
        HoodVelocity,
        HoodPosition,
        HoodTemp,
        HoodCurrentAmpsSupply
        );


    inputs.HoodAppliedVolts = HoodAppliedVolts.getValueAsDouble();
    inputs.HoodAmpsStator = HoodCurrentAmpsStator.getValueAsDouble();
    inputs.HoodAmpsSupply = HoodCurrentAmpsSupply.getValueAsDouble();
    inputs.HoodTempCelcius = HoodTemp.getValueAsDouble();
    inputs.HoodPositionDeg = Units.rotationsToDegrees(HoodPosition.getValueAsDouble());
    inputs.Hood_pos_offset = Hood_pos_offset;
    inputs.HoodPositionRot = HoodController.getPosition().getValueAsDouble();
    //Use if using cancoder
    //inputs.HoodPositionRot = HoodEncoder.getPosition().getValueAsDouble() - Hood_pos_offset;
    inputs.HoodPositionRotREAL = HoodEncoder.getPosition().getValueAsDouble(); 
    inputs.HoodVelocityRadPerSec = Units.rotationsToRadians(HoodVelocity.getValueAsDouble());

    inputs.HoodAbsolutePositionDegrees =
        MathUtil.inputModulus(
            Rotation2d.fromRotations(HoodEncoder.getAbsolutePosition().getValueAsDouble())
                .minus(encoderOffset)
                .getDegrees(),
            -180,
            180);

  }


  
  //Offset would be used when we need 
  @Override
  public void setHoodVoltagePos(double position) {
    HoodController.setControl(voltPosition.withPosition(position + Hood_pos_offset));
  }


  @Override
  public void setHoodBrakeMode(boolean enable) {
    if (enable) {
      HoodController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      HoodController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  

    //Offset would be used when we need 
 
}
