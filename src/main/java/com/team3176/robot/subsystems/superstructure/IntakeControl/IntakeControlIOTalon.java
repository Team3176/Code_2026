// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.IntakeControl;

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
public class IntakeControlIOTalon implements IntakeControlIO {

  private TalonFX IntakePositionController;
  
  private CANcoder IntakePositionEncoder;
  VelocityVoltage voltVelocity = new VelocityVoltage(0);
  VoltageOut IntakePositionVolts = new VoltageOut(0.0);
  PositionVoltage voltIntakePosition = new PositionVoltage(0);
  private Rotation2d encoderOffset; 
  
  DigitalInput HoodLinebreak;

  private final StatusSignal<Voltage> IntakePositionAppliedVolts;
  private final StatusSignal<Current> IntakePositionCurrentAmpsStator;
  private final StatusSignal<Current> IntakePositionCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> IntakePositionVelocity;
  private final StatusSignal<Angle> IntakePositionPosition;
  private final StatusSignal<Angle> IntakePositionAbsolutePosition;
  private final StatusSignal<Temperature> IntakePositionTemp;


  public IntakeControlIOTalon() {

 
    TalonFXConfiguration IntakePositionConfigs = new TalonFXConfiguration();
    
 
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    IntakePositionController = new TalonFX(Hardwaremap.IntakePosition_CID, Hardwaremap.Intake_CBN);
    
    IntakePositionEncoder = new CANcoder(Hardwaremap.IntakePositionCancoder_CID, Hardwaremap.Intake_CBN);
 

 
    //var HoodEncoderConfig = new CANcoderConfiguration();
    encoderOffset = Rotation2d.fromDegrees(SuperStructureConstants.IntakePosition_ENCODER_OFFSET);
    //HoodEncoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    

    //armPivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    IntakePositionConfigs.Slot0.kP = 3; // An error of 1 rotation results in 2.4 V output
    IntakePositionConfigs.Slot0.kI = 0.1; // No output for integrated error
    IntakePositionConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // set max output voltage limits speed - 14V is max output available 
    IntakePositionConfigs.Voltage.PeakForwardVoltage = SuperStructureConstants.IntakePosition_MAX_OUTPUT_VOLTS; 
    IntakePositionConfigs.Voltage.PeakReverseVoltage = SuperStructureConstants.IntakePosition_MAXNeg_OUTPUT_VOLTS;

    IntakePositionConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //TODO if position from Cancoder define which CanCoder / remote sensor to use for position feedback
    //HoodConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.HoodCancoder_CID;
    //HoodConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    //HoodConfigs.Feedback.SensorToMechanismRatio = 1.0;

    IntakePositionConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    IntakePositionConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    IntakePositionConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    IntakePositionConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.6;
    IntakePositionConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    IntakePositionConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    IntakePositionConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

    TalonUtils.applyTalonFxConfigs(IntakePositionController, IntakePositionConfigs);
    //IntakePositionController.setPosition(0, 0);

    


    IntakePositionAppliedVolts = IntakePositionController.getMotorVoltage();
    IntakePositionCurrentAmpsStator = IntakePositionController.getStatorCurrent();
    IntakePositionCurrentAmpsSupply = IntakePositionController.getSupplyCurrent();
    IntakePositionVelocity = IntakePositionController.getVelocity();
    IntakePositionPosition = IntakePositionController.getPosition();
    
    //If you want to use a cancode use this definition 
    //HoodPosition = HoodEncoder.getPositionSinceBoot();
    IntakePositionAbsolutePosition = IntakePositionController.getPosition();
    IntakePositionTemp = IntakePositionController.getDeviceTemp();

   // Hood_pos_offset = HoodEncoder.getPosition().getValueAsDouble();


    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        IntakePositionAppliedVolts,
        IntakePositionCurrentAmpsStator,
        IntakePositionVelocity,
        IntakePositionPosition,
        IntakePositionTemp,
        IntakePositionCurrentAmpsSupply);

    IntakePositionController.optimizeBusUtilization();
    
  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeControlIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        IntakePositionAppliedVolts,
        IntakePositionCurrentAmpsStator,
        IntakePositionVelocity,
        IntakePositionPosition,
        IntakePositionTemp,
        IntakePositionCurrentAmpsSupply
        );


    inputs.IntakeAppliedVolts = IntakePositionAppliedVolts.getValueAsDouble();
    inputs.IntakeAmpsStator = IntakePositionCurrentAmpsStator.getValueAsDouble();
    inputs.IntakeAmpsSupply = IntakePositionCurrentAmpsSupply.getValueAsDouble();
    inputs.IntakeTempCelcius = IntakePositionTemp.getValueAsDouble();
    inputs.IntakePositionDeg = Units.rotationsToDegrees(IntakePositionPosition.getValueAsDouble());
    //inputs.Hood_pos_offset = Hood_pos_offset;
    inputs.IntakePositionRot = IntakePositionController.getPosition().getValueAsDouble();
    //Use if using cancoder
    //inputs.HoodPositionRot = HoodEncoder.getPosition().getValueAsDouble() - Hood_pos_offset;
    inputs.IntakePositionRotREAL = IntakePositionEncoder.getPosition().getValueAsDouble(); 
    inputs.IntakeVelocityRadPerSec = Units.rotationsToRadians(IntakePositionVelocity.getValueAsDouble());

    inputs.IntakeAbsolutePositionDegrees =
        MathUtil.inputModulus(
            Rotation2d.fromRotations(IntakePositionEncoder.getAbsolutePosition().getValueAsDouble())
                .minus(encoderOffset)
                .getDegrees(),
            -180,
            180);

  }


  
  //Offset would be used when we need 
  @Override
  public void setIntakePositionVoltagePos(double position) {
    IntakePositionController.setControl(voltIntakePosition.withPosition(position * SuperStructureConstants.Intake_Position_MULTIPLIER + SuperStructureConstants.Intake_pos_offset));
  }


  @Override
  public void setIntakePositionBrakeMode(boolean enable) {
    if (enable) {
      IntakePositionController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      IntakePositionController.setNeutralMode(NeutralModeValue.Coast);
    }
  }


    //Offset would be used when we need 
 
}
