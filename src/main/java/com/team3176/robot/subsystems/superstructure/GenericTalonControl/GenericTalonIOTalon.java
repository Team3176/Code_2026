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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

/** Template hardware interface for a closed loop subsystem. */
public class GenericTalonIOTalon implements GenericTalonIO {

  private TalonFX genericTalonController;
  private CANcoder genericTalonEncoder;
  VelocityVoltage voltVelocity;
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
 
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    genericTalonController = new TalonFX(Hardwaremap.genericTalon_CID, Hardwaremap.genericTalon_CBN);

    genericTalonEncoder = new CANcoder(Hardwaremap.genericTalonCancoder_CID, Hardwaremap.genericTalon_CBN);
 

 
    //var genericTalonEncoderConfig = new CANcoderConfiguration();
    encoderOffset = Rotation2d.fromDegrees(SuperStructureConstants.GenericTalon_ENCODER_OFFSET);
    //genericTalonEncoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    

    //armPivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    genericTalonConfigs.Slot0.kP = 15; // An error of 1 rotation results in 2.4 V output
    genericTalonConfigs.Slot0.kI = 0.1; // No output for integrated error
    genericTalonConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    genericTalonConfigs.Voltage.PeakForwardVoltage = 16;
    genericTalonConfigs.Voltage.PeakReverseVoltage = -16;
    genericTalonConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    genericTalonConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.armCancoder_CID;
    genericTalonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    genericTalonConfigs.Feedback.SensorToMechanismRatio = 1.0;

    genericTalonConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    genericTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    genericTalonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    genericTalonConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        0.6;
    genericTalonConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    genericTalonConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        0.0;
    genericTalonConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

    TalonUtils.applyTalonFxConfigs(genericTalonController, genericTalonConfigs);
    //genericTalonController.setPosition(0, 0);

    genericTalonAppliedVolts = genericTalonController.getMotorVoltage();
    genericTalonCurrentAmpsStator = genericTalonController.getStatorCurrent();
    genericTalonCurrentAmpsSupply = genericTalonController.getSupplyCurrent();
    genericTalonVelocity = genericTalonController.getVelocity();
    genericTalonPosition = genericTalonEncoder.getPositionSinceBoot();
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

    // inputs.isRollerLinebreak = (!rollerLinebreak.get());
    // inputs.isPivotLinebreak = (!pivotLinebreak.get());

    inputs.genericTalonAppliedVolts = genericTalonAppliedVolts.getValueAsDouble();
    inputs.genericTalonAmpsStator = genericTalonCurrentAmpsStator.getValueAsDouble();
    inputs.genericTalonAmpsSupply = genericTalonCurrentAmpsSupply.getValueAsDouble();
    inputs.genericTalonTempCelcius = genericTalonTemp.getValueAsDouble();
    inputs.genericTalonPositionDeg = Units.rotationsToDegrees(genericTalonPosition.getValueAsDouble());
    inputs.genericTalon_pos_offset = genericTalon_pos_offset;
    inputs.genericTalonPositionRot = genericTalonEncoder.getPosition().getValueAsDouble() - genericTalon_pos_offset;
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



  @Override
  public void setGenericTalonVolts(double volts) {
    genericTalonController.setControl(genericTalonVolts.withOutput(volts));
  }

  @Override
  public void setGenericTalonVoltagePos(double position) {
    genericTalonController.setControl(voltPosition.withPosition(position + genericTalon_pos_offset));
  }

  @Override
  public void setGenericTalonCurrent(double current) {
    genericTalonController.setControl(genericTalonVolts.withOutput(current));
  }

  @Override
  public void setGenericTalonBrakeMode(boolean enable) {
    if (enable) {
      genericTalonController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      genericTalonController.setNeutralMode(NeutralModeValue.Coast);
    }
  }
}
