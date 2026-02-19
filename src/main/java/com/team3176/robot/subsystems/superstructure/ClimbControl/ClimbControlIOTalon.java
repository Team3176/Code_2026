// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.ClimbControl;

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
public class ClimbControlIOTalon implements ClimbControlIO {

  private TalonFX ClimbLeftController;
  private TalonFX ClimbRightController;
  
  PositionVoltage voltPosition = new PositionVoltage(0);

  private final StatusSignal<Voltage> ClimbAppliedVolts;
  private final StatusSignal<Current> ClimbCurrentAmpsStator;
  private final StatusSignal<Current> ClimbCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> ClimbVelocity;
  private final StatusSignal<Angle> ClimbPosition;
  private final StatusSignal<Angle> ClimbAbsolutePosition;
  private final StatusSignal<Temperature> ClimbTemp;


  public ClimbControlIOTalon() {

 
    TalonFXConfiguration ClimbLeftConfigs = new TalonFXConfiguration();
    TalonFXConfiguration ClimbRightConfigs = new TalonFXConfiguration();
    
    ClimbLeftController = new TalonFX(Hardwaremap.ClimbLeft_CID, Hardwaremap.Climb_CBN);
    ClimbRightController = new TalonFX(Hardwaremap.ClimbRight_CID, Hardwaremap.Climb_CBN);

    ClimbLeftConfigs.Slot0.kP = 3; // An error of 1 rotation results in 2.4 V output
    ClimbLeftConfigs.Slot0.kI = 0.1; // No output for integrated error
    ClimbLeftConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // set max output voltage limits speed - 14V is max output available 
    ClimbLeftConfigs.Voltage.PeakForwardVoltage = SuperStructureConstants.Climb_MAX_OUTPUT_VOLTS; 
    ClimbLeftConfigs.Voltage.PeakReverseVoltage = SuperStructureConstants.Climb_MAXNeg_OUTPUT_VOLTS;

    ClimbLeftConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    ClimbLeftConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    ClimbLeftConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    ClimbLeftConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    ClimbLeftConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.6;
    ClimbLeftConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    ClimbLeftConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    ClimbLeftConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 



    ClimbRightConfigs.Slot0.kP = 3; // An error of 1 rotation results in 2.4 V output
    ClimbRightConfigs.Slot0.kI = 0.1; // No output for integrated error
    ClimbRightConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // set max output voltage limits speed - 14V is max output available 
    ClimbRightConfigs.Voltage.PeakForwardVoltage = SuperStructureConstants.Climb_MAX_OUTPUT_VOLTS; 
    ClimbRightConfigs.Voltage.PeakReverseVoltage = SuperStructureConstants.Climb_MAXNeg_OUTPUT_VOLTS;

    ClimbRightConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    ClimbRightConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    ClimbRightConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    ClimbRightConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    ClimbRightConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.6;
    ClimbRightConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    ClimbRightConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    ClimbRightConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

    TalonUtils.applyTalonFxConfigs(ClimbLeftController, ClimbLeftConfigs);

    TalonUtils.applyTalonFxConfigs(ClimbRightController, ClimbRightConfigs);


    


    ClimbAppliedVolts = ClimbLeftController.getMotorVoltage();
    ClimbCurrentAmpsStator = ClimbLeftController.getStatorCurrent();
    ClimbCurrentAmpsSupply = ClimbLeftController.getSupplyCurrent();
    ClimbVelocity = ClimbLeftController.getVelocity();
    ClimbPosition = ClimbLeftController.getPosition();
    ClimbTemp = ClimbLeftController.getDeviceTemp();
    ClimbAbsolutePosition = ClimbLeftController.getPosition();


    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        ClimbAppliedVolts,
        ClimbCurrentAmpsStator,
        ClimbVelocity,
        ClimbPosition,
        ClimbTemp,
        ClimbCurrentAmpsSupply);

    ClimbLeftController.optimizeBusUtilization();
    
  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ClimbControlIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        ClimbAppliedVolts,
        ClimbCurrentAmpsStator,
        ClimbVelocity,
        ClimbPosition,
        ClimbTemp,
        ClimbCurrentAmpsSupply
        );


    inputs.ClimbAppliedVolts = ClimbAppliedVolts.getValueAsDouble();
    inputs.ClimbAmpsStator = ClimbCurrentAmpsStator.getValueAsDouble();
    inputs.ClimbAmpsSupply = ClimbCurrentAmpsSupply.getValueAsDouble();
    inputs.ClimbTempCelcius = ClimbTemp.getValueAsDouble();
    inputs.ClimbPositionDeg = Units.rotationsToDegrees(ClimbPosition.getValueAsDouble());
    //inputs.Climb_pos_offset = Climb_pos_offset;
    inputs.ClimbPositionRot = ClimbLeftController.getPosition().getValueAsDouble();
    //Use if using cancoder
    //inputs.ClimbPositionRot = ClimbEncoder.getPosition().getValueAsDouble() - Climb_pos_offset;
    inputs.ClimbPositionRotREAL = ClimbLeftController.getPosition().getValueAsDouble(); 
    inputs.ClimbVelocityRadPerSec = Units.rotationsToRadians(ClimbVelocity.getValueAsDouble());

    inputs.ClimbAbsolutePositionDegrees =
        MathUtil.inputModulus(
            Rotation2d.fromRotations(ClimbLeftController.getPosition().getValueAsDouble())
                .getDegrees(),
            -180,
            180);

  }


  
  //Offset would be used when we need 
  @Override
  public void setClimbBothPos(double position) {
    ClimbLeftController.setControl(voltPosition.withPosition(position * SuperStructureConstants.Climb_Position_MULTIPLIER + SuperStructureConstants.ClimbLeft_pos_offset));
    ClimbRightController.setControl(voltPosition.withPosition(position * SuperStructureConstants.Climb_Position_MULTIPLIER + SuperStructureConstants.ClimbRight_pos_offset));
  } 

    @Override
  public void setClimbLeftPos(double position) {
    ClimbLeftController.setControl(voltPosition.withPosition(position * SuperStructureConstants.Climb_Position_MULTIPLIER + SuperStructureConstants.ClimbLeft_pos_offset));
   } 

    @Override
  public void setClimbRightPos(double position) {
    ClimbRightController.setControl(voltPosition.withPosition(position * SuperStructureConstants.Climb_Position_MULTIPLIER + SuperStructureConstants.ClimbRight_pos_offset));
  } 
 
}
