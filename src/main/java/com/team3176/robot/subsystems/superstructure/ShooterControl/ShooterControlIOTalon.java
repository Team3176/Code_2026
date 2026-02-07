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
public class ShooterControlIOTalon implements ShooterControlIO {



  //Dual Motor Leader Follower Speed Control
  private TalonFX shooterLeaderSpeedController;
  private TalonFX shooterFollowerSpeedController;


 
  VelocityVoltage voltVelocity = new VelocityVoltage(0);
  VoltageOut genericTalonVolts = new VoltageOut(0.0);


  
  

  private final StatusSignal<Voltage> shooterAppliedVolts;
  private final StatusSignal<Current> shooterCurrentAmpsStator;
  private final StatusSignal<Current> shooterCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> shooterVelocity;
  private final StatusSignal<Angle> shooterPosition;

  private final StatusSignal<Temperature> shooterTemp;


  public ShooterControlIOTalon() {

 
    TalonFXConfiguration shooterDualSpeedConfigs = new TalonFXConfiguration();
 
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

  

    shooterLeaderSpeedController = new TalonFX(Hardwaremap.shooterLeaderSpeed_CID, Hardwaremap.shooterDualSpeed_CBN);
    shooterFollowerSpeedController = new TalonFX(Hardwaremap.shooterFollowerSpeed_CID, Hardwaremap.shooterDualSpeed_CBN);


     //Setup SPEED CONTROL FOR Dual Motor Leader / Follower - this definition assumes that the motors need to spin opposite directions for the mechanisim. 
    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    shooterDualSpeedConfigs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    shooterDualSpeedConfigs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    shooterDualSpeedConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    shooterDualSpeedConfigs.Slot0.kI = 0; // No output for integrated error
    shooterDualSpeedConfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    shooterDualSpeedConfigs.Voltage.withPeakForwardVoltage (SuperStructureConstants.ShooterDualSpeed_MAX_OUTPUT_VOLTS)
      .withPeakReverseVoltage(SuperStructureConstants.ShooterDualSpeed_MAXNeg_OUTPUT_VOLTS);

    TalonUtils.applyTalonFxConfigs(shooterLeaderSpeedController, shooterDualSpeedConfigs);
    TalonUtils.applyTalonFxConfigs(shooterFollowerSpeedController, shooterDualSpeedConfigs);
    shooterFollowerSpeedController.setControl(new Follower(shooterLeaderSpeedController.getDeviceID(), MotorAlignmentValue.Opposed)); 



    // Set variables for viewing. - TODO select the group that makes the most sense for your mechanism 
    shooterAppliedVolts = shooterLeaderSpeedController.getMotorVoltage();
    shooterCurrentAmpsStator = shooterLeaderSpeedController.getStatorCurrent();
    shooterCurrentAmpsSupply = shooterLeaderSpeedController.getSupplyCurrent();
    shooterVelocity = shooterLeaderSpeedController.getVelocity();
    shooterPosition = shooterLeaderSpeedController.getPosition();
    
    

    
    shooterTemp = shooterLeaderSpeedController.getDeviceTemp();



    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        shooterAppliedVolts,
        shooterCurrentAmpsStator,
        shooterVelocity,
        shooterPosition,
        shooterTemp,
        shooterCurrentAmpsSupply);

   

    shooterLeaderSpeedController.optimizeBusUtilization();
    shooterFollowerSpeedController.optimizeBusUtilization();


  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ShooterControlIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        shooterAppliedVolts,
        shooterCurrentAmpsStator,
        shooterVelocity,
        shooterPosition,
        shooterTemp,
        shooterCurrentAmpsSupply
        );
  }


  @Override
  public void setDualShooterSpeedBrakeMode(boolean enable) {
    if (enable) {
      shooterLeaderSpeedController.setNeutralMode(NeutralModeValue.Brake);
      shooterFollowerSpeedController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      shooterLeaderSpeedController.setNeutralMode(NeutralModeValue.Coast);
      shooterFollowerSpeedController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

    //Offset would be used when we need 
  @Override
  public void setDualShooterSpeedVelocity(double speed_RPS) {
    shooterLeaderSpeedController.setControl(voltVelocity.withVelocity(speed_RPS));
  }


}
