// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.HoodControl;

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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;



/** Template hardware interface for a closed loop subsystem. */
public class HoodIOSpark implements HoodIO {

  private SparkMax hoodSparkMotor;
 

  
  private SparkClosedLoopController sparkPositionController;
  
  private RelativeEncoder positionEncoder;

  private final Double HoodAppliedVolts;

  private Double homeposition = 0.0; 

  DigitalInput hoodToplimitswitch;
  DigitalInput hoodBottomlimitswitch;
  

  public HoodIOSpark() {

 
    SparkMaxConfig hoodSparkConfigs = new SparkMaxConfig();
    

    hoodSparkMotor = new SparkMax(Hardwaremap.HoodSpark_CID, MotorType.kBrushless);
    
    

    //setup the encoders

    positionEncoder = hoodSparkMotor.getEncoder();
    
    hoodToplimitswitch = new DigitalInput(Hardwaremap.hoodToplimitswitch_DIO);
    hoodBottomlimitswitch = new DigitalInput(Hardwaremap.hoodBottomlimitswitch_DIO);


    //Position Control Gains
    hoodSparkConfigs.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    hoodSparkConfigs.closedLoop.p(.1); 
    hoodSparkConfigs.closedLoop.i(0); 
    hoodSparkConfigs.closedLoop.d(0); 


    // set max output current limits - 1
    hoodSparkConfigs.smartCurrentLimit(40);

    hoodSparkConfigs.inverted(false);
    hoodSparkConfigs.idleMode(IdleMode.kBrake);
    hoodSparkConfigs.encoder.positionConversionFactor(1);

    //Apply the configuration to the Spark Flex Controller
    hoodSparkMotor.configure(hoodSparkConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sparkPositionController = hoodSparkMotor.getClosedLoopController();

    
    HoodAppliedVolts = hoodSparkMotor.getAppliedOutput();


    //TODO add limit switches if needed



  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.HoodAppliedVolts = HoodAppliedVolts;
    inputs.hoodToplimitswitch = (!hoodToplimitswitch.get());
    inputs.hoodBottomlimitswitch = !hoodBottomlimitswitch.get();
    inputs.HoodPositionRot = positionEncoder.getPosition();
    homeposition = inputs.HoodPositionRot;
  }


  //Use this to provide a speed based on voltage - it is not "controlling to speed"
  @Override
  public void setHoodVolts(double volts) {
    hoodSparkMotor.setVoltage(volts);
  }

  //position is based on rotations so long as it is in bounds
  @Override
  public void setHoodVoltagePos(double position) {
      sparkPositionController.setSetpoint(position, ControlType.kPosition);
    
  }

    @Override
  public void setHoodVisionPos(double position) {
    if(position <= SuperStructureConstants.Hood_MaxPosition + homeposition || position >= SuperStructureConstants.Hood_ZERO_POS + homeposition){
      sparkPositionController.setSetpoint(position, ControlType.kPosition);
    }
  }

}
