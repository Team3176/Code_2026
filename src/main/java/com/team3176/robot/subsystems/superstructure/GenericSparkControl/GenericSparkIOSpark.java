// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.GenericSparkControl;

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
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;


/** Template hardware interface for a closed loop subsystem. */
public class GenericSparkIOSpark implements GenericSparkIO {

  private SparkFlex genericSparkFlexController;
  private SparkFlex genericSparkFlexSpeedController;

  //Dual Motor Leader Follower Speed Control
  private SparkFlex genericSparkFlexLeaderSpeedController;
  private SparkFlex genericSparkFlexFollowerSpeedController;

  private SparkClosedLoopController sparkPositionController;
  private SparkClosedLoopController sparkSpeedController;
  private SparkClosedLoopController sparkDualSpeedController;

  private RelativeEncoder positionEncoder;
  private RelativeEncoder speedEncoder; 
  private RelativeEncoder dualSpeedEncoder; 

  public GenericSparkIOSpark() {

 
    SparkFlexConfig genericSparkFlexConfigs = new SparkFlexConfig();
    SparkFlexConfig genericSparkFlexSpeedConfigs = new SparkFlexConfig();
    
    SparkFlexConfig genericSparkFlexDualSpeedConfigs = new SparkFlexConfig();
    SparkFlexConfig genericSparkFlexDualFollowerSpeedConfigs = new SparkFlexConfig();
 
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    genericSparkFlexController = new SparkFlex(Hardwaremap.genericSparkFlex_CID, MotorType.kBrushless);
    genericSparkFlexSpeedController = new SparkFlex(Hardwaremap.genericSparkFlexSpeed_CID, MotorType.kBrushless);

    genericSparkFlexLeaderSpeedController = new SparkFlex(Hardwaremap.genericSparkFlexLeaderSpeed_CID, MotorType.kBrushless);
    genericSparkFlexFollowerSpeedController = new SparkFlex(Hardwaremap.genericSparkFlexFollowerSpeed_CID, MotorType.kBrushless);

    //setup the encoders

    positionEncoder = genericSparkFlexController.getEncoder();
    speedEncoder = genericSparkFlexSpeedController.getEncoder();
    dualSpeedEncoder = genericSparkFlexLeaderSpeedController.getEncoder();



    //Position Control Gains
    genericSparkFlexConfigs.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    genericSparkFlexConfigs.closedLoop.p(1); 
    genericSparkFlexConfigs.closedLoop.i(0); 
    genericSparkFlexConfigs.closedLoop.d(0); 


    // set max output current limits - 1
    genericSparkFlexConfigs.smartCurrentLimit(60);

    genericSparkFlexConfigs.inverted(false);
    genericSparkFlexConfigs.idleMode(IdleMode.kBrake);
    genericSparkFlexConfigs.encoder.positionConversionFactor(1);

    //Apply the configuration to the Spark Flex Controller
    genericSparkFlexController.configure(genericSparkFlexConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sparkPositionController = genericSparkFlexController.getClosedLoopController();

    



    //TODO add limit switches if needed


    //SETUP SPEED CONTROL CONFIGS
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    genericSparkFlexSpeedConfigs.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    genericSparkFlexSpeedConfigs.closedLoop.p(0.003); 
    genericSparkFlexSpeedConfigs.closedLoop.i(0.002);
    genericSparkFlexSpeedConfigs.closedLoop.d(0.001); 
  
    // set max output current limits TODO check stall current of speed / roller
    genericSparkFlexSpeedConfigs.smartCurrentLimit(60);

    genericSparkFlexSpeedConfigs.inverted(false);
    genericSparkFlexSpeedConfigs.idleMode(IdleMode.kCoast);
    genericSparkFlexSpeedConfigs.encoder.velocityConversionFactor(1);

    //Apply the configuration to the Spark Flex Controller
    genericSparkFlexSpeedController.configure(genericSparkFlexSpeedConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sparkSpeedController = genericSparkFlexSpeedController.getClosedLoopController();

    //
    //SETUP  DualSPEED CONTROL CONFIGS
    //
    genericSparkFlexDualSpeedConfigs.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    genericSparkFlexDualSpeedConfigs.closedLoop.p(0.001); 
    genericSparkFlexDualSpeedConfigs.closedLoop.i(0);
    genericSparkFlexDualSpeedConfigs.closedLoop.d(0); 
  
    // set max output current limits TODO check stall current of speed / roller
    genericSparkFlexDualSpeedConfigs.smartCurrentLimit(60);

    genericSparkFlexDualSpeedConfigs.inverted(false);
    genericSparkFlexDualSpeedConfigs.idleMode(IdleMode.kCoast);
    genericSparkFlexDualSpeedConfigs.encoder.velocityConversionFactor(1);

    genericSparkFlexDualFollowerSpeedConfigs.closedLoop.p(0.001); 
    genericSparkFlexDualFollowerSpeedConfigs.closedLoop.i(0);
    genericSparkFlexDualFollowerSpeedConfigs.closedLoop.d(0); 
  
    // set max output current limits TODO check stall current of speed / roller
    genericSparkFlexDualFollowerSpeedConfigs.smartCurrentLimit(60);
    genericSparkFlexDualFollowerSpeedConfigs.inverted(false);
    genericSparkFlexDualFollowerSpeedConfigs.idleMode(IdleMode.kCoast);
    genericSparkFlexDualFollowerSpeedConfigs.follow(genericSparkFlexLeaderSpeedController, true);

    //Apply the configuration to the Spark Flex Controller
    genericSparkFlexLeaderSpeedController.configure(genericSparkFlexDualSpeedConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    genericSparkFlexFollowerSpeedController.configure(genericSparkFlexDualFollowerSpeedConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sparkDualSpeedController = genericSparkFlexLeaderSpeedController.getClosedLoopController();
  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(GenericSparkIOInputs inputs) {
   

  }


  //Use this to provide a speed based on voltage - it is not "controlling to speed"
  @Override
  public void setGenericSparkVolts(double volts) {
    genericSparkFlexController.setVoltage(volts * SuperStructureConstants.GenericSpark_MAX_OUTPUT_VOLTS);
  }

  //position is based on rotations
  @Override
  public void setGenericSparkPosition(double position) {
    sparkPositionController.setSetpoint(position, ControlType.kPosition);
  }




    //Offset would be used when we need 
  @Override
  public void setGenericSparkSpeedVelocity(double speed_RPM) {
    sparkSpeedController.setSetpoint(speed_RPM, ControlType.kVelocity);
  }



  

  @Override
  public void setGenericSparkDualSpeedVelocity(double speed_RPM) {
    sparkDualSpeedController.setSetpoint(speed_RPM, ControlType.kVelocity);
  }


}
