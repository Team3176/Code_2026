package com.team3176.robot.constants;

public class SuperStructureConstants {


  public static final double GenericTalon_MAX_OUTPUT_VOLTS = 2 ; //Sets Max forward Speed
  public static final double GenericTalon_MAXNeg_OUTPUT_VOLTS = -2; //Sets Max Reverse Speed
  public static final double GenericTalon_kP = .5;
  public static final double GenericTalon_kI = 0; 
  public static final double GenericTalon_kD = 0; 
  //Set if using gravity for feedforward
  public static final double GenericTalon_kg = 0.2;
  public static final double GenericTalon_ZERO_POS = 0.39;
  public static final double GenericTalon_ENCODER_OFFSET = 0;



  public static final double GenericTalonSpeed_MAX_OUTPUT_VOLTS = 4; //Sets Max forward Speed
  public static final double GenericTalonSpeed_MAXNeg_OUTPUT_VOLTS = -4; //Sets Max Reverse Speed
  public static final double GenericTalonSpeed_Max_RPS = 20;




  public static final double GenericTalonDualSpeed_MAX_OUTPUT_VOLTS = 4; //Sets Max forward Speed
  public static final double GenericTalonDualSpeed_MAXNeg_OUTPUT_VOLTS = -4; //Sets Max Reverse Speed
  public static final double GenericTalonDualSpeed_Max_RPS = 20;




  //
  // Constants used by the Generic Spark Motor Controller Subsystems. 
  //
  public static final double GenericSpark_MAX_OUTPUT_VOLTS = 4 ; //Sets Max forward Speed
  public static final double GenericSpark_MAXNeg_OUTPUT_VOLTS = -4; //Sets Max Reverse Speed

  public static final double GenericSpark_kP = .5;
  public static final double GenericSpark_kI = 0; 
  public static final double GenericSpark_kD = 0; 

  //Set if using gravity for feedforward
  public static final double GenericSpark_kg = 0.2;

  public static final double GenericSpark_ZERO_POS = 0.39;

  public static final double GenericSpark_ENCODER_OFFSET = 0;


  public static final double GenericSparkSpeed_MAX_OUTPUT_VOLTS = 4; //Sets Max forward Speed
  public static final double GenericSparkSpeed_MAXNeg_OUTPUT_VOLTS = -4; //Sets Max Reverse Speed
  public static final double GenericSparkSpeed_Max_RPM = 10;


  public static final double GenericSparkDualSpeed_MAX_OUTPUT_VOLTS = 4; //Sets Max forward Speed
  public static final double GenericSparkDualSpeed_MAXNeg_OUTPUT_VOLTS = -4; //Sets Max Reverse Speed
  public static final double GenericSparkDualSpeed_Max_RPM = 20;

  public static final double Kicker_MAX_OUTPUT_VOLTS = 3; //Sets Max forward Speed
  public static final double Kicker_MAXNeg_OUTPUT_VOLTS = -3; //Sets Max Reverse Speed
  public static final double Kicker_kP = .5;
  public static final double Kicker_kI = 0; 
  public static final double Kicker_kD = 0; 
  public static final double Kicker_kg = 0.2;
  public static final double Kicker_ZERO_POS = 0.39;  
  public static final double Kicker_ENCODER_OFFSET = 0;
  public static final double Kicker_Speed_On = 20;
  public static final double Kicker_Speed_Off = 0;


  public static final double Hood_ZERO_POS = 0.39;
  public static final double Hood_MaxPosition = 12; // ten tooth pinion 23 tooth rack, 1 to 1 gear ratio, input is from -1 to 1
  
  public static final double Hood_ENCODER_OFFSET = 0;

  public static final double Hood_Position_MULTIPLIER = 6; // ten tooth pinion 23 tooth rack, 1 to 1 gear ratio, input is from -1 to 1
  
  public static final double Hood_pos_offset = 0; //changed from 1 to 0

  public static final double HoodErrorMoveHood = 0;


  public static final double Hood_MAX_OUTPUT_VOLTS = 2 ; //Sets Max forward Speed
  public static final double Hood_MAXNeg_OUTPUT_VOLTS = -2; //Sets Max Reverse Speed
  public static final double Hood_kP = .5;
  public static final double Hood_kI = 0; 
  public static final double Hood_kD = 0; 
  public static final double HoodUpIncrement = 0.3; 
  public static final double HoodDownIncrement = 0.3; 

  //Set if using gravity for feedforward
  public static final double Hood_kg = 2.0;


  // Intake Control Constants 
  public static final double IntakeRollerMaxDutyCycle = 0.2; // This is 20% max duty cycle 
  public static final double Intake_ZERO_POS = 0.39;
  public static final double IntakePosition_ENCODER_OFFSET = 0;
  public static final double Intake_Position_MULTIPLIER = 5.39; // 12:1 gear box at just under 1/3 of an out put rotation
  public static final double Intake_pos_offset = 0;
  public static final double IntakePosition_MAX_OUTPUT_VOLTS = 12 ; //Sets Max forward Speed
  public static final double IntakePosition_MAXNeg_OUTPUT_VOLTS = -12; //Sets Max Reverse Speed
  public static final double IntakeControl_kP = .5;
  public static final double IntakeControl_kI = 0; 
  public static final double IntakeControl_kD = 0; 

  //Set if using gravity for feedforward
  public static final double Intake_kg = 0.2;


  public static final double KickerSpeed_MAX_OUTPUT_VOLTS = 3; //Sets Max forward Speed
  public static final double KickerSpeed_MAXNeg_OUTPUT_VOLTS = -3; //Sets Max Reverse Speed
  public static final double KickerSpeed_Max_RPS = 30;

///
// Shooter Constants
///

  public static final double ShooterDualSpeed_MAX_OUTPUT_VOLTS = 12 ; //Sets Max forward Speed
  public static final double ShooterDualSpeed_MAXNeg_OUTPUT_VOLTS = -12; //Sets Max Reverse Speed
  public static final double ShooterDualSpeed_Max_RPS = 100; // speed is multiplied by two due to joystick so currently this equals 50 * 2 = 100
  public static final double runDualShooterSpeedIDLE_SPEED = 6;
  
///
/// Turret Constants
///
  public static final double TurretRotation_MAX_OUTPUT_VOLTS = 2 ; //Sets Max forward Speed
  public static final double TurretRotation_MAXNeg_OUTPUT_VOLTS = -2; //Sets Max Reverse Speed
  public static final double TurretRotation_kP = .5;
  public static final double TurretRotation_kI = 0; 
  public static final double TurretRotation_kD = 0; 
  //Set if using gravity for feedforward
  public static final double TurretRotation_kg = 0.2;
  public static final double TurretRotation_ZERO_POS = 0.39;
  public static final double TurretRotation_ENCODER_OFFSET = 0;


/// One Rotation (360 Degrees of Turret)
///  72 Tooth Gear on Turrent
///  12 Tooth Gear Drive of Turret
///  5:1 Gear Ratio on Motor Stage 1
///  1:1 Gear Ratio on Motor Stage 2 *** there is not a second stage at the moment
/// 
/// This ratio gives 1 motor rotation is 12 degrees of motor turret movement. 

  public static final double TurretGearTeeth = 72; //number of teeth on turret gear
  public static final double TurretDriveTeeth = 12; // number of teeth on turret drive gear (attached to motor)
  public static final double MotorStage1Ratio = 5;
  public static final double MotorStage2Ratio = 1;

  public static final double TurretPositionFullRotation = ((TurretGearTeeth / TurretDriveTeeth) * MotorStage1Ratio * MotorStage2Ratio); 
  public static final double TurretRadianToRotations = TurretPositionFullRotation / 6.283; //2pi = 6.283
  public static final double TurretRangeInRadian = 200 / 360 * 6.283; // 200 degrees of movement in radian 
  public static final double TurretFullRangeInMotorRotations = (TurretPositionFullRotation * (TurretRangeInRadian/6.283));
  public static final double TurretErrorMoveDeadband = .00005 * TurretFullRangeInMotorRotations; // max allowable error is 0.1% of available range 
  public static final double TurretHomePosition = .5 * TurretFullRangeInMotorRotations; // Assume half of full sweep is the prefered homed position
  
  public static final double TurrentIncrement = .2; //When manually controlling the Turret use drive it this many rotations

  public static final double TurretPotCounterClock = 1.287;// Min Measured
  public static final double TurretPotClockwise = 3.38;// Max measured
  public static final double TurrentCenterPosPot = 2.369; //This is measured vs computed ->  //((TurretPotClockwise - TurretPotCounterClock) / 2) + TurretPotCounterClock;
  public static final double TurretPotCounterClockOffset = 1.300; //1.287 Min Measured
  public static final double TurretPotClockwiseOffset = 3.300;// 3.38 Max measured


  public static final double TurretClockwiseRotHome = -10.616; // Min Meausred will change based on motor power up
  public static final double TurretCounterClockwiseRot = 5.368; // Max measured will change based on motor power up
  

  public static final double TurretRotPerVolt = (TurretCounterClockwiseRot - TurretClockwiseRotHome) / (TurretPotClockwise - TurretPotCounterClock);
  public static final double TurretVoltPerRot = 1 / TurretRotPerVolt;
  public static final double TurrentCenterRotFromHome = (TurrentCenterPosPot -  TurretPotCounterClock) * TurretRotPerVolt;


 /// Climb Constants

  public static final double Climb_ZERO_POS = 0.39;
  public static final double Climb_ENCODER_OFFSET = 0;
  public static final double Climb_Position_MULTIPLIER = 5; //TODO - assumes 5 rotations is climb but need to sort travel distaqnce and sprocket size. 
  public static final double ClimbLeft_pos_offset = 0; //changed from 1 to 0
  public static final double ClimbRight_pos_offset = 0; //changed from 1 to 0
  public static final double Climb_MAX_OUTPUT_VOLTS = 6 ; //Sets Max forward Speed
  public static final double Climb_MAXNeg_OUTPUT_VOLTS = -6; //Sets Max Reverse Speed
  public static final double Climb_kP = .5;
  public static final double Climb_kI = 0; 
  public static final double Climb_kD = 0; 
  public static final double ClimbUpIncrement = 0.1; 
  public static final double ClimbDownIncrement = 0.1; 
  public static final double ClimbMaxExtend = 250;


  /// Spindexer
  
  public static final double Spindexer_MAX_OUTPUT_VOLTS = 2 ; //Sets Max forward Speed
  public static final double Spindexer_MAXNeg_OUTPUT_VOLTS = -2; //Sets Max Reverse Speed
  public static final double Spindexer_kP = .5;
  public static final double Spindexer_kI = 0; 
  public static final double Spindexer_kD = 0; 
  //Set if using gravity for feedforward
  public static final double Spindexer_kg = 0.2;
  public static final double Spindexer_ZERO_POS = 0.39;
  public static final double Spindexer_ENCODER_OFFSET = 0;
  public static final double Spindexer_Speed_On = 20;
  public static final double Spindexer_Speed_Off = 0;


  public static final double SpindexerSpeed_MAX_OUTPUT_VOLTS = 4; //Sets Max forward Speed
  public static final double SpindexerSpeed_MAXNeg_OUTPUT_VOLTS = -4; //Sets Max Reverse Speed
  public static final double SpindexerSpeed_Max_RPS = 40;
  

}
