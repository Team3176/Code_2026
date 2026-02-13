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
  public static final double GenericSpark_MAX_OUTPUT_VOLTS = 2 ; //Sets Max forward Speed
  public static final double GenericSpark_MAXNeg_OUTPUT_VOLTS = -2; //Sets Max Reverse Speed
  public static final double GenericSpark_kP = .5;
  public static final double GenericSpark_kI = 0; 
  public static final double GenericSpark_kD = 0; 

  //Set if using gravity for feedforward
  public static final double GenericSpark_kg = 0.2;

  public static final double GenericSpark_ZERO_POS = 0.39;

  public static final double GenericSpark_ENCODER_OFFSET = 0;


  public static final double GenericSparkSpeed_MAX_OUTPUT_VOLTS = 4; //Sets Max forward Speed
  public static final double GenericSparkSpeed_MAXNeg_OUTPUT_VOLTS = -4; //Sets Max Reverse Speed
  public static final double GenericSparkSpeed_Max_RPM = 20;


  public static final double GenericSparkDualSpeed_MAX_OUTPUT_VOLTS = 4; //Sets Max forward Speed
  public static final double GenericSparkDualSpeed_MAXNeg_OUTPUT_VOLTS = -4; //Sets Max Reverse Speed
  public static final double GenericSparkDualSpeed_Max_RPM = 20;

  public static final double Kicker_MAX_OUTPUT_VOLTS = 2 ; //Sets Max forward Speed
  public static final double Kicker_MAXNeg_OUTPUT_VOLTS = -2; //Sets Max Reverse Speed
  public static final double Kicker_kP = .5;
  public static final double Kicker_kI = 0; 
  public static final double Kicker_kD = 0; 
  public static final double Kicker_kg = 0.2;
  public static final double Kicker_ZERO_POS = 0.39;  
  public static final double Kicker_ENCODER_OFFSET = 0;


  public static final double Hood_ZERO_POS = 0.39;

  public static final double Hood_ENCODER_OFFSET = 0;

  public static final double Hood_Position_MULTIPLIER = 1; // ten tooth pinion 23 tooth rack, 1 to 1 gear ratio, input is from -1 to 1
  public static final double Hood_pos_offset = 1;


public static final double Hood_MAX_OUTPUT_VOLTS = 2 ; //Sets Max forward Speed
  public static final double Hood_MAXNeg_OUTPUT_VOLTS = -2; //Sets Max Reverse Speed
   public static final double Hood_kP = .5;
  public static final double Hood_kI = 0; 
  public static final double Hood_kD = 0; 

  //Set if using gravity for feedforward
  public static final double Hood_kg = 0.2;


      public static final double KickerSpeed_MAX_OUTPUT_VOLTS = 4; //Sets Max forward Speed
  public static final double KickerSpeed_MAXNeg_OUTPUT_VOLTS = -4; //Sets Max Reverse Speed
  public static final double KickerSpeed_Max_RPS = 20;

///
// Shooter Constants
///

  public static final double ShooterDualSpeed_MAX_OUTPUT_VOLTS = 12 ; //Sets Max forward Speed
  public static final double ShooterDualSpeed_MAXNeg_OUTPUT_VOLTS = -12; //Sets Max Reverse Speed


  public static final double ShooterDualSpeed_Max_RPS = 100; // speed is multiplied by two due to joystick so currently this equals 50 * 2 = 100


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
 public static final double TurretDegreesToRotations = TurretPositionFullRotation / 360; // currently .083 rotations per degree
 public static final double TurretRangeInDegrees = 200;
 public static final double TurretFullRangeInMotorRotations = (TurretPositionFullRotation * (TurretRangeInDegrees/360));
 public static final double TurretErrorMoveDeadband = .00005 * TurretFullRangeInMotorRotations; // max allowable error is 0.1% of available range 
 public static final double TurretHomePosition = .5 * TurretFullRangeInMotorRotations; // Assume half of full sweep is the prefered homed position





}
