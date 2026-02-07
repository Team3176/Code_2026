package com.team3176.robot.constants;

public class SuperStructureConstants {


  public static final double GenericTalon_MAX_OUTPUT_VOLTS = 2 ; //Sets Max forward Speed
  public static final double GenericTalon_MAXNeg_OUTPUT_VOLTS = -2; //Sets Max Reverse Speed
   public static final double GenericTalon_kP = .5;
  public static final double GenericTalon_kI = 0; 
  public static final double GenericTalon_kD = 0; 

  public static final double Kicker_MAX_OUTPUT_VOLTS = 2 ; //Sets Max forward Speed
  public static final double Kicker_MAXNeg_OUTPUT_VOLTS = -2; //Sets Max Reverse Speed
   public static final double Kicker_kP = .5;
  public static final double Kicker_kI = 0; 
  public static final double Kicker_kD = 0; 

  //Set if using gravity for feedforward
  public static final double GenericTalon_kg = 0.2;

  public static final double Kicker_kg = 0.2;

  public static final double GenericTalon_ZERO_POS = 0.39;

  public static final double Kicker_ZERO_POS = 0.39;  

  public static final double GenericTalon_ENCODER_OFFSET = 0;

  public static final double Kicker_ENCODER_OFFSET = 0;

  public static final double GenericTalonSpeed_MAX_OUTPUT_VOLTS = 4; //Sets Max forward Speed
  public static final double GenericTalonSpeed_MAXNeg_OUTPUT_VOLTS = -4; //Sets Max Reverse Speed
  public static final double GenericTalonSpeed_Max_RPS = 20;

  public static final double KickerSpeed_MAX_OUTPUT_VOLTS = 4; //Sets Max forward Speed
  public static final double KickerSpeed_MAXNeg_OUTPUT_VOLTS = -4; //Sets Max Reverse Speed
  public static final double KickerSpeed_Max_RPS = 20;



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




}
