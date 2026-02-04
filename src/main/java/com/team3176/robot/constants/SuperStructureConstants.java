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

public static final double Hood_MAX_OUTPUT_VOLTS = 2 ; //Sets Max forward Speed
  public static final double Hood_MAXNeg_OUTPUT_VOLTS = -2; //Sets Max Reverse Speed
   public static final double Hood_kP = .5;
  public static final double Hood_kI = 0; 
  public static final double Hood_kD = 0; 

  //Set if using gravity for feedforward
  public static final double Hood_kg = 0.2;

  public static final double Hood_ZERO_POS = 0.39;

  public static final double Hood_ENCODER_OFFSET = 0;

  public static final double Hood_Position_MULTIPLIER = 1; // ten tooth pinion 23 tooth rack, 1 to 1 gear ratio, input is from -1 to 1
  public static final double Hood_pos_offset = 1;
}
