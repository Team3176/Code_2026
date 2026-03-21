// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

public final class ControllerConstants {
  public static final int ROT_ID = 0;
  public static final int TRANS_ID = 1;
  public static final int OP_ID = 2;
  public static final int SWITCH_ID = 3;
  public static final double TRIGGER_THRESHOLD = 0.1;

  public static final double SLOW_DRIVE_MULT =   0.5; // TODO: CHECK THESE VALUES ON NEW BOT but these are for 2021
  public static final int FORWARD_AXIS_INVERSION = -1;
  public static final int STRAFE_AXIS_INVERSION = -1;
  public static final int SPIN_AXIS_INVERSION = -1;
  public static final double SPIN_AXIS_SCALER= 0.5;
}


/// EURO Truck Switch Box
///  Park Button Down B6 
///  Park Button Up B7
///  Start Button B 21
/// 
///    19   17   15   11
///   [ x] [ x] [  ] [  ]   // Red Swtiches 
///    20   18   16   14
/// 
///   (10) ( 9) ( 8) (26)  //Round buttons
/// 
///   [25] [24] [23] [27]  // Square Buttons
/// 
///    29    1    3    5
///   {  } {  } {  } {  }  // toggels 
///    28    0    2    4