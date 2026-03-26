package frc.robot.constants;

import frc.robot.constants.BaseConstants;
import frc.robot.constants.BaseConstants.RobotType;

/** File for storing all hardware IDs to prevent double assignments */
public class Hardwaremap {
  /*
   * Superstructer CIDs & CBNs
   */
// Assign the CAN IDs for the network - must only use once
  /*public static final int genericTalon_CID = 100;
  public static final int genericTalonSpeed_CID = 110;
  public static final int genericTalonCancoder_CID = 120;    
  public static final int genericTalonLeaderSpeed_CID = 121;
  public static final int genericTalonFollowerSpeed_CID = 122;
  public static final int genericSparkFlex_CID = 300;
  public static final int genericSparkFlexSpeed_CID = 310;
  public static final int genericSparkFlexCancoder_CID = 320;    
  public static final int genericSparkFlexLeaderSpeed_CID = 410;
  public static final int genericSparkFlexFollowerSpeed_CID = 420;
*/


  //public static final int Kicker_CID = 53;
  //public static final int KickerCancoder_CID = 530; 
  public static final int KickerSpeed_CID = 53;
  
//added a 0 to the end of each of the kicker CID's
  
  //public static final int HoodSpeed_CID = 570;
  public static final int Hood_CID = 57;
  public static final int HoodSpark_CID = 57;
  public static final int HoodCancoder_CID = 57;    

  public static final int ClimbLeft_CID = 58;
 // public static final int ClimbRight_CID = 59;

  public static final int Spindexer_CID = 52;

// Intake Control Constants
  public static final int IntakePosition_CID = 50;
  public static final int IntakeRoller_CID = 51;
  //public static final int IntakePositionCancoder_CID = 500;    

  public static final int shooterLeaderSpeed_CID = 55;
  public static final int shooterFollowerSpeed_CID = 56;

  public static final int turretRotation_CID = 54;
  //public static final int turretRotationCancoder_CID = 540;

  
  public static int PDH_CID = 11;
 // public static int laserCan_CID = 48;
  public static int pigeon_CID = 5;
 // public static int TOF_LEFT_CID = 8;
 // public static int TOF_RIGHT_CID = 7;
 // public static int TOF_CENTER_CID = 9;


// Tell the robot which CAN network to configure the devices to

/// Generic Motor Controller Setup - TODO Delete
  public static final String genericTalon_CBN = "rio";
  public static final String genericTalonSpeed_CBN = "rio";
  public static final String genericTalonDualSpeed_CBN = "rio";
  public static final String genericSparkFlex_CBN = "rio";
  public static final String genericSparkFlexSpeed_CBN = "rio";
  public static final String genericSparkFlexDualSpeed_CBN = "rio";

  
  /// 2026 Mechanisms 
  public static final String shooterDualSpeed_CBN = "rio";
  
  public static final String Kicker_CBN = "rio";
  public static final String KickerSpeed_CBN = "rio";
  public static final String Hood_CBN = "rio";
  public static final String HoodSpeed_CBN = "rio";
  public static final String turretRotation_CBN = "rio";
  public static final String Intake_CBN = "rio";
  public static final String Spindexer_CBN = "rio";

  public static final String Climb_CBN = "rio";
  
  ///Generic IO for all Games - 
  public static String PDH_CBN = "rio";


  /* Use this area to define which Digital IO pins on the RIO limit swtiches and devices are attached too.  */

  public static final int turretClockwiseLimitSwitch_DIO = 5;
  public static final int turretCounterClockwiseLimitSwitch_DIO = 6;
  public static final int hoodToplimitswitch_DIO = 7;
  public static final int hoodBottomlimitswitch_DIO = 8;

 /* us this section to assign PWM Ports from the RIO - IE LED Lights via Blinkin */
  public static final int blinkin_pwm_port = 9;


}
