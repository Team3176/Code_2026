package com.team3176.robot.constants;

import com.team3176.robot.constants.BaseConstants;
import com.team3176.robot.constants.BaseConstants.RobotType;

/** File for storing all hardware IDs to prevent double assignments */
public class Hardwaremap {
  /*
   * Superstructer CIDs & CBNs
   */
// Assign the CAN IDs for the network - must only use once
  public static final int genericTalon_CID = 10;
  public static final int genericTalonSpeed_CID = 11;
  public static final int genericTalonCancoder_CID = 12;    
  public static final int armPivot_CID = 29; 

  public static final int genericTalonLeaderSpeed_CID = 21;
  public static final int genericTalonFollowerSpeed_CID = 22;


  public static final int genericSparkFlex_CID = 30;
  public static final int genericSparkFlexSpeed_CID = 31;
  public static final int genericSparkFlexCancoder_CID = 32;    
 

  public static final int genericSparkFlexLeaderSpeed_CID = 41;
  public static final int genericSparkFlexFollowerSpeed_CID = 42;



  
  public static int PDH_CID = 1;
  public static int laserCan_CID = 48;
  public static int pigeon_CID = 5;
  public static int TOF_LEFT_CID = 8;
  public static int TOF_RIGHT_CID = 7;
  public static int TOF_CENTER_CID = 9;


// Tell the robot which CAN network to configure the devices to
  public static final String genericTalon_CBN = "rio";
  public static final String genericTalonSpeed_CBN = "rio";
  public static final String genericTalonDualSpeed_CBN = "rio";

  public static final String genericSparkFlex_CBN = "rio";
  public static final String genericSparkFlexSpeed_CBN = "rio";
  public static final String genericSparkFlexDualSpeed_CBN = "rio";


  public static final String conveyor_CBN = "rio";
  public static final String shooterWheelUpper_CBN = "rio";
  public static final String shooterWheelLower_CBN = "rio";
  public static final String shooterTransfer_CBN = "rio";
  public static final String shooterPivot_CBN = "rio";
  public static final String armRoller_CBN = "rio";
  public static final String armPivot_CBN = "rio";
  public static final String indexerPivot_CBN = "rio";
  /*   public static final String LaserCan_CBN = "rio"; */
  public static final String climb_CBN = "rio";
  public static final String elevatorLeft_CBN = "rio";
  public static final String elevatorRight_CBN = "rio";
  public static final String indexerRoller_CBN = "rio";

  public static String PDH_CBN = "rio";


  /* Use this area to define which Digital IO pins on the RIO limit swtiches and devices are attached too.  */

  // public static final int elevatorBottomLimitSwitch_DIO = 5;
  // public static final int elevatorTopLimitSwitch_DIO = 6;

 /* us this section to assign PWM Ports from the RIO - IE LED Lights via Blinkin */
  public static final int blinkin_pwm_port = 9;

  /*
  TODO: This AREA DEFINES THE PODS need to update for 2026
   */
  // statics constants for swerve pods
  public static final String SWERVEPOD_CTRE_CBN =
      BaseConstants.getRobot() == RobotType.ROBOT_2025C ? "canivore" : "rio";
  public static final String SWERVEPOD_CBN = SWERVEPOD_CTRE_CBN;
  public static final SwervePodHardwareID POD001 =
      new SwervePodHardwareID(1, 10, SWERVEPOD_CBN, 11, SWERVEPOD_CBN, 12, SWERVEPOD_CBN, -38.6, true);
  public static final SwervePodHardwareID POD002 =
      new SwervePodHardwareID(2, 20, SWERVEPOD_CBN, 21, SWERVEPOD_CBN, 22, SWERVEPOD_CBN, 56.25, true);
  public static final SwervePodHardwareID POD003 =
      new SwervePodHardwareID(3, 30, SWERVEPOD_CBN, 31, SWERVEPOD_CBN, 32, SWERVEPOD_CBN, -149.678, true);
  public static final SwervePodHardwareID POD004 =
      new SwervePodHardwareID(
          4, 40, SWERVEPOD_CBN, 41, SWERVEPOD_CBN, 42, SWERVEPOD_CBN, -168.135, false);
  public static final SwervePodHardwareID POD005 =
      new SwervePodHardwareID(5, 13, SWERVEPOD_CBN, 14, SWERVEPOD_CBN, 15, SWERVEPOD_CBN, 103.525, false);

  public static final String SWERVEPOD_REV_CBN = "rio";

  public static final SwervePodHardwareID FR = POD001;
  public static final SwervePodHardwareID FL = POD002;
  public static final SwervePodHardwareID BL = POD004;
  public static final SwervePodHardwareID BR = POD003;

  public static final int STEER_FR_CID = 11;
  public static final int STEER_FL_CID = 21;
  public static final int STEER_BL_CID = 31;
  public static final int STEER_BR_CID = 41;
  public static final String STEER_FR_CBN = SWERVEPOD_REV_CBN;
  public static final String STEER_FL_CBN = SWERVEPOD_REV_CBN;
  public static final String STEER_BL_CBN = SWERVEPOD_REV_CBN;
  public static final String STEER_BR_CBN = SWERVEPOD_REV_CBN;
}
