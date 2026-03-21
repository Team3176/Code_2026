package frc.robot.constants;
/// 2026 Rebuilt information on the match and field 
/// 
public class MatchConstants {

    //Timers
    /// 2026 Rebuilt
    ///Autonomous: 20 seconds
    /// Auto Count Period? 3 seconds
    ///Teleoperated: 110 seconds
    ///End Game: 30 seconds
    /// 
    /// Total Time = 160 seconds
    public static final double ENDGAMEALERT_Time = 33 ; //Endgame Time Seconds Left 
    public static final double ENDAUTOALERT_Time = 20 ; //Endgame Time Seconds Left 

    ///2026 Position Calucation Sheet 
    /// https://docs.google.com/spreadsheets/d/1OCndV1KGRs-ASCikktj9pBJ_q7Bz6eaIaeRv8FXyaLU/edit?usp=sharing
    
    // AndyMark Field Location of Blue and Red HUBs
    public static final double[] blueGoalLocation = { 4.611, 4.021 }; // (x,y) blueGoal is at (181.56,158.32) inches
    public static final double[] redGoalLocation = { 11.901, 4.021 }; // (x,y)redGoal is at (468.56,158.32) inches

    // AndyMark Field Location of "Passing Zones"
    public static final double[] bluePassLocationNearSide = {  2.286, 2.091 }; // 
    public static final double[] bluePassLocationFarSide  = {  2.286, 5.952 }; // 
    public static final double[] redPassLocationNearSide  = { 14.227, 2.091 }; // 
    public static final double[] redPassLocationFarSide   = { 14.227, 5.952 }; // 

    //AndyMark Field Locations of "Hood Down Zones"
    //Keep It Simple 
    public static final double hoodDownBlueSideEnter_X	  =  4.192524;	
    public static final double hoodDownBlueSideExit_X	  =  5.030724;
    public static final double hoodDownRedSideExit_X	  = 11.482324;
    public static final double hoodDownRedSideEnter_X     = 12.320524;
    public static final double hoodDownScoringTableSide_Y =  1.27;
    public static final double hoodDownOppositeSide_Y     =  6.772656;


    //AndyMark Field Auto Shooting / Passing Locations
    //Shooting
    public static final double autoShootBlueLessThan_X	   =  4.192524;	
    public static final double autoShootRedGreaterThan_X   = 12.320524;

    //Passing Auto Pass when between 
    public static final double autoPassWhenGreaterThan_X  =  5.030724;
    public static final double autoPassWhenLessThan_X	  = 11.482324; 
    public static final double centerLine_Y               =  4.021328;         
}

