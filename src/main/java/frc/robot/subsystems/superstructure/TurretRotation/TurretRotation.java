package frc.robot.subsystems.superstructure.TurretRotation;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import yams.mechanisms.positional.Arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.BaseConstants.Mode;
import frc.robot.constants.BaseConstants.RobotType;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.constants.*;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunablePID;





public class TurretRotation extends SubsystemBase {
 private static TurretRotation instance;
  private final TurretRotationIO io;
  private final TurretRotationIOInputsAutoLogged inputs = new TurretRotationIOInputsAutoLogged();

  private Timer deployTime = new Timer();
  private double positionSetpoint;
  private double position_offset = SuperStructureConstants.TurretRotation_ENCODER_OFFSET;
  private double desiredRotations = 0.0;
  private boolean ishomed = false;
  private double positionHome = SuperStructureConstants.TurretRotation_ZERO_POS;
 // private LEDSubsystem leds;
  private double homePos = 0;
  private double currentPosRot = 0;
  private double deltaRot = 0;
  private double centerRot = 0;
  private double commandedRot = 0;
  private double goToPosRotation = 0;
  private double demandedRotAbsoluteDeg = 0;
  private double demandedRotAbsoluteRad = 0;
  private double demandedDistance = 0;


  private TurretRotation(TurretRotationIO io) {
    this.io = io;
   // leds = LEDSubsystem.getInstance();
  }

  public boolean getClockwiseLimitswitch() {
    return inputs.turretClockwiselimitswitch;
  }

  public boolean getCounterClockwiseLimitswitch() {
    return inputs.turretCounterclockwiselimitswitch;
  }


  public static TurretRotation getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new TurretRotation(new TurretRotationIOTalon() {});
      } 
      else {
        instance = new TurretRotation(new TurretRotationIOSim() {});
      }
    }
    return instance;
  }



  public Command TurretRotation2Home() {
    return this.runOnce(
      () -> {
       setTurretRotationVoltagePos(positionHome); 
      }); 
    }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runTurretRotation(DoubleSupplier position) {
    return this.run(
      () -> { 
        setTurretRotationVoltagePos(position.getAsDouble());
      });
  }

  public Command runTurretRotationVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setTurretRotationVolts(position.getAsDouble());
      }, 
      () -> {
        setTurretRotationVolts(0.0);
      });
  }

   private void setTurretRotationVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    io.setTurretRotationVolts(volts);
  }

  private void setTurretRotationVoltagePos(double position) {
    io.setTurretRotationVoltagePos(position);
  }

  //Used to reset home position based on what is read from sensor currently
  public void setCurrentHomePos() {
    this.homePos = inputs.turretRotationPositionRot;
  }

  public void setCurrentPosToIncrementFrom() {
    this.currentPosRot = inputs.turretRotationPositionRot;
  }

  public Command moveTurretRightbyIncrement() {
    return this.runOnce(
      () -> {
        moveTurretRightIncrement();
      }
    );
  }

  public void moveTurretRightIncrement() {
    setCurrentPosToIncrementFrom();
    double moveToPos = this.currentPosRot;
    
    if (!inputs.turretClockwiselimitswitch){
      moveToPos = moveToPos + SuperStructureConstants.TurrentIncrement;

    }
     setTurretRotationVoltagePos(moveToPos);
  }
  
  public Command moveTurretLeftbyIncrement() {
    return this.runOnce(
      () -> {
        moveTurretLeftIncrement();
      }
    );
  }

  public void moveTurretLeftIncrement () {
    setCurrentPosToIncrementFrom();
    double moveToPos = this.currentPosRot;
    if (!inputs.turretCounterclockwiselimitswitch){
      moveToPos = moveToPos- SuperStructureConstants.TurrentIncrement;

    }

    setTurretRotationVoltagePos(moveToPos);
  }

  //Use this command for target tracking - 
  public Command runTurretRotationFromVision(DoubleSupplier positionErrorDegrees, BooleanSupplier isTargetLocked, LEDSubsystem leds ) {
    double positionErrorRotations =   positionErrorDegrees.getAsDouble(); // SuperStructureConstants.TurretDegreesToRotations; //convert degrees of error into rotations
    
    return this.run(
      () -> { 
        turretRotationFromVision(positionErrorDegrees.getAsDouble(), isTargetLocked.getAsBoolean(), leds);
      });
  }

  private void turretRotationFromVision(double positionErrorRotations, boolean isTargetLocked , LEDSubsystem leds){
      // curret position in rotations
      double curretPosition = inputs.turretRotationPositionRot;
      //io.setTurretRotationError( positionErrorRotations, isTargetLocked);
      if (isTargetLocked){
        if ((Math.abs(positionErrorRotations) > SuperStructureConstants.TurretErrorMoveDeadband )){
          // adjust the position based on the error identified 
           if (inputs.turretClockwiselimitswitch && positionErrorRotations < 0) {
            positionErrorRotations = 0;
          }
           else if (inputs.turretCounterclockwiselimitswitch && positionErrorRotations >= 0) {
            positionErrorRotations = 0;
          }
          
          io.setTurretRotationError(curretPosition + positionErrorRotations * SuperStructureConstants.TurretRadianToRotations, isTargetLocked);
          leds.turretTracking();
        }
        else {
          leds.turretLockedOn();
        }
      }
      else {
        leds.turretVisonLost();
      }
  }
 

  public Command moveTurretToCenter() {
    return this.runOnce(
      () -> {
        turrentToCenterPoint();
      }
    );
  }

  private void turrentToCenterPoint(){
    setCurrentPosToIncrementFrom();
    
    
    double goToPosRot =  (SuperStructureConstants.TurrentCenterPosPot - inputs.turretAnalogPOT_Value) * SuperStructureConstants.TurretRotPerVolt;
    setTurretRotationVoltagePos(goToPosRot + this.currentPosRot);
     SmartDashboard.putNumber("Turret Move Rotations", goToPosRot + this.currentPosRot);
     desiredRotations = goToPosRot + this.currentPosRot;

      //This should be used as the home position update each time it is centered
      this.homePos = desiredRotations; 
  }

    public Command moveTurretToMaxClockwise() {
    return this.runOnce(
      () -> {
        turrentToMaxClockwise();
      }
    );
  }

  private void turrentToMaxClockwise(){
    setCurrentPosToIncrementFrom();
    
    double goToPosRot =  (SuperStructureConstants.TurretPotClockwiseOffset - inputs.turretAnalogPOT_Value) * SuperStructureConstants.TurretRotPerVolt;
    setTurretRotationVoltagePos(goToPosRot + this.currentPosRot);
     SmartDashboard.putNumber("Turret Move Rotations", goToPosRot + this.currentPosRot);
     desiredRotations = goToPosRot + this.currentPosRot;
  }
    public Command moveTurretToMaxCounterClockwise() {
    return this.runOnce(
      () -> {
        turrentToMaxCounterClockwise();
      }
    );
  }

  private void turrentToMaxCounterClockwise(){
    setCurrentPosToIncrementFrom();
    
    double goToPosRot =  (SuperStructureConstants.TurretPotCounterClockOffset - inputs.turretAnalogPOT_Value) * SuperStructureConstants.TurretRotPerVolt;
    setTurretRotationVoltagePos(goToPosRot + this.currentPosRot);
     SmartDashboard.putNumber("Turret Move Rotations", goToPosRot + this.currentPosRot);
     desiredRotations = goToPosRot + this.currentPosRot;
  }


/* Scoring Table                (0,0)
    _______________________________
          |                       |
          |            {_*  robot |  robotRotationVsFeild - get angle offset from robot front to blue wall 
          |           /  | (1,1)  |
          |         /             |
          |       /      |        |
          |     /                 |
          |___ /_}_ _ _ _|        |  robotAngleToHub - use the coordinates of robot to get an angle to the robot     
          |  |                    |
(4.6,8.0)  x | hub                |
          |  |                    |
 */
  //Use this command for target tracking - 
  public Command runTurretRotationFromVisionLocation(DoubleSupplier turretRotRaidianToPoint) {
  //  double turretRotAbsolute =   turretRotToPoint.getAsDouble(); // SuperStructureConstants.TurretDegreesToRotations; //convert degrees of error into rotations
    
    return this.run(
      () -> { 
        turretRotationFromLocation(turretRotRaidianToPoint.getAsDouble());
      });
  }

  private void turretRotationFromLocation(double turretRotRaidianAbsolute){
      // curret position in rotations
      double curretPosition = inputs.turretRotationPositionRot;
      //io.setTurretRotationError( positionErrorRotations, isTargetLocked);
      double goToPosRot = curretPosition;
      //Identify the rotations of center point
      double centerPosRot =  ((SuperStructureConstants.TurrentCenterPosPot - inputs.turretAnalogPOT_Value) * SuperStructureConstants.TurretRotPerVolt) + curretPosition;
      double maxLeftPosRot =  ((SuperStructureConstants.TurretPOTCounterClockMax - inputs.turretAnalogPOT_Value) * SuperStructureConstants.TurretRotPerVolt) + curretPosition;
      double maxRightPosRot =  ((SuperStructureConstants.TurretPOTClockwiseMax - inputs.turretAnalogPOT_Value) * SuperStructureConstants.TurretRotPerVolt) + curretPosition;
      
      double deltaRotations = turretRotRaidianAbsolute * SuperStructureConstants.TurretRadianToRotations;
      
      
      
      goToPosRot = centerPosRot - deltaRotations;
      
      if( goToPosRot < maxLeftPosRot){
        goToPosRot = maxLeftPosRot;
      }
      if (goToPosRot > maxRightPosRot) {
        goToPosRot = maxRightPosRot;
      }

      commandedRot = goToPosRot;

      
      //Used to update the Smart Dashboard;
      desiredRotations = centerPosRot - deltaRotations;
      deltaRot = deltaRotations;
      centerRot = centerPosRot;
      goToPosRotation = goToPosRot;
      demandedRotAbsoluteDeg = Units.radiansToDegrees(turretRotRaidianAbsolute);
      demandedRotAbsoluteRad = turretRotRaidianAbsolute;
     


      setTurretRotationVoltagePos(goToPosRot);
      

  }
 

  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("TurretRotation", inputs);
     
    Logger.recordOutput("TurretRotation/setpoint", this.positionSetpoint);
   
    SmartDashboard.putBoolean("Turret LimitSwitch Clockwise", inputs.turretClockwiselimitswitch);
    SmartDashboard.putBoolean("Turret LimitSwitch Counter Clockwise", inputs.turretCounterclockwiselimitswitch);
    SmartDashboard.putNumber("Turret Current Rotation", inputs.turretRotationPositionRot);
    SmartDashboard.putNumber("Turret Desired Rotation", desiredRotations);
    SmartDashboard.putNumber("Turret Delta Rotations", deltaRot);
    SmartDashboard.putNumber("Turret Center Rotations", centerRot); 
    SmartDashboard.putNumber("Turret Commanded Rotations", commandedRot);
    SmartDashboard.putNumber("Turret GoTO Rotations", goToPosRotation);
    
    SmartDashboard.putNumber("Turret demand Degrees", demandedRotAbsoluteDeg);
    SmartDashboard.putNumber("Turret demand Rad", demandedRotAbsoluteRad);
    
    SmartDashboard.putNumber("Turret POT Volgate", inputs.turretAnalogPOT_Value);
   
   // Use Limit Switches not to break anything - May be double dipping on limit switches based on method call. - safe than sorry

    if (inputs.turretClockwiselimitswitch && inputs.turretRotationAppliedVolts > 0) {
      io.setTurretRotationVoltage(0);
    }

    if (inputs.turretCounterclockwiselimitswitch && inputs.turretRotationAppliedVolts <= 0) {
      io.setTurretRotationVoltage(0);
    }
    
  }
}
