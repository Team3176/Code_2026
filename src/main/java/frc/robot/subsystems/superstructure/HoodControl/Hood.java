package frc.robot.subsystems.superstructure.HoodControl;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.smartdashboard;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.BaseConstants.Mode;
import frc.robot.constants.BaseConstants.RobotType;

import frc.robot.constants.*;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunablePID;
import frc.robot.mathUtil.LinearInterpolationTable;;;



public class Hood extends SubsystemBase {
 private static Hood instance;
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final TunablePID positionMotorPID;
  private Timer deployTime = new Timer();
  private double positionSetpoint;
  private double position_offset = SuperStructureConstants.Hood_ENCODER_OFFSET;
  private boolean ishomed = false;
  private double positionHome = SuperStructureConstants.Hood_ZERO_POS;
  private double currentPosRot = 0;
  private double[][] hoodShootLUT = {SuperStructureConstants.botDistanceLUT, SuperStructureConstants.botShootHoodPosLUT};
  private double[][] hoodPassLUT = {SuperStructureConstants.botDistanceLUT, SuperStructureConstants.botPassHoodPosLUT};
  
  private LinearInterpolationTable distanceToHoodGoalLUT = new LinearInterpolationTable(hoodShootLUT);
  private LinearInterpolationTable distanceToHoodPassLUT = new LinearInterpolationTable(hoodPassLUT);


  private Hood(HoodIO io) {
    this.io = io;
    this.positionMotorPID = new TunablePID("HoodPIDConstants", SuperStructureConstants.Hood_kP, SuperStructureConstants.Hood_kI, SuperStructureConstants.Hood_kD);
  
    this.positionHome = inputs.HoodPositionRot;



  }

  public boolean getTopLimitswitch() {
    return inputs.hoodToplimitswitch;
  }

  public boolean getBottomLimitswitch() {
    return inputs.hoodBottomlimitswitch;
  }


  public static Hood getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        //instance = new Hood(new HoodIOTalon() {});
        instance = new Hood(new HoodIOSpark() {});
      } else {
        instance = new Hood(new HoodIOSim() {});
      }
    }
    return instance;
  }



  public Command Hood2Home() {
    return this.runOnce(
      () -> {
       setHoodVoltagePos(positionHome); 
      }); 
    }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runHood(DoubleSupplier position) {
    return this.run(
      () -> { 
        setHoodVoltagePos(Math.abs(position.getAsDouble() * SuperStructureConstants.Hood_Position_MULTIPLIER));// using throttle input only go positive
      });
  }

    public Command runHoodFromDistance(DoubleSupplier distance, BooleanSupplier isInTrench, BooleanSupplier isPass) {
    return this.run(
      () -> { 
        hoodRotationsFromVision(distance.getAsDouble(), isInTrench.getAsBoolean(), isPass.getAsBoolean());
      });
  }



  public Command runHoodVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setHoodVolts(position.getAsDouble());
      }, 
      () -> {
        setHoodVolts(0.0);
      });
  }


  public Command setHoodONEPos() {
    return this.runOnce(
      () -> { 
        setHoodVoltagePos(SuperStructureConstants.HoodONERot);
      });
   }

  public Command setHoodTWOPos() {
    return this.runOnce(
      () -> { 
        setHoodVoltagePos(SuperStructureConstants.HoodTWORot);
      });
   }

     public Command setHoodTHREEPos() {
    return this.runOnce(
      () -> { 
        setHoodVoltagePos(SuperStructureConstants.HoodTHREERot);
      });
   }

     public Command setHoodFOURPos() {
    return this.runOnce(
      () -> { 
        setHoodVoltagePos(SuperStructureConstants.HoodFOURRot);
      });
   }
   
  private void setHoodVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    io.setHoodVolts(volts);
  }

  private void setHoodVoltagePos(double position) {
    io.setHoodVoltagePos(position);
  }

  public Command deployFromHomeCmd() {
    return this.runOnce(
      () -> {
        deployFromHome();
      }
    );
  }
  //Used to reset home position based on what is read from sensor currently
  public void setCurrentHomePos() {
    this.positionHome = inputs.HoodPositionRot;
    inputs.HoodHomePosROT = this.positionHome;
  }
  
  public void setCurrentPos() {
    this.currentPosRot = inputs.HoodPositionRot;
  }

  public void deployFromHome() {
    setCurrentPos();
    double deployPos = this.currentPosRot + SuperStructureConstants.HoodUpIncrement;
    setHoodVoltagePos(deployPos);
  }

  
  
  public Command retractTowardHome() {
    return this.runOnce(
      () -> {
        retractTowardHomePostion();
      }
    );
  }

  public void retractTowardHomePostion () {
    setCurrentPos();
    double retractIncrement = SuperStructureConstants.HoodDownIncrement;
    if (inputs.hoodBottomlimitswitch){
      retractIncrement = 0;
    }
    
    double deployPos = this.currentPosRot - retractIncrement;
    setHoodVoltagePos(deployPos);
  }

  /*public Command incrementalDeploy() {
    return this.runOnce(
      () -> {
        deployIncremental();
      }
    );
  }

  public void deployIncremental() {
    double currentPos = inputs.HoodPositionRot;
    currentPos = currentPos + 0.25;
    setHoodVoltagePos(currentPos);
  }*/


//Use this method when shooting at the goal
  private void hoodRotationsFromVision(Double distance, Boolean isInTrench, Boolean isPass){
    // current position in rotations
    double currentPosition = inputs.HoodPositionRot;
    //double hoodPositionFromHoop = (Math.pow (4.25, distance)) - .0797; // update this based on table data for hood distance
    double hoodPositionFromPOI = 0;
    if (isPass){
      hoodPositionFromPOI = distanceToHoodPassLUT.interpolate(distance); // update this based on table data for hood distance
    }
    else {
      hoodPositionFromPOI = distanceToHoodGoalLUT.interpolate(distance); // update this based on table data for hood distance
    }
    double hoodPositionRequest = currentPosition;

    if (isInTrench){ //Drop the hood
      hoodPositionRequest = SuperStructureConstants.Hood_ZERO_POS;
    }
    else {
      if (inputs.hoodToplimitswitch || hoodPositionFromPOI < SuperStructureConstants.Hood_MaxPosition){
        hoodPositionRequest = hoodPositionFromPOI;
      }
      else if (inputs.hoodBottomlimitswitch || hoodPositionFromPOI >= SuperStructureConstants.Hood_ZERO_POS){
        hoodPositionRequest = hoodPositionFromPOI;
      }
    }
    
    io.setHoodVisionPos(hoodPositionRequest);

   // SmartDashboard.putNumber("HoodPosition Request", hoodPositionRequest);
    SmartDashboard.putNumber("Hood Distance Command", distance);
  }


 
  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("Hood", inputs);
     
    Logger.recordOutput("Hood/setpoint", this.positionSetpoint);
   
    positionMotorPID.checkParemeterUpdate();

    SmartDashboard.putNumber("Hood Position", inputs.HoodPositionRot);
   // SmartDashboard.putNumber("Hood Volts", inputs.HoodAppliedVolts);
   // SmartDashboard.putBoolean("Hood Top Switch", inputs.hoodToplimitswitch);
   // SmartDashboard.putBoolean("Hood Bottom Switch", inputs.hoodBottomlimitswitch);
    
       // Use Limit Switches not to break anything - May be double dipping on limit switches based on method call. - safe than sorry

    if (inputs.hoodToplimitswitch && inputs.HoodAppliedVolts > 0) {
      io.setHoodVolts(0);
    }

    if (inputs.hoodBottomlimitswitch && inputs.HoodAppliedVolts < 0) {
      io.setHoodVolts(0);
    }
  
    
    //SmartDashboard.putNumber("Hood Position", inputs.HoodPositionRot);
  }
}
