package frc.robot.subsystems.superstructure.ShooterControl;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.positional.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.BaseConstants.Mode;
import frc.robot.constants.BaseConstants.RobotType;
import frc.robot.mathUtil.LinearInterpolationTable;
import frc.robot.constants.*;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunablePID;



public class ShooterControl extends SubsystemBase {
private static ShooterControl instance;
private final ShooterControlIO io;
private final ShooterControlIOInputsAutoLogged inputs = new ShooterControlIOInputsAutoLogged();
private double[][] shootSpeedGoalLUT = {SuperStructureConstants.botDistanceLUT, SuperStructureConstants.botShooterSpeedLUT};
private LinearInterpolationTable distanceToGoalShooterLUT = new LinearInterpolationTable(shootSpeedGoalLUT);
private boolean shooterIsShutDown;

private double currentShootRequst = SuperStructureConstants.Shooter_Speed_On;



private ShooterControl(ShooterControlIO io) {
  this.io = io;

  shooterIsShutDown = false;

  }

  public static ShooterControl getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new ShooterControl(new ShooterControlIOTalon() {});
      } else {
        instance = new ShooterControl(new ShooterControlIOSim() {});
      }
    }
    return instance;
  }




  // USE THESE COMMANDS FOR Dual Motor SPEED CONTROL

  private void setDualShooterSpeedControl(double Speed_RPS) {

      io.setDualShooterSpeedVelocity(Speed_RPS);
  
  }




  public void setDualShooterSpeedCoast() {
    io.setDualShooterSpeedBrakeMode(false);
  }

  public void setDualShooterSpeedBrake() {
    io.setDualShooterSpeedBrakeMode(true);
  }

  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runDualShooterSpeed(DoubleSupplier Speed_RPS) {
    return this.run(
      () -> { 
        setDualShooterSpeedControl((Speed_RPS.getAsDouble() +1 ) * SuperStructureConstants.ShooterDualSpeed_Max_RPS); //TODO this assumes -1 -> 1 based on joysick
      });
  }

  public Command runDualShooterSpeedIDLE() {
    return this.runOnce(
      () -> { 
        setDualShooterSpeedControl( SuperStructureConstants.runDualShooterSpeedIDLE_SPEED); // idle system
      });
  }

  public Command toggleShooterStatus() {
    return this.runOnce(
      () -> {
        shooterIsShutDown = !shooterIsShutDown;
      }
    );
    
  }

    public Command setShooterONESpeed() {
    return this.runOnce(
      () -> { 
        setShooterSpeedConstant(SuperStructureConstants.ShooterONESpeed);
      });
   }

  public Command setShooterTWOSpeed() {
    return this.runOnce(
      () -> { 
        setShooterSpeedConstant(SuperStructureConstants.ShooterTWOSpeed);
      });
   }

     public Command setShooterTHREESpeed() {
    return this.runOnce(
      () -> { 
        setShooterSpeedConstant(SuperStructureConstants.ShooterTHREESpeed);
      });
   }

     public Command setShooterFOURSpeed() {
    return this.runOnce(
      () -> { 
        setShooterSpeedConstant(SuperStructureConstants.ShooterFOURSpeed);
      });
   }

  public void setShooterSpeedConstant(double ShootSpeed) {
     currentShootRequst = ShootSpeed;
  }

  public Command runDualShooterGoalVision(DoubleSupplier distance) {
    return this.run(
      () -> { 
        shooterSpeedFromVision(distance.getAsDouble() ); // idle system
      });
  }

  //Use this method when shooting at the goal
  private void shooterSpeedFromVision(Double distance){
    // current position in rotations
    SmartDashboard.putNumber("Shooter distance", distance);

    //double hoodPositionFromHoop = (Math.pow (4.25, distance)) - .0797; // update this based on table data for hood distance
     double shooterSpeedFromDistance = distanceToGoalShooterLUT.interpolate(distance); // update this based on table data for hood distance
    double shooterSpeedRequest = shooterSpeedFromDistance;

    //if (isTargetLocked){
    
      if (shooterSpeedFromDistance < SuperStructureConstants.runDualShooterSpeedIDLE_SPEED){
        shooterSpeedRequest = SuperStructureConstants.runDualShooterSpeedIDLE_SPEED;
      }
      else if ( shooterSpeedFromDistance >= SuperStructureConstants.ShooterDualSpeed_Max_RPS){
        shooterSpeedRequest = SuperStructureConstants.ShooterDualSpeed_Max_RPS;
      }

      setDualShooterSpeedControl(shooterSpeedRequest);

      SmartDashboard.putNumber("Shooter Requested Speed", shooterSpeedRequest);
      SmartDashboard.putNumber("Shooter Requested SpeedVia Distance", shooterSpeedFromDistance);
      
     
   // }

  }
  
  public Command runShooterOn() {
    return this.runOnce(
      () -> {
        setDualShooterSpeedControl(currentShootRequst);
      }); 
    }

    public Command runShooterReverse() {
    return this.runOnce(
      () -> {
        setDualShooterSpeedControl(SuperStructureConstants.Shooter_Speed_Reverse);
      }); 
    }

  public Command runShooterOff() {
    return this.runOnce(
      () -> {
        setDualShooterSpeedControl(SuperStructureConstants.Shooter_Speed_Off);
      }); 
    }



  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("Shooter", inputs);
     
    SmartDashboard.putNumber("Shooter Speed", inputs.shooterVelocityRot); 
    SmartDashboard.putBoolean("Shooter Is Shutdown", shooterIsShutDown);   
  }
  

}
