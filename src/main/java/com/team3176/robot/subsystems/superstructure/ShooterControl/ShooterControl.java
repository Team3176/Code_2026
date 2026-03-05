package com.team3176.robot.subsystems.superstructure.ShooterControl;


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

import com.team3176.robot.constants.BaseConstants.Mode;
import com.team3176.robot.constants.BaseConstants.RobotType;
import com.team3176.robot.mathUtil.LinearInterpolationTable;
import com.team3176.robot.constants.*;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;



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
    if( shooterIsShutDown == true) {
       io.setDualShooterSpeedVelocity(0);
    } else {
      io.setDualShooterSpeedVelocity(Speed_RPS);
    }
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
    return this.run(
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
    return this.run(
      () -> { 
        setShooterSpeedConstant(SuperStructureConstants.ShooterONESpeed);
      });
   }

  public Command setShooterTWOSpeed() {
    return this.run(
      () -> { 
        setShooterSpeedConstant(SuperStructureConstants.ShooterTWOSpeed);
      });
   }

     public Command setShooterTHREESpeed() {
    return this.run(
      () -> { 
        setShooterSpeedConstant(SuperStructureConstants.ShooterTHREESpeed);
      });
   }

     public Command setShooterFOURSpeed() {
    return this.run(
      () -> { 
        setShooterSpeedConstant(SuperStructureConstants.ShooterFOURSpeed);
      });
   }

  public void setShooterSpeedConstant(double ShootSpeed) {
     currentShootRequst = ShootSpeed;
  }

  public Command runDualShooterGoalVision(DoubleSupplier distance, BooleanSupplier isTargetLocked) {
    return this.run(
      () -> { 
        shooterSpeedFromVision(distance.getAsDouble(), isTargetLocked.getAsBoolean() ); // idle system
      });
  }

  //Use this method when shooting at the goal
  private void shooterSpeedFromVision(Double distance, Boolean isTargetLocked){
    // current position in rotations
    double currentSpeed = inputs.shooterVelocityRadPerSec;
    //double hoodPositionFromHoop = (Math.pow (4.25, distance)) - .0797; // update this based on table data for hood distance
    double shooterSpeedFromDistance = distanceToGoalShooterLUT.interpolate(distance); // update this based on table data for hood distance
    double shooterSpeedRequest = currentSpeed;

    if (isTargetLocked){
    
      if (shooterIsShutDown || shooterSpeedFromDistance < SuperStructureConstants.runDualShooterSpeedIDLE_SPEED){
        shooterSpeedRequest = SuperStructureConstants.runDualShooterSpeedIDLE_SPEED;
      }
      else if ( shooterSpeedFromDistance >= SuperStructureConstants.ShooterDualSpeed_Max_RPS){
        shooterSpeedRequest = SuperStructureConstants.ShooterDualSpeed_Max_RPS;
      }

      setDualShooterSpeedControl(shooterSpeedRequest);
    
    }

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
  }
  

}
