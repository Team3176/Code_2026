package frc.robot.subsystems.superstructure.KickerControl;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.BaseConstants.Mode;
import frc.robot.constants.BaseConstants.RobotType;
import frc.robot.subsystems.superstructure.Superstructure;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import frc.robot.constants.*;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunablePID;



public class Kicker extends SubsystemBase {
 private static Kicker instance;
  private final KickerIO io;
  private final kickerIOInputsAutoLogged inputs = new kickerIOInputsAutoLogged();


  private Timer deployTime = new Timer();
 


  private Kicker(KickerIO io) {
    this.io = io;

  }




  public static Kicker getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Kicker(new KickerIOTalon() {});
      } else {
        instance = new Kicker(new KickerIOSim() {});
      }
    }
    return instance;
  }

 

 

    public void setkickerCoast() {
    io.setkickerBrakeMode(false);
  }

  public void setkickerBrake() {
    io.setkickerBrakeMode(true);
  }

  
  public Command setkicker2Coast() {
    return this.runOnce(
      () -> {
        setkickerCoast();
      }); 
  }  
  
  public Command runkickerOn() {
    return this.runOnce(
      () -> {
        setKickerComand(SuperStructureConstants.Kicker_Speed_On);
      }); 
    }

    public Command runkickerReverse() {
    return this.runOnce(
      () -> {
        setKickerComand(SuperStructureConstants.Kicker_Speed_Reverse);
      }); 
    }

  public Command runkickerOff() {
    return this.runOnce(
      () -> {
        setKickerComand(SuperStructureConstants.Kicker_Speed_Off);
      }); 
    }

  public Command setkicker2Brake() {
    return this.runOnce(
      () -> {
        setkickerBrake();
      }); 
    }



  // USE THESE COMMANDS FOR SPEED CONTROL

  private void setkickerSpeedControl(double Speed_RPS) {
    io.setkickerSpeedVelocity(Speed_RPS);
  }

  private void setKickerComand(double Speed_RPS) {
    io.setkickerSpeedVelocity(Speed_RPS);
  }

  public void setkickerSpeedCoast() {
    io.setkickerSpeedBrakeMode(false);
  }

  public void setkickerSpeedBrake() {
    io.setkickerSpeedBrakeMode(true);
  }
  
 
  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runkickerSpeed(DoubleSupplier Speed_RPS) {
    return this.run(
      () -> { 
        setkickerSpeedControl(Speed_RPS.getAsDouble() * SuperStructureConstants.KickerSpeed_Max_RPS); //TODO this assumes -1 -> based on joysick
      });
  }


  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("kicker", inputs);

  //  SmartDashboard.putNumber("Kicker Speed", inputs.kickerVelocityRadPerSec);
  }
}
