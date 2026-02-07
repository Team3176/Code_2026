package com.team3176.robot.subsystems.superstructure.KickerControl;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.positional.Arm;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team3176.robot.constants.BaseConstants.Mode;
import com.team3176.robot.constants.BaseConstants.RobotType;
import com.team3176.robot.constants.*;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;



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

  public Command setkicker2Brake() {
    return this.runOnce(
      () -> {
        setkickerBrake();
      }); 
    }

  public Command incrementalDeploy() {
    return this.runOnce(
      () -> {
        deployIncremental();
      }
    );
  }

  public void deployIncremental() {
  }

  // USE THESE COMMANDS FOR SPEED CONTROL

  private void setkickerSpeedControl(double Speed_RPS) {
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
  }
}
