package com.team3176.robot.subsystems.superstructure.ShooterControl;


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



public class ShooterControl extends SubsystemBase {
 private static ShooterControl instance;
  private final ShooterControlIO io;
  private final ShooterControlIOInputsAutoLogged inputs = new ShooterControlIOInputsAutoLogged();


  private Timer deployTime = new Timer();

 



  private ShooterControl(ShooterControlIO io) {
    this.io = io;
    



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
        setDualShooterSpeedControl((Speed_RPS.getAsDouble() +1 ) * SuperStructureConstants.ShooterDualSpeed_Max_RPS); //TODO this assumes -1 -> based on joysick
      });
  }



  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("Shooter", inputs);
     
        
  }
}
