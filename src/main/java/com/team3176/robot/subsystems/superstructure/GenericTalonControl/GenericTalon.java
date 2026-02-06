package com.team3176.robot.subsystems.superstructure.GenericTalonControl;


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



public class GenericTalon extends SubsystemBase {
 private static GenericTalon instance;
  private final GenericTalonIO io;
  private final GenericTalonIOInputsAutoLogged inputs = new GenericTalonIOInputsAutoLogged();

  private final TunablePID positionMotorPID;
  private Timer deployTime = new Timer();
  private double positionSetpoint;
  private double position_offset = SuperStructureConstants.GenericTalon_ENCODER_OFFSET;
  private boolean ishomed = false;
  private double positionHome = SuperStructureConstants.GenericTalon_ZERO_POS;
 
  private double homePos = 0;


  private GenericTalon(GenericTalonIO io) {
    this.io = io;
    this.positionMotorPID = new TunablePID("GenericTalonPIDConstants", SuperStructureConstants.GenericTalon_kP, SuperStructureConstants.GenericTalon_kI, SuperStructureConstants.GenericTalon_kD);
  
    this.positionHome = inputs.genericTalonPositionRot;



  }




  public static GenericTalon getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new GenericTalon(new GenericTalonIOTalon() {});
      } else {
        instance = new GenericTalon(new GenericTalonIOSim() {});
      }
    }
    return instance;
  }



  public Command genericTalon2Home() {
    return this.runOnce(
      () -> {
       setGenericTalonVoltagePos(positionHome); 
      }); 
    }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runGenericTalon(DoubleSupplier position) {
    return this.run(
      () -> { 
        setGenericTalonVoltagePos(position.getAsDouble()*3);
      });
  }

  public Command runGenericTalonVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setGenericTalonVolts(position.getAsDouble());
      }, 
      () -> {
        setGenericTalonVolts(0.0);
      });
  }


  private void setGenericTalonVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    io.setGenericTalonVolts(volts);
  }

  private void setGenericTalonVoltagePos(double position) {
    io.setGenericTalonVoltagePos(position);
  }

    public void setGenericTalonCoast() {
    io.setGenericTalonBrakeMode(false);
  }

  public void setGenericTalonBrake() {
    io.setGenericTalonBrakeMode(true);
  }

  
  public Command setGenericTalon2Coast() {
    return this.runOnce(
      () -> {
        setGenericTalonCoast();
      }); 
    }

  public Command setGenericTalon2Brake() {
    return this.runOnce(
      () -> {
        setGenericTalonBrake();
      }); 
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
    this.homePos = inputs.genericTalonPositionRot;
  }

  public void deployFromHome() {
    setCurrentHomePos();
    double deployPos = this.homePos + .25;
    setGenericTalonVoltagePos(deployPos);
  }
  
  public Command retractTowardHome() {
    return this.runOnce(
      () -> {
        retractTowardHomePostion();
      }
    );
  }

  public void retractTowardHomePostion () {
    setCurrentHomePos();
    double deployPos = this.homePos - .10;
    setGenericTalonVoltagePos(deployPos);
  }

  public Command incrementalDeploy() {
    return this.runOnce(
      () -> {
        deployIncremental();
      }
    );
  }

  public void deployIncremental() {
    double currentPos = inputs.genericTalonPositionRot;
    currentPos = currentPos + 0.25;
    setGenericTalonVoltagePos(currentPos);
  }

  // USE THESE COMMANDS FOR SPEED CONTROL

  private void setGenericTalonSpeedControl(double Speed_RPS) {
    io.setGenericTalonSpeedVelocity(Speed_RPS);
  }

  public void setGenericTalonSpeedCoast() {
    io.setGenericTalonSpeedBrakeMode(false);
  }

  public void setGenericTalonSpeedBrake() {
    io.setGenericTalonSpeedBrakeMode(true);
  }

  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runGenericTalonSpeed(DoubleSupplier Speed_RPS) {
    return this.run(
      () -> { 
        setGenericTalonSpeedControl(Speed_RPS.getAsDouble() * SuperStructureConstants.GenericTalonSpeed_Max_RPS); //TODO this assumes -1 -> based on joysick
      });
  }

  // USE THESE COMMANDS FOR Dual Motor SPEED CONTROL

  private void setGenericTalonDualSpeedControl(double Speed_RPS) {
    io.setGenericTalonDualSpeedVelocity(Speed_RPS);
  }

  public void setGenericTalonDualSpeedCoast() {
    io.setGenericTalonDualSpeedBrakeMode(false);
  }

  public void setGenericTalonDualSpeedBrake() {
    io.setGenericTalonDualSpeedBrakeMode(true);
  }

  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runGenericTalonDualSpeed(DoubleSupplier Speed_RPS) {
    return this.run(
      () -> { 
        setGenericTalonDualSpeedControl(Speed_RPS.getAsDouble() * SuperStructureConstants.GenericTalonDualSpeed_Max_RPS); //TODO this assumes -1 -> based on joysick
      });
  }



  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("GenericTalon", inputs);
     
    Logger.recordOutput("GenericTalon/setpoint", this.positionSetpoint);
   
    positionMotorPID.checkParemeterUpdate();
    
  }
}
