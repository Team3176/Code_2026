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



public class Shooter extends SubsystemBase {
 private static Shooter instance;
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final TunablePID positionMotorPID;
  private Timer deployTime = new Timer();
  private double positionSetpoint;
  private double position_offset = SuperStructureConstants.Shooter_ENCODER_OFFSET;
  private boolean ishomed = false;
  private double positionHome = SuperStructureConstants.Shooter_ZERO_POS;
 
  private double homePos = 0;


  private Shooter(ShooterIO io) {
    this.io = io;
    this.positionMotorPID = new TunablePID("ShooterPIDConstants", SuperStructureConstants.Shooter_kP, SuperStructureConstants.Shooter_kI, SuperStructureConstants.Shooter_kD);
  
    this.positionHome = inputs.ShooterPositionRot;



  }




  public static Shooter getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Shooter(new ShooterIOTalon() {});
      } else {
        instance = new Shooter(new ShooterIOSim() {});
      }
    }
    return instance;
  }



  public Command Shooter2Home() {
    return this.runOnce(
      () -> {
       setShooterVoltagePos(positionHome); 
      }); 
    }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runShooter(DoubleSupplier position) {
    return this.run(
      () -> { 
        setShooterVoltagePos(position.getAsDouble()*3);
      });
  }

  public Command runShooterVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setShooterVolts(position.getAsDouble());
      }, 
      () -> {
        setShooterVolts(0.0);
      });
  }


  private void setShooterVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    io.setShooterVolts(volts);
  }

  private void setShooterVoltagePos(double position) {
    io.setShooterVoltagePos(position);
  }

    public void setShooterCoast() {
    io.setShooterBrakeMode(false);
  }

  public void setShooterBrake() {
    io.setShooterBrakeMode(true);
  }

  
  public Command setShooter2Coast() {
    return this.runOnce(
      () -> {
        setShooterCoast();
      }); 
    }

  public Command setShooter2Brake() {
    return this.runOnce(
      () -> {
        setShooterBrake();
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
    this.homePos = inputs.ShooterPositionRot;
  }

  public void deployFromHome() {
    setCurrentHomePos();
    double deployPos = this.homePos + .25;
    setShooterVoltagePos(deployPos);
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
    setShooterVoltagePos(deployPos);
  }

  public Command incrementalDeploy() {
    return this.runOnce(
      () -> {
        deployIncremental();
      }
    );
  }

  public void deployIncremental() {
    double currentPos = inputs.ShooterPositionRot;
    currentPos = currentPos + 0.25;
    setShooterVoltagePos(currentPos);
  }

  // USE THESE COMMANDS FOR SPEED CONTROL

  private void setShooterSpeedControl(double Speed_RPS) {
    io.setShooterSpeedVelocity(Speed_RPS);
  }

  public void setShooterSpeedCoast() {
    io.setShooterSpeedBrakeMode(false);
  }

  public void setShooterSpeedBrake() {
    io.setShooterSpeedBrakeMode(true);
  }

  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runShooterSpeed(DoubleSupplier Speed_RPS) {
    return this.run(
      () -> { 
        setShooterSpeedControl(Speed_RPS.getAsDouble() * SuperStructureConstants.ShooterSpeed_Max_RPS); //TODO this assumes -1 -> based on joysick
      });
  }


  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("Shooter", inputs);
     
    Logger.recordOutput("Shooter/setpoint", this.positionSetpoint);
   
    positionMotorPID.checkParemeterUpdate();
    
  }
}
