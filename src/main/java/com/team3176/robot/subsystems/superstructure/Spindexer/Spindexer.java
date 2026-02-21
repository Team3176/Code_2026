package com.team3176.robot.subsystems.superstructure.Spindexer;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.positional.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team3176.robot.constants.BaseConstants.Mode;
import com.team3176.robot.constants.BaseConstants.RobotType;
import com.team3176.robot.constants.*;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;



public class Spindexer extends SubsystemBase {
 private static Spindexer instance;
  private final SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  private final TunablePID positionMotorPID;
  private Timer deployTime = new Timer();
  private double positionSetpoint;
  private double position_offset = SuperStructureConstants.Spindexer_ENCODER_OFFSET;
  private boolean ishomed = false;
  private double positionHome = SuperStructureConstants.Spindexer_ZERO_POS;
 
  private double homePos = 0;


  private Spindexer(SpindexerIO io) {
    this.io = io;
    this.positionMotorPID = new TunablePID("SpindexerPIDConstants", SuperStructureConstants.Spindexer_kP, SuperStructureConstants.Spindexer_kI, SuperStructureConstants.Spindexer_kD);
  
    this.positionHome = inputs.SpindexerPositionRot;



  }




  public static Spindexer getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Spindexer(new SpindexerIOTalon() {});
      } else {
        instance = new Spindexer(new SpindexerIOSim() {});
      }
    }
    return instance;
  }





  public Command runSpindexerVoltageManual(DoubleSupplier volts) {
    return this.runEnd(
      () -> {
        setSpindexerVolts(volts.getAsDouble());
      }, 
      () -> {
        setSpindexerVolts(0.0);
      });
  }


  private void setSpindexerVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    io.setSpindexerVolts(volts);
  }


  // USE THESE COMMANDS FOR SPEED CONTROL

  private void setSpindexerSpeedControl(double Speed_RPS) {
    io.setSpindexerSpeedVelocity(Speed_RPS);
  }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runSpindexerSpeed(DoubleSupplier Speed_RPS) {
    return this.run(
      () -> { 
        setSpindexerSpeedControl(Speed_RPS.getAsDouble() * SuperStructureConstants.SpindexerSpeed_Max_RPS); //TODO this assumes -1 -> based on joysick
      });
  }


  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("Spindexer", inputs);
     
    Logger.recordOutput("Spindexer/setpoint", this.positionSetpoint);
   
    positionMotorPID.checkParemeterUpdate();

    SmartDashboard.putNumber("Spindexer Speed", inputs.SpindexerVelocity);
  }
}
