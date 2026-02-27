package com.team3176.robot.subsystems.superstructure.IntakeControl;


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



public class IntakeControl extends SubsystemBase {
 private static IntakeControl instance;
  private final IntakeControlIO io;
  private final IntakeControlIOInputsAutoLogged inputs = new IntakeControlIOInputsAutoLogged();

  private final TunablePID positionMotorPID;
  private Timer deployTime = new Timer();
  private double positionSetpoint;
  private double position_offset = SuperStructureConstants.IntakePosition_ENCODER_OFFSET;
  private boolean ishomed = false;
  private double positionHome = SuperStructureConstants.Intake_ZERO_POS;
 
  private double homePos = 0;


  private IntakeControl(IntakeControlIO io) {
    this.io = io;
    this.positionMotorPID = new TunablePID("IntakeControlPIDConstants", SuperStructureConstants.IntakeControl_kP, SuperStructureConstants.IntakeControl_kI, SuperStructureConstants.IntakeControl_kD);
  
    this.positionHome = inputs.IntakePositionRot;
  }

  public static IntakeControl getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new IntakeControl(new IntakeControlIOTalonSpark() {});
      } else {
        instance = new IntakeControl(new IntakeControlIOSim() {});
      }
    }
    return instance;
  }



  public Command Intake2Home() {
    return this.runOnce(
      () -> {
       setIntakePositionVoltagePos(positionHome); 
      }); 
    }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runIntakePosition(DoubleSupplier position) {
    return this.run(
      () -> { 
        setIntakePositionVoltagePos(position.getAsDouble());
      });
  }

  public Command runIntakePositionVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setIntakePositionVolts(position.getAsDouble());
      }, 
      () -> {
        setIntakePositionVolts(0.0);
      });
  }


  private void setIntakePositionVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    io.setIntakePositionVolts(volts);
  }

  private void setIntakePositionVoltagePos(double position) {
    io.setIntakePositionVoltagePos(position);
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
    this.homePos = inputs.IntakePositionRot;
  }

  public void deployFromHome() {
 
    double deployPos = this.homePos + SuperStructureConstants.Intake_Extend_POS;
    setIntakePositionVoltagePos(deployPos);
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
    setIntakePositionVoltagePos(deployPos);
  }



  public Command runIntakeRoller(DoubleSupplier dutyCycle) {
    return this.run(
      () -> { 
        intakeRollerDutyCycle(dutyCycle.getAsDouble());
      });
  }

  public void intakeRollerDutyCycle(double dutyCycle){
     io.setIntakeRollerVelocity(dutyCycle); 
  }

 
  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("Intake Position", inputs);
     
    Logger.recordOutput("Intake/setpoint", this.positionSetpoint);
   
    positionMotorPID.checkParemeterUpdate();

    SmartDashboard.putNumber("Intake Pivot Position", inputs.IntakePositionRot);
    
    SmartDashboard.putNumber("Intake Roller Speed", inputs.IntakeVelocityRadPerSec);
  }
}
