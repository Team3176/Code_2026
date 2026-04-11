package frc.robot.subsystems.superstructure.IntakeControl;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.BaseConstants.Mode;
import frc.robot.constants.BaseConstants.RobotType;
import frc.robot.constants.*;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunablePID;



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
  private enum rollerSpeedStatus {OFF, INTAKE, IDLE, REVERSE};
  private double homePos = 0;

  rollerSpeedStatus Status = rollerSpeedStatus.OFF; 


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
       //setIntakePositionVoltagePos(positionHome); 
       retractToHomePostion();
      }); 
    }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runIntakePosition(DoubleSupplier position) {
    return this.run(
      () -> { 
        setIntakePositionVoltagePosSlot0(position.getAsDouble());
      });
  }

  public Command runIntakePositionVoltageManual(DoubleSupplier volts) {
    return this.runEnd(
      () -> {
        setIntakePositionVolts(volts.getAsDouble());
      }, 
      () -> {
        setIntakePositionVolts(0.0);
      });
  }


  private void setIntakePositionVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    if (volts < 0) {
      io.setIntakeRollerVelocity(SuperStructureConstants.IntakeRollerIdleSpeed);
    }
    io.setIntakePositionVolts(volts);
  }

  //Use for Deploy 
  private void setIntakePositionVoltagePosSlot0(double position) {
    io.setIntakePositionVoltagePosSlot0(position);
  }
  //Use for Retracting
    private void setIntakePositionVoltagePosSlot1(double position) {
    io.setIntakePositionVoltagePosSlot1(position);
  }


  //Used to reset home position based on what is read from sensor currently
  public void setCurrentHomePos() {
    this.homePos = inputs.IntakePositionRot;
  }

  public Command deployFromHomeCmd() {
    return this.runOnce(
      () -> {
        deployFromHome();
      }
    );
  }

  public void deployFromHome() {
 
    double deployPos = SuperStructureConstants.Intake_Extend_POS;
    setIntakePositionVoltagePosSlot0(deployPos);
    io.setIntakeRollerVelocity(SuperStructureConstants.IntakeRollerIntakeSpeed); 
    Status = rollerSpeedStatus.INTAKE;
  }
  

  // Incremental retracting
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
    setIntakePositionVoltagePosSlot1(deployPos);
   
  }
  // Retract to Home Positiong 
  public Command retractToHome() {
    return this.runOnce(
      () -> {
        retractToHomePostion();
         
      }
    );
  }

    public void retractToHomePostion () {
      setIntakePositionVoltagePosSlot1(positionHome); 
      io.setIntakeRollerVelocity(SuperStructureConstants.IntakeRollerIdleSpeed); 
      Status = rollerSpeedStatus.IDLE;
   
  }

/// This is the area for running / commending roller speeds

  public Command runIntakeRoller(DoubleSupplier dutyCycle) {
    return this.run(
      () -> { 
        intakeRollerDutyCycle(dutyCycle.getAsDouble());
      });
  }

  public void intakeRollerDutyCycle(double dutyCycle){
     io.setIntakeRollerVelocity(dutyCycle); 
  }

  public Command runIntakeRollerReverse() {
    return this.runOnce(
      () -> { 
        intakeRollerReverse();
      });
  }

  public void intakeRollerReverse(){
     io.setIntakeRollerVelocity(SuperStructureConstants.IntakeRollerIntakeReverse); 
  }

  public Command runIntakeRollerResume() {
    return this.runOnce(
      () -> { 
        intakeRollerResume();
      });
  }

  public void intakeRollerResume(){
    if (Status == rollerSpeedStatus.IDLE){
      io.setIntakeRollerVelocity(SuperStructureConstants.IntakeRollerIdleSpeed); 
    }
    else if (Status == rollerSpeedStatus.INTAKE){
      io.setIntakeRollerVelocity(SuperStructureConstants.IntakeRollerIntakeSpeed); 
    }
    else if (Status == rollerSpeedStatus.OFF){
      io.setIntakeRollerVelocity(0); 
    }
  }

  public Command runIntakeRollerIdle() {
    return this.runOnce(
      () -> { 
        intakeRollerIdle();
      });
  }

  public void intakeRollerIdle(){
   
      io.setIntakeRollerVelocity(SuperStructureConstants.IntakeRollerIdleSpeed); 
      Status = rollerSpeedStatus.IDLE;
  }

  public Command intakeRollerMaxOverride() {
    return this.runEnd(
      () -> {
        setIntakeRollerMax();
      }, 
      () -> {
        intakeRollerResume();
      });
  }

  public void setIntakeRollerMax() {
 
    io.setIntakeRollerVelocity(SuperStructureConstants.IntakeRollerIdleSpeed); 
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
