package com.team3176.robot.subsystems.superstructure.intake;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import yams.mechanisms.positional.Arm;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team3176.robot.subsystems.superstructure.intake.Intake;
import com.team3176.robot.subsystems.superstructure.intake.IntakeIOSim;
import com.team3176.robot.subsystems.superstructure.intake.IntakeIOTalon;
import com.team3176.robot.subsystems.superstructure.intake.IntakeIO.IntakeIOInputs;

import com.team3176.robot.constants.BaseConstants.Mode;
import com.team3176.robot.constants.BaseConstants.RobotType;
import com.team3176.robot.constants.*;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;


public class Intake extends SubsystemBase {
  private static Intake instance;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final TunablePID positionMotorPID;
  private Timer deployTime = new Timer();
  private double positionSetpoint;
  private double position_offset = SuperStructureConstants.GenericTalon_ENCODER_OFFSET;
  private boolean ishomed = false;
  private double positionHome = SuperStructureConstants.GenericTalon_ZERO_POS;
 
  private double homePos = 0;

  private Intake(IntakeIO io) {
    this.io = io;
    this.positionMotorPID = new TunablePID("IntakePIDConstants", SuperStructureConstants.GenericTalon_kP, SuperStructureConstants.GenericTalon_kI, SuperStructureConstants.GenericTalon_kD);
  
    this.positionHome = inputs.intakePositionRot;

  }

  public static Intake getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Intake(new IntakeIOTalon() {});
      } else {
        instance = new Intake(new IntakeIOSim() {});
      }
    }
    return instance;
  }

  public Command intake2Home() {
    return this.runOnce(
      () -> {
       setIntakeVoltagePos(positionHome); 
      }); 
    }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runIntake(DoubleSupplier position) {
    return this.run(
      () -> { 
        setIntakeVoltagePos(position.getAsDouble());
      });
  }

  public Command runIntakeVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setIntakeVolts(position.getAsDouble());
      }, 
      () -> {
        setIntakeVolts(0.0);
      });
  }

  private void setIntakeVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    io.setIntakeVolts(volts);
  }

  private void setIntakeVoltagePos(double position) {
    io.setIntakeVoltagePos(position);
  }

    public void setIntakeCoast() {
    io.setIntakeBrakeMode(false);
  }

  public void setIntakeBrake() {
    io.setIntakeBrakeMode(true);
  }

   public Command setIntake2Coast() {
    return this.runOnce(
      () -> {
        setIntakeCoast();
      }); 
    }

  public Command setIntake2Brake() {
    return this.runOnce(
      () -> {
        setIntakeBrake();
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
    this.homePos = inputs.intakePositionRot;
  }

  public void deployFromHome() {
    setCurrentHomePos();
    double deployPos = this.homePos + .25;
    setIntakeVoltagePos(deployPos);
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
    setIntakeVoltagePos(deployPos);
  }

  public Command incrementalDeploy() {
    return this.runOnce(
      () -> {
        deployIncremental();
      }
    );
  }

  public void deployIncremental() {
    double currentPos = inputs.intakePositionRot;
    currentPos = currentPos + 0.25;
    setIntakeVoltagePos(currentPos);
  }

// USE THESE COMMANDS FOR SPEED CONTROL

  private void setIntakeRollerControl(double Speed_RPS) {
    io.setIntakeRollerVelocity(Speed_RPS);
  }

  public void setIntakeRollerCoast() {
    io.setIntakeRollerBrakeMode(false);
  }

  public void setIntakeRollerBrake() {
    io.setIntakeRollerBrakeMode(true);
  }

   //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runIntakeRoller(DoubleSupplier Speed_RPS) {
    return this.run(
      () -> { 
        setIntakeRollerControl(Speed_RPS.getAsDouble() * SuperStructureConstants.GenericTalonSpeed_Max_RPS); //TODO this assumes -1 -> based on joysick
      });
  }

   @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);
     
    Logger.recordOutput("Intake/setpoint", this.positionSetpoint);
   
    positionMotorPID.checkParemeterUpdate();
    
  }
}
