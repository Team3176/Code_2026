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
  private final LoggedTunableNumber L0TuneSetpoint, L1TuneSetpoint, L2TuneSetpoint, L3TuneSetpoint, L4TuneSetpoint;
  private final TunablePID positionMotorPID;
  private Timer deployTime = new Timer();
  private double positionSetpoint;
  private double position_offset = 0;
  private boolean ishomed = false;
  private double positionHome = SuperStructureConstants.GenericTalon_L0_POS;
  private double L0Setpoint, L1Setpoint, L2Setpoint, L3Setpoint, L4Setpoint;
  private double homePos = 0;
  public enum POS {
    L0,
    L1,
    L2,
    L3,
    L4,
  }
  public POS currentPosTrack = POS.L0;

  private enum positionStates {
    DEPLOY,
    RETRACT,
    IDLE,
    HOLD,
  };
  

  private positionStates positionState = positionStates.HOLD;
  // DigitalInput linebreak1 = new DigitalInput(Hardwaremap.ArmRollerLinebreak_DIO);

  private GenericTalon(GenericTalonIO io) {
    this.io = io;
    this.positionMotorPID = new TunablePID("ArmPivot", 3.0, 0.0, 0.0);
    this.L0TuneSetpoint = new LoggedTunableNumber("GenericTalon/L0Setpoint", SuperStructureConstants.GenericTalon_L0_POS);
    this.L1TuneSetpoint = new LoggedTunableNumber("GenericTalon/L1Setpoint", SuperStructureConstants.GenericTalon_L1_POS);
    this.L2TuneSetpoint = new LoggedTunableNumber("GenericTalon/L2Setpoint", SuperStructureConstants.GenericTalon_L2_POS);
    this.L3TuneSetpoint = new LoggedTunableNumber("GenericTalon/L3Setpoint", SuperStructureConstants.GenericTalon_L3_POS);
    this.L4TuneSetpoint = new LoggedTunableNumber("GenericTalon/L4Setpoint", SuperStructureConstants.GenericTalon_L4_POS);
    this.positionHome = inputs.genericTalonPositionRot;


    L0Setpoint = SuperStructureConstants.GenericTalon_L0_POS;
    L1Setpoint = SuperStructureConstants.GenericTalon_L1_POS;
    L2Setpoint = SuperStructureConstants.GenericTalon_L2_POS;
    L3Setpoint = SuperStructureConstants.GenericTalon_L3_POS;
    L4Setpoint = SuperStructureConstants.GenericTalon_L4_POS;
  }

  public Command setPosTrack(POS pos){
    return this.runOnce(() -> {
      currentPosTrack = pos;
    });
  }

  private void runGenericTalon(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts it.
    // invert the motor if that is NOT true
    io.setGenericTalonVolts(volts);
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



  // Example command to show how to set the position state
  public Command toGenericTalon(double position_input) {
    return this.runOnce(
        () -> {
          this.positionSetpoint = position_input;
          deployTime.restart();
        });
  }

  public Command genericTalon2Home() {
    return this.runOnce(
      () -> {
       setGenericTalonVoltagePos(positionHome); 
      }); 
    }

  // TODO: might need to deploy the Arm during a spit but maybe not

  public Command runGenericTalon(DoubleSupplier position) {
    return this.run(
      () -> { 
        setGenericTalonVoltagePos(position.getAsDouble());
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

  public void setGenericTalonCurrent() {
    io.setGenericTalonCurrent(5);
  }

  public Command setGenericTalonCurrents() {
    return this.run(() -> {setGenericTalonCurrent();});
  }
  
  public Command setPivot2Coast() {
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

  public Command incrementalDeAlgae() {
    return this.runOnce(
      () -> {
        deAlgaeIncremental();
      }
    );
  }

  public void deAlgaeIncremental() {
    double currentPos = inputs.genericTalonPositionRot;
    currentPos = currentPos + 0.25;
    setGenericTalonVoltagePos(currentPos);
  }

  @Override
  public void periodic() {
    
    io.updateInputs(inputs);
    if (L0TuneSetpoint.hasChanged(hashCode())) {
      L0Setpoint = L0TuneSetpoint.get();
    }
    if (L1TuneSetpoint.hasChanged(hashCode())) {
      L1Setpoint = L1TuneSetpoint.get();
    }
    if (L2TuneSetpoint.hasChanged(hashCode())) {
      L2Setpoint = L2TuneSetpoint.get();
    }
    if (L3TuneSetpoint.hasChanged(hashCode())) {
      L3Setpoint = L3TuneSetpoint.get();
    }
    if (L4TuneSetpoint.hasChanged(hashCode())) {
      L4Setpoint = L4TuneSetpoint.get();
    }


    Logger.processInputs("GenericTalon", inputs);
    Logger.recordOutput("GenericTalon/state", positionState);

   

    
    Logger.recordOutput("GenericTalon/setpoint", this.positionSetpoint);
   
    positionMotorPID.checkParemeterUpdate();
    
  }
}
