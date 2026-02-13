package com.team3176.robot.subsystems.superstructure.TurretRotation;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.positional.Arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team3176.robot.constants.BaseConstants.Mode;
import com.team3176.robot.constants.BaseConstants.RobotType;
import com.team3176.robot.subsystems.leds.LEDSubsystem;
import com.team3176.robot.constants.*;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;





public class TurretRotation extends SubsystemBase {
 private static TurretRotation instance;
  private final TurretRotationIO io;
  private final TurretRotationIOInputsAutoLogged inputs = new TurretRotationIOInputsAutoLogged();

  private Timer deployTime = new Timer();
  private double positionSetpoint;
  private double position_offset = SuperStructureConstants.TurretRotation_ENCODER_OFFSET;
  private boolean ishomed = false;
  private double positionHome = SuperStructureConstants.TurretRotation_ZERO_POS;
 // private LEDSubsystem leds;
  private double homePos = 0;


  private TurretRotation(TurretRotationIO io) {
    this.io = io;
   // leds = LEDSubsystem.getInstance();
  }

  public boolean getClockwiseLimitswitch() {
    return inputs.turretClockwiselimitswitch;
  }

  public boolean getCounterClockwiseLimitswitch() {
    return inputs.turretCounterclockwiselimitswitch;
  }


  public static TurretRotation getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new TurretRotation(new TurretRotationIOTalon() {});
      } 
      else {
        instance = new TurretRotation(new TurretRotationIOSim() {});
      }
    }
    return instance;
  }



  public Command TurretRotation2Home() {
    return this.runOnce(
      () -> {
       setTurretRotationVoltagePos(positionHome); 
      }); 
    }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runTurretRotation(DoubleSupplier position) {
    return this.run(
      () -> { 
        setTurretRotationVoltagePos(position.getAsDouble());
      });
  }

  public Command runTurretRotationVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setTurretRotationVolts(position.getAsDouble());
      }, 
      () -> {
        setTurretRotationVolts(0.0);
      });
  }

   private void setTurretRotationVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    io.setTurretRotationVolts(volts);
  }

  private void setTurretRotationVoltagePos(double position) {
    io.setTurretRotationVoltagePos(position);
  }

  //Used to reset home position based on what is read from sensor currently
  public void setCurrentHomePos() {
    this.homePos = inputs.turretRotationPositionRot;
  }

  public void deployFromHome() {
    setCurrentHomePos();
    double deployPos = this.homePos + .25;
    setTurretRotationVoltagePos(deployPos);
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
    setTurretRotationVoltagePos(deployPos);
  }

  //Use this command for target tracking - 
  public Command runTurretRotationFromVision(DoubleSupplier positionErrorDegrees, BooleanSupplier isTargetLocked, LEDSubsystem leds ) {
    double positionErrorRotations =   positionErrorDegrees.getAsDouble(); // SuperStructureConstants.TurretDegreesToRotations; //convert degrees of error into rotations
    
    return this.run(
      () -> { 
        turretRotationFromVision(positionErrorDegrees.getAsDouble(), isTargetLocked.getAsBoolean(), leds);
      });
  }

  private void turretRotationFromVision(double positionErrorRotations, boolean isTargetLocked , LEDSubsystem leds){
      // curret position in rotations
      double curretPosition = inputs.turretRotationPositionRot;
      //io.setTurretRotationError( positionErrorRotations, isTargetLocked);
      if (isTargetLocked){
        if ((Math.abs(positionErrorRotations) > SuperStructureConstants.TurretErrorMoveDeadband )){
          // adjust the position based on the error identified 
          io.setTurretRotationError(curretPosition + positionErrorRotations * SuperStructureConstants.TurretRadianToRotations, isTargetLocked);
          leds.turretTracking();
        }
        else {
          leds.turretLockedOn();
        }
      }
      else {
        leds.turretVisonLost();
      }
  }
 


  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("TurretRotation", inputs);
     
    Logger.recordOutput("TurretRotation/setpoint", this.positionSetpoint);
   
   // Use Limit Switches not to break anything - May be double dipping on limit switches based on method call. - safe than sorry
/*
    if (inputs.turretClockwiselimitswitch && inputs.turretRotationAppliedVolts < 0) {
      io.setTurretRotationVoltage(0);
    }

    if (inputs.turretCounterclockwiselimitswitch && inputs.turretRotationAppliedVolts >= 0) {
      io.setTurretRotationVoltage(0);
    }
  */  
  }
}
