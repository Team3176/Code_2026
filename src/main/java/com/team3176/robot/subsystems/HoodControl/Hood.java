package com.team3176.robot.subsystems.HoodControl;


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



public class Hood extends SubsystemBase {
 private static Hood instance;
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private final TunablePID positionMotorPID;
  private Timer deployTime = new Timer();
  private double positionSetpoint;
  private double position_offset = SuperStructureConstants.Hood_ENCODER_OFFSET;
  private boolean ishomed = false;
  private double positionHome = SuperStructureConstants.Hood_ZERO_POS;
 
  private double homePos = 0;


  private Hood(HoodIO io) {
    this.io = io;
    this.positionMotorPID = new TunablePID("HoodPIDConstants", SuperStructureConstants.Hood_kP, SuperStructureConstants.Hood_kI, SuperStructureConstants.Hood_kD);
  
    this.positionHome = inputs.HoodPositionRot;



  }




  public static Hood getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Hood(new HoodIOTalon() {});
      } else {
        instance = new Hood(new HoodIOSim() {});
      }
    }
    return instance;
  }



  public Command Hood2Home() {
    return this.runOnce(
      () -> {
       setHoodVoltagePos(positionHome); 
      }); 
    }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runHood(DoubleSupplier position) {
    return this.run(
      () -> { 
        setHoodVoltagePos(position.getAsDouble()*3);
      });
  }

  public Command runHoodVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setHoodVolts(position.getAsDouble());
      }, 
      () -> {
        setHoodVolts(0.0);
      });
  }


  private void setHoodVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    io.setHoodVolts(volts);
  }

  private void setHoodVoltagePos(double position) {
    io.setHoodVoltagePos(position);
  }

    public void setHoodCoast() {
    io.setHoodBrakeMode(false);
  }

  public void setHoodBrake() {
    io.setHoodBrakeMode(true);
  }

  
  public Command setHood2Coast() {
    return this.runOnce(
      () -> {
        setHoodCoast();
      }); 
    }

  public Command setHood2Brake() {
    return this.runOnce(
      () -> {
        setHoodBrake();
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
    this.homePos = inputs.HoodPositionRot;
  }

  public void deployFromHome() {
    setCurrentHomePos();
    double deployPos = this.homePos + .25;
    setHoodVoltagePos(deployPos);
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
    setHoodVoltagePos(deployPos);
  }

  public Command incrementalDeploy() {
    return this.runOnce(
      () -> {
        deployIncremental();
      }
    );
  }

  public void deployIncremental() {
    double currentPos = inputs.HoodPositionRot;
    currentPos = currentPos + 0.25;
    setHoodVoltagePos(currentPos);
  }

 
  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("Hood", inputs);
     
    Logger.recordOutput("Hood/setpoint", this.positionSetpoint);
   
    positionMotorPID.checkParemeterUpdate();
    
  }
}
