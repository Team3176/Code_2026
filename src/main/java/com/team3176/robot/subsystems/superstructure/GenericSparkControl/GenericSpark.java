package com.team3176.robot.subsystems.superstructure.GenericSparkControl;


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



public class GenericSpark extends SubsystemBase {
 private static GenericSpark instance;
  private final GenericSparkIO io;
  private final GenericSparkIOInputsAutoLogged inputs = new GenericSparkIOInputsAutoLogged();


  private Timer deployTime = new Timer();
  private double position_offset = SuperStructureConstants.GenericSpark_ENCODER_OFFSET;
  private boolean ishomed = false;
  private double positionHome = SuperStructureConstants.GenericSpark_ZERO_POS;
 
  private double homePos = 0;


  private GenericSpark(GenericSparkIO io) {
    this.io = io;
    
  }




  public static GenericSpark getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new GenericSpark(new GenericSparkIOSpark() {});
      } else {
        instance = new GenericSpark(new GenericSparkIOSim() {});
      }
    }
    return instance;
  }



  public Command genericSpark2Home() {
    return this.runOnce(
      () -> {
       setGenericSparkPos(positionHome); 
      }); 
    }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runGenericSpark(DoubleSupplier position) {
    return this.run(
      () -> { 
        setGenericSparkPos(position.getAsDouble());
      });
  }

  public Command runGenericSparkVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setGenericSparkVolts(position.getAsDouble());
      }, 
      () -> {
        setGenericSparkVolts(0.0);
      });
  }


  private void setGenericSparkVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    io.setGenericSparkVolts(volts);
  }

  private void setGenericSparkPos(double position) {
    io.setGenericSparkPosition(position);
  }



  // USE THESE COMMANDS FOR SPEED CONTROL

  private void setGenericSparkSpeedControl(double Speed_RPM) {
    io.setGenericSparkSpeedVelocity(Speed_RPM);
  }



  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runGenericSparkSpeed(DoubleSupplier Speed_RPM) {
    return this.run(
      () -> { 
        setGenericSparkSpeedControl(Speed_RPM.getAsDouble() * SuperStructureConstants.GenericSparkSpeed_Max_RPM); //TODO this assumes -1 -> based on joysick
      });
  }

  // USE THESE COMMANDS FOR Dual Motor SPEED CONTROL

  private void setGenericSparkDualSpeedControl(double Speed_RPM) {
    io.setGenericSparkDualSpeedVelocity(Speed_RPM);
  }


  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runGenericSparkDualSpeed(DoubleSupplier Speed_RPM) {
    return this.run(
      () -> { 
        setGenericSparkDualSpeedControl(Speed_RPM.getAsDouble() * SuperStructureConstants.GenericTalonDualSpeed_Max_RPS); //TODO this assumes -1 -> based on joysick
      });
  }



  @Override
  public void periodic() {
    
    //io.updateInputs(inputs);

    //Logger.processInputs("GenericSpark", inputs);
     
    //Logger.recordOutput("GenericSpark/setpoint", this.positionSetpoint);
   
    //positionMotorPID.checkParemeterUpdate();
    
  }
}
