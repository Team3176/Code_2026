package frc.robot.subsystems.superstructure.ClimbControl;


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
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.constants.*;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunablePID;



public class ClimbControl extends SubsystemBase {
 private static ClimbControl instance;
private final ClimbControlIO io;
private final ClimbControlIOInputsAutoLogged inputs = new ClimbControlIOInputsAutoLogged();

  private final TunablePID positionMotorPID;
  private Timer deployTime = new Timer();
  private double positionSetpoint;
  private double position_offset = SuperStructureConstants.Climb_ENCODER_OFFSET;
  private boolean ishomed = false;
  private double positionHomeLeft = SuperStructureConstants.Climb_ZERO_POS;
  private double positionHomeRight = SuperStructureConstants.Climb_ZERO_POS;
 
  private double homePos = 0;


  private ClimbControl(ClimbControlIO io) {
    this.io = io;
    this.positionMotorPID = new TunablePID("ClimbControlPIDConstants", SuperStructureConstants.Climb_kP, SuperStructureConstants.Climb_kI, SuperStructureConstants.Climb_kD);
  
    this.positionHomeRight = inputs.ClimbPositionRotRight;
    this.positionHomeLeft = inputs.ClimbPositionRotLeft;

  }




  public static ClimbControl getInstance() {
    if (instance == null) {
      if (BaseConstants.getMode() == Mode.REAL && BaseConstants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new ClimbControl(new ClimbControlIOTalon() {});
      } else {
        instance = new ClimbControl(new ClimbControlIOSim() {});
      }
    }
    return instance;
  }




  public Command Climb2Home() {
    return this.runOnce(
      () -> {
       setClimbVoltagePos(positionHomeLeft); 
    });
  }

  public Command Climb2Extend() {
    return this.runOnce(
      () -> {
       setClimbVoltagePos(SuperStructureConstants.ClimbMaxExtend); 
      }); 
    }

  //Provide a position suggest scaling from a joy stick or similar to get the desired number of rotations
  public Command runClimb(DoubleSupplier position) {
    return this.run(
      () -> { 
        setClimbVoltagePos(position.getAsDouble());
      });
  }

  public Command runClimbUp(DoubleSupplier position) {
    return this.run(
      () -> { 
        setClimbVoltagePos(position.getAsDouble());
      });
  }
    public Command runClimbDown(DoubleSupplier position) {
    return this.run(
      () -> { 
        setClimbVoltagePos(position.getAsDouble());
      });
  }

  public Command runClimbVoltageManual(DoubleSupplier position) {
    return this.runEnd(
      () -> {
        setClimbVolts(-position.getAsDouble());
      }, 
      () -> {
        setClimbVolts(0.0);
      });
  }


  private void setClimbVolts(double volts) {
    // this assumes positive voltage deploys and negative voltage retracts.
    // invert the motor if that is NOT true
    io.setClimbVolts(volts);
  }

  private void setClimbVoltagePos(double position) {
    io.setClimbBothPos(position);
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
    this.homePos = inputs.ClimbPositionRotLeft;
    
  }

  public void deployFromHome() {
    setCurrentHomePos();
    double deployPos = this.homePos + SuperStructureConstants.ClimbUpIncrement;
    setClimbVoltagePos(deployPos);
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
    double deployPos = this.homePos - SuperStructureConstants.ClimbDownIncrement;
    setClimbVoltagePos(deployPos);
  }

  public Command incrementalDeploy() {
    return this.runOnce(
      () -> {
        deployIncremental();
      }
    );
  }

  public void deployIncremental() {
    double currentPos = inputs.ClimbPositionRotLeft;
    currentPos = currentPos + 0.25;
    setClimbVoltagePos(currentPos);
  }

  public Command moveLeftRightPosition(DoubleSupplier deltaLeft, DoubleSupplier deltaRight) {
    return this.run(
        () -> {
          deltaLeftRightClimb(5 * deltaRight.getAsDouble(), 5* deltaLeft.getAsDouble());
          
        });
/*         () -> {
          io.setRightVoltage(0.0);
          io.setLeftVoltage(0.0);
        }); */
  }

  public void deltaLeftRightClimb(double deltaLeft, double deltaRight) {
    SmartDashboard.putNumber("climb volt command left", deltaLeft);
    SmartDashboard.putNumber("climb volt command right", deltaRight);

     if((inputs.ClimbPositionRotLeft >= SuperStructureConstants.ClimbMaxExtend && deltaLeft < 0) ){
      io.setLeftVoltage(0.0);
    }
    else{
      io.setLeftVoltage(-deltaLeft);
    }
     if((inputs.ClimbPositionRotRight >= SuperStructureConstants.ClimbMaxExtend && deltaRight < 0 )){
        io.setRightVoltage(0);
    }
    else{
      io.setRightVoltage(-deltaRight);
    } 
       
    //  io.setLeftVoltage(-deltaLeft);

    //  io.setRightVoltage(-deltaRight);
  
  }

  public Command stopLeftRight() {
    return this.runOnce(
        () -> {
          io.setLeftVoltage(0.0);
          io.setRightVoltage(0.0);
        });
  }

 
  @Override
  public void periodic() {
    
    io.updateInputs(inputs);

    Logger.processInputs("Climb", inputs);
     
    Logger.recordOutput("Climb/setpoint", this.positionSetpoint);
   
    positionMotorPID.checkParemeterUpdate();
    
    SmartDashboard.putNumber("Climb Position Right", inputs.ClimbPositionRotRight);
    SmartDashboard.putNumber("Climb Position Left", inputs.ClimbPositionRotLeft);


  }
}
