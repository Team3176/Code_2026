package com.team3176.robot.subsystems.superstructure.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.team3176.robot.constants.BaseConstants.RobotType;
import com.team3176.robot.subsystems.superstructure.intake.IntakeIO.IntakeIOInputs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import com.team3176.robot.Constants;
import com.team3176.robot.Constants.Mode;
import com.team3176.robot.Constants.RobotType;
import com.team3176.robot.constants.*;

/*import team3176.robot.Constants;
 import team3176.robot.Constants.Mode;
import team3176.robot.Constants.RobotType;
import team3176.robot.constants.*;
import team3176.robot.util.LoggedTunableNumber;
import team3176.robot.util.TunablePID; */


public class Intake extends SubsystemBase {
   private static Intake instance;
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final TalonFXConfiguration configs = new TalonFXConfiguration();
  public Intake(IntakeIO io) {
    this.io = io;
  }

  private void runintake(double volts) {
    // this assumes positive voltage deploys the intake and negative voltage retracts it.
    // invert the motor if that is NOT true
    io.setPivotVolts(volts);
  }

  private void intakeGoToPosition(double position) {
    io.setIntakePIDPosition(position);
  }

  
  public Command setIntakePosition(DoubleSupplier position) {
    return this.runEnd(
        () -> {
          intakeGoToPosition((position.getAsDouble()));
        },
        () -> io.setIntakeVoltage(0.0));
  }

  
  public Command moveIntakePosition(DoubleSupplier delta) {
    return this.runEnd(
        () -> {
          io.setIntakeVoltage((5 * delta.getAsDouble()));
        },
        () -> io.setIntakeVoltage(0.0));
  }

    public static Intake getInstance() {
    if (instance == null) {
      if (Constants.getMode() == Mode.REAL && Constants.getRobot() != RobotType.ROBOT_DEFENSE) {
        instance = new Intake(new IntakeIOTalon() {});
      } else {
        instance = new Intake(new IntakeIOSim() {});
      }
    }
    return instance;
  }

  public void intake() {

  }

  public void stop() {
  }

  @Override
  public void periodic() {

  }
}
