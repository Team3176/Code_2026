package com.team3176.robot.subsystems.superstructure;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.team3176.robot.subsystems.leds.LEDSubsystem;

import com.team3176.robot.subsystems.superstructure.GenericSparkControl.GenericSpark;

import com.team3176.robot.subsystems.superstructure.HoodControl.Hood;
import com.team3176.robot.subsystems.superstructure.IntakeControl.IntakeControl;
//import com.team3176.robot.constants.FieldConstants;
// import java.util.function.IntSupplier;
import com.team3176.robot.subsystems.superstructure.GenericTalonControl.GenericTalon;
import com.team3176.robot.subsystems.superstructure.GenericSparkControl.GenericSpark;
import com.team3176.robot.subsystems.superstructure.KickerControl.Kicker;
import com.team3176.robot.subsystems.superstructure.ShooterControl.ShooterControl;
import com.team3176.robot.subsystems.superstructure.TurretRotation.TurretRotation;
import com.team3176.robot.subsystems.superstructure.IntakeControl.IntakeControl;
import com.team3176.robot.util.LoggedTunableNumber;
import com.ctre.phoenix6.StatusSignal;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;
public class Superstructure {
  private static Superstructure instance;


  private Hood hood;
  private ShooterControl shooter;
  private TurretRotation turretRotation;
  private IntakeControl intake;

  private Kicker kicker;

  public Superstructure() {


    shooter = ShooterControl.getInstance();
    turretRotation = TurretRotation.getInstance();
    intake = IntakeControl.getInstance();
    kicker = Kicker.getInstance();
    hood = Hood.getInstance();
  }

  public Command HoodMotor(DoubleSupplier position) {
    return (hood.runHood(() -> position.getAsDouble()));
  }

  public Command IntakePositionMotor(DoubleSupplier position) {
    return (intake.runIntakePosition(() -> position.getAsDouble()));
  }

  public Command IntakeRollerMotor(DoubleSupplier Speed_DutyCylce) {
    return (intake.runIntakeRoller(() -> Speed_DutyCylce.getAsDouble()));
  }
    public Command HoodUp() {
    return (hood.deployFromHomeCmd());
  }
    public Command HoodDown() {
    return (hood.retractTowardHome());
  }

  public Command shooterMotorSpeed(DoubleSupplier Speed_RPS) {
    return (shooter.runDualShooterSpeed(() -> Speed_RPS.getAsDouble()));
  }

  public Command kickerMotorSpeed(DoubleSupplier Speed_RPS) {
    return (kicker.runkickerSpeed(() -> Speed_RPS.getAsDouble()));
  }

  public Command runTurretRotationFromVision(DoubleSupplier positionError, BooleanSupplier isTargetLocked, LEDSubsystem leds ) {
    //return (turretRotation.runTurretRotation(() -> positionError.getAsDouble()));
    return (turretRotation.runTurretRotationFromVision(() -> positionError.getAsDouble(), () -> isTargetLocked.getAsBoolean(), leds));
  }
  
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }
}