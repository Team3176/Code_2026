package com.team3176.robot.subsystems.superstructure;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;
//import com.team3176.robot.constants.FieldConstants;
// import java.util.function.IntSupplier;
import com.team3176.robot.subsystems.superstructure.ShooterControl.Shooter;
import com.team3176.robot.util.LoggedTunableNumber;
import com.ctre.phoenix6.StatusSignal;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;
public class Superstructure {
  private static Superstructure instance;

  private Shooter Shooter;

  public Superstructure() {

    Shooter = Shooter.getInstance();
  }

  public Command shooterPositionMotor(DoubleSupplier position) {
    return (Shooter.runShooter(() -> position.getAsDouble()));
  }

  public Command shooterMotorSpeed(DoubleSupplier Speed_RPS) {
    return (Shooter.runShooterSpeed(() -> Speed_RPS.getAsDouble()));
  }
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }
}
