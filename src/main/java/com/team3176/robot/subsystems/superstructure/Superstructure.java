package com.team3176.robot.subsystems.superstructure;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.DoubleSupplier;
//import com.team3176.robot.constants.FieldConstants;
// import java.util.function.IntSupplier;
import com.team3176.robot.subsystems.superstructure.GenericTalonControl.GenericTalon;
import com.team3176.robot.subsystems.superstructure.intake.Intake;
import com.team3176.robot.util.LoggedTunableNumber;
import com.ctre.phoenix6.StatusSignal;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;
public class Superstructure {
  private static Superstructure instance;

  //private GenericTalon genericTalon;
  private  Intake intake;

  public Superstructure() {

    //genericTalon = GenericTalon.getInstance();
    intake = Intake.getInstance();
  }

  public Command intakePositionMotor(DoubleSupplier position) {
    return (intake.runIntake(() -> position.getAsDouble()));
  }

    public Command intakeRollerSpeed(DoubleSupplier Speed_RPS) {
    return (intake.runIntakeRoller(() -> Speed_RPS.getAsDouble()));
  }
  
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }
}
