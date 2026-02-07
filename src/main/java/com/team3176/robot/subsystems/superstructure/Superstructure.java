package com.team3176.robot.subsystems.superstructure;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.DoubleSupplier;

import com.team3176.robot.subsystems.superstructure.GenericSparkControl.GenericSpark;

import com.team3176.robot.subsystems.HoodControl.Hood;
//import com.team3176.robot.constants.FieldConstants;
// import java.util.function.IntSupplier;
import com.team3176.robot.subsystems.superstructure.GenericTalonControl.GenericTalon;
import com.team3176.robot.subsystems.superstructure.GenericSparkControl.GenericSpark;
import com.team3176.robot.subsystems.superstructure.KickerControl.Kicker;
import com.team3176.robot.util.LoggedTunableNumber;
import com.ctre.phoenix6.StatusSignal;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.util.LoggedTunableNumber;
import com.team3176.robot.util.TunablePID;
public class Superstructure {
  private static Superstructure instance;

  private GenericTalon genericTalon;
  private GenericSpark genericSpark;
  private Hood hood;

  private Kicker kicker;

  public Superstructure() {

    genericTalon = GenericTalon.getInstance();
    genericSpark = GenericSpark.getInstance();

    kicker = Kicker.getInstance();

    hood = hood.getInstance();
  }

  public Command genericPositionMotor(DoubleSupplier position) {
    return (genericTalon.runGenericTalon(() -> position.getAsDouble()));
  }

    public Command HoodMotor(DoubleSupplier position) {
    return (hood.runHood(() -> position.getAsDouble()));
  }

  public Command genericMotorSpeed(DoubleSupplier Speed_RPS) {
    return (genericTalon.runGenericTalonSpeed(() -> Speed_RPS.getAsDouble()));
  }

  public Command genericDualMotorSpeed(DoubleSupplier Speed_RPS) {
    return (genericTalon.runGenericTalonDualSpeed(() -> Speed_RPS.getAsDouble()));
  }

  public Command genericSparkPositionMotor(DoubleSupplier position) {
    return (genericSpark.runGenericSpark(() -> position.getAsDouble()));
  }

  public Command genericSparkMotorSpeed(DoubleSupplier Speed_RPM) {
    return (genericSpark.runGenericSparkSpeed(() -> Speed_RPM.getAsDouble()));
  }

  public Command genericSparkDualMotorSpeed(DoubleSupplier Speed_RPM) {
    return (genericSpark.runGenericSparkDualSpeed(() -> Speed_RPM.getAsDouble()));
  }


    public Command kickerMotorSpeed(DoubleSupplier Speed_RPS) {
    return (kicker.runkickerSpeed(() -> Speed_RPS.getAsDouble()));
  }
  
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }
}