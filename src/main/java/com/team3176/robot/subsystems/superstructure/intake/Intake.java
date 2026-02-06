package com.team3176.robot.subsystems.superstructure.intake;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.team3176.robot.subsystems.superstructure.intake.IntakeIO.IntakeIOInputs;

/* import team3176.robot.Constants;
import team3176.robot.Constants.Mode;
import team3176.robot.Constants.RobotType;
import team3176.robot.constants.*;
import team3176.robot.util.LoggedTunableNumber;
import team3176.robot.util.TunablePID; */

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void intake() {

  }

  public void stop() {
  }

  @Override
  public void periodic() {

  }
}
