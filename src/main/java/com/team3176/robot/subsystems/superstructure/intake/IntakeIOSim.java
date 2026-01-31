package com.team3176.robot.subsystems.superstructure.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;
//import team3176.robot.Constants;
//import team3176.robot.constants.SuperStructureConstants;

import com.team3176.robot.constants.BaseConstants;

/** Template hardware interface for a closed loop subsystem. */
public class IntakeIOSim implements IntakeIO {

  private SingleJointedArmSim intakeSim;
 
  private double appliedVolts;

 

  public IntakeIOSim() {
    intakeSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1), 20, 0.5, 0.7, -1.0 * Math.PI, 3.14, true, 0.0);
      }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.update(BaseConstants.LOOP_PERIODIC_SECS);
    inputs.intakePositionDeg = Units.radiansToDegrees(intakeSim.getAngleRads()) + 90;
    inputs.intakeVelocityRadPerSec = intakeSim.getVelocityRadPerSec();
    inputs.intakeAppliedVolts = appliedVolts;
    inputs.intakeAmpsStator = intakeSim.getCurrentDrawAmps();
    inputs.intakeTempCelcius = 0.0;
    Logger.recordOutput("Intake/SimIntakePos", intakeSim.getAngleRads());
  }
  @Override
  public void setIntakeVolts(double volts) {
    if (DriverStation.isEnabled()) {
      appliedVolts = volts;
    } else {
      appliedVolts = 0.0;
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
    intakeSim.setInputVoltage(appliedVolts);
  }
  }

