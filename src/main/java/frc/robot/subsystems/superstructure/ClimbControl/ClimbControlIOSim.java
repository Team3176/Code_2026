// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.ClimbControl;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.BaseConstants;

/** Template hardware interface for a closed loop subsystem. */
public class ClimbControlIOSim implements ClimbControlIO {

  private SingleJointedArmSim ClimbSim;
 
  private double appliedVolts;

  public ClimbControlIOSim() {
    ClimbSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1), 20, 0.5, 0.7, -1.0 * Math.PI, 3.14, true, 0.0);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ClimbControlIOInputs inputs) {
    ClimbSim.update(BaseConstants.LOOP_PERIODIC_SECS);
    inputs.ClimbPositionDeg = Units.radiansToDegrees(ClimbSim.getAngleRads()) + 90;
    inputs.ClimbVelocityRadPerSec = ClimbSim.getVelocityRadPerSec();
    inputs.ClimbAppliedVolts = appliedVolts;
    inputs.ClimbAmpsStator = ClimbSim.getCurrentDrawAmps();
    inputs.ClimbTempCelcius = 0.0;
    Logger.recordOutput("Climb/SimClimbPos", ClimbSim.getAngleRads());
  }

  @Override
  public void setClimbVolts(double volts) {
    if (DriverStation.isEnabled()) {
      appliedVolts = volts;
    } else {
      appliedVolts = 0.0;
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
    ClimbSim.setInputVoltage(appliedVolts);
  }
}
