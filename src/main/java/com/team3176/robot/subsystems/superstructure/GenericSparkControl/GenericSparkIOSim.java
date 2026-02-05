// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.GenericSparkControl;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;
import com.team3176.robot.constants.BaseConstants;

/** Template hardware interface for a closed loop subsystem. */
public class GenericSparkIOSim implements GenericSparkIO {

  private SingleJointedArmSim genericSparkSim;
 
  private double appliedVolts;

  public GenericSparkIOSim() {
    genericSparkSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1), 20, 0.5, 0.7, -1.0 * Math.PI, 3.14, true, 0.0);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(GenericSparkIOInputs inputs) {
    genericSparkSim.update(BaseConstants.LOOP_PERIODIC_SECS);
    inputs.genericSparkPositionDeg = Units.radiansToDegrees(genericSparkSim.getAngleRads()) + 90;
    inputs.genericSparkVelocityRadPerSec = genericSparkSim.getVelocityRadPerSec();
    inputs.genericSparkAppliedVolts = appliedVolts;
    inputs.genericSparkAmpsStator = genericSparkSim.getCurrentDrawAmps();
    inputs.genericSparkTempCelcius = 0.0;
    Logger.recordOutput("GenericSpark/SimGenericSparkPos", genericSparkSim.getAngleRads());
  }

  @Override
  public void setGenericSparkVolts(double volts) {
    if (DriverStation.isEnabled()) {
      appliedVolts = volts;
    } else {
      appliedVolts = 0.0;
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
    genericSparkSim.setInputVoltage(appliedVolts);
  }
}
