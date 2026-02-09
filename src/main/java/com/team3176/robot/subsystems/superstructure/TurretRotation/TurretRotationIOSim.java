// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.TurretRotation;

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
public class TurretRotationIOSim implements TurretRotationIO {

  private SingleJointedArmSim turretRotationSim;
 
  private double appliedVolts;

  public TurretRotationIOSim() {
    turretRotationSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1), 20, 0.5, 0.7, -1.0 * Math.PI, 3.14, true, 0.0);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(TurretRotationIOInputs inputs) {
    turretRotationSim.update(BaseConstants.LOOP_PERIODIC_SECS);
    inputs.turretRotationPositionDeg = Units.radiansToDegrees(turretRotationSim.getAngleRads()) + 90;
    inputs.turretRotationVelocityRadPerSec = turretRotationSim.getVelocityRadPerSec();
    inputs.turretRotationAppliedVolts = appliedVolts;
    inputs.turretRotationAmpsStator = turretRotationSim.getCurrentDrawAmps();
    inputs.turretRotationTempCelcius = 0.0;
    Logger.recordOutput("TurretRotation/SimTurretRotationPos", turretRotationSim.getAngleRads());
  }

  @Override
  public void setTurretRotationVolts(double volts) {
    if (DriverStation.isEnabled()) {
      appliedVolts = volts;
    } else {
      appliedVolts = 0.0;
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
    turretRotationSim.setInputVoltage(appliedVolts);
  }
}
