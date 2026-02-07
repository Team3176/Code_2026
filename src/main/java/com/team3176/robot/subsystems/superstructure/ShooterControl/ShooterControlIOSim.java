// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3176.robot.subsystems.superstructure.ShooterControl;

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
public class ShooterControlIOSim implements ShooterControlIO {

  private SingleJointedArmSim shooterSim;
 
  private double appliedVolts;

  public ShooterControlIOSim() {
    shooterSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500(1), 20, 0.5, 0.7, -1.0 * Math.PI, 3.14, true, 0.0);
  }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ShooterControlIOInputs inputs) {
    shooterSim.update(BaseConstants.LOOP_PERIODIC_SECS);
    inputs.genericTalonPositionDeg = Units.radiansToDegrees(shooterSim.getAngleRads()) + 90;
    inputs.genericTalonVelocityRadPerSec = shooterSim.getVelocityRadPerSec();
    inputs.genericTalonAppliedVolts = appliedVolts;
    inputs.genericTalonAmpsStator = shooterSim.getCurrentDrawAmps();
    inputs.genericTalonTempCelcius = 0.0;
    Logger.recordOutput("Shooter/SimShooternPos", shooterSim.getAngleRads());
  }

  @Override
  public void setDualShooterSpeedVelocity(double volts) {
    if (DriverStation.isEnabled()) {
      appliedVolts = volts;
    } else {
      appliedVolts = 0.0;
    }
    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
    shooterSim.setInputVoltage(appliedVolts);
  }
}
