package com.team3176.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team3176.robot.Constants;
import com.team3176.robot.generated.TunerConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Simple Turret subsystem using a TalonFX. This is a straightforward wrapper; replace with
 * a YAMS-based implementation later if desired. */
public class Turret extends SubsystemBase {
  private final TalonFX motor;

  public Turret() {
    // Create motor on given CAN ID using configured CAN bus name
    motor = new TalonFX(Constants.CAN.TURRET_MOTOR, TunerConstants.DrivetrainConstants.CANBusName);

    // Basic configuration: set position to 0 (seed) and brake mode
    var cfg = new TalonFXConfiguration();
    try {
      cfg.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
      motor.getConfigurator().apply(cfg, 0.25);
    } catch (Exception e) {
      // ignore config errors for now; device may not be present during simulation
    }

    try {
      motor.setPosition(0.0, 0.25);
    } catch (Exception e) {
      // ignore
    }
  }

  /** Returns current turret angle in radians. Uses motor rotations / gear ratio -> turret rotations. */
  public double getAngleRadians() {
    try {
      // TalonFX position reported as rotations
      double motorRotations = motor.getPosition().getValueAsDouble();
      double turretRotations = motorRotations / Constants.Mechanism.TURRET_MOTOR_TO_TURRET_GEAR_RATIO;
      return Units.rotationsToRadians(turretRotations);
    } catch (Exception e) {
      return 0.0;
    }
  }

  /** Returns current turret angle as Rotation2d. */
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(getAngleRadians());
  }

  /** Set open-loop percent output on the turret motor (-1.0..1.0). */
  public void setPercentOutput(double percent) {
    try {
      motor.setVoltage(percent * 12.0);
    } catch (Exception e) {
      // ignore when hardware not available
    }
  }

  /** Stop the turret motor. */
  public void stop() {
    setPercentOutput(0.0);
  }
}
