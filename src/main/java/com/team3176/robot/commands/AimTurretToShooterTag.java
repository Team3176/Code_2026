package com.team3176.robot.commands;

import com.team3176.robot.subsystems.turret.Turret;
import com.team3176.robot.subsystems.vision.Vision;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import com.team3176.robot.Constants;

/** Command that aims the turret to center the best AprilTag seen by the shooter camera. */
public class AimTurretToShooterTag extends CommandBase {
  private final Vision vision;
  private final Turret turret;
  private final int cameraIndex;
  private final PIDController pid;

  public AimTurretToShooterTag(Vision vision, Turret turret) {
    this(vision, turret, Constants.Mechanism.SHOOTER_CAMERA_INDEX);
  }

  public AimTurretToShooterTag(Vision vision, Turret turret, int cameraIndex) {
    this.vision = vision;
    this.turret = turret;
    this.cameraIndex = cameraIndex;
    // PID gains - conservative defaults; tune on robot
    pid = new PIDController(0.02, 0.0, 0.001);
    pid.setTolerance(Math.toRadians(0.75)); // ~0.75 deg tolerance
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    pid.reset();
  }

  @Override
  public void execute() {
    // If no camera or no observations, hold position
    if (!vision.isCameraConnected(cameraIndex) || !vision.hasPoseObservations(cameraIndex)) {
      turret.stop();
      return;
    }

    Rotation2d tx = vision.getTargetX(cameraIndex);
    double yawRad = tx.getRadians(); // positive/negative depends on camera orientation

    // Use PID on yaw error to generate percent output
    double output = pid.calculate(yawRad, 0.0);
    // Clamp output to safe range
    double max = 0.35; // conservative
    output = Math.max(-max, Math.min(max, output));

    turret.setPercentOutput(output);
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }
}

/*
 Minimal local CommandBase stub so the file compiles even if WPILib is not on the classpath.
 This mirrors the small subset of functionality your command uses (addRequirements, lifecycle hooks).
 Replace this with the real WPILib dependency when available.
*/
abstract class CommandBase {
  public CommandBase() {}

  // Accept any requirements; a loose signature avoids coupling to WPILib subsystem types.
  public void addRequirements(Object... requirements) {}

  // Lifecycle methods that concrete commands override.
  public void initialize() {}
  public void execute() {}
  public boolean isFinished() { return true; }
  public void end(boolean interrupted) {}

  // Convenience stubs (no-op) for scheduling API calls that might be used elsewhere.
  public void schedule() {}
  public void cancel() {}
}
