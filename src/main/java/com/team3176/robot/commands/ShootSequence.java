package com.team3176.robot.commands;

import static com.team3176.robot.constants.DriveConstants.AutoConstants.kEndTriggerDebounce;
import static com.team3176.robot.constants.DriveConstants.AutoConstants.kPositionTolerance;
import static com.team3176.robot.constants.DriveConstants.AutoConstants.kRotationTolerance;
import static com.team3176.robot.constants.DriveConstants.AutoConstants.kSpeedTolerance;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.team3176.robot.constants.DriveConstants;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.subsystems.drivetrain.Drive;
import com.team3176.robot.subsystems.superstructure.*;
import com.team3176.robot.subsystems.superstructure.KickerControl.Kicker;
import com.team3176.robot.subsystems.superstructure.ShooterControl.ShooterControl;
import com.team3176.robot.subsystems.superstructure.ShooterControl.ShooterControlIO;
import com.team3176.robot.subsystems.superstructure.Spindexer.Spindexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ShootSequence extends SequentialCommandGroup {
    Superstructure superstructure = Superstructure.getInstance();
    public ShootSequence(DoubleSupplier speed) {
        addCommands(
            ShooterControl.getInstance().runDualShooterSpeed(speed).withTimeout(SuperStructureConstants.Shooter_SpinUpSeconds),
            Kicker.getInstance().runkickerSpeed(speed).withTimeout(SuperStructureConstants.Kicker_SpinUpSeconds),
            Spindexer.getInstance().runSpindexerSpeed(speed));
            //ShooterControl.getInstance().runDualShooterSpeed(speed);
             
        
    }
}