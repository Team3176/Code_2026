// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import frc.robot.subsystems.controller.Controller;
//import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

//import frc.robot.Constants.OperatorConstants;
import frc.robot.constants.MatchConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//Imports For Swerve
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.math.geometry.Rotation2d;


import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.precisionPigeon;
import frc.robot.subsystems.precisionVision;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // ******** Declarations for swerve drivetrain ***** /
  private double MaxSpeed = 1.0 *  TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = 1.0* RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                 // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);
  
  private boolean shooterLockon = false; 
  private double shooterDistance = 0.0;

 // private final CommandJoystick rotStick = new CommandJoystick(0);
 // private final CommandJoystick transStick = new CommandJoystick(1);
 
  public precisionVision thisRobotVisionHandler;
  public precisionPigeon thisRobotIMUHandler;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // ********* End of swerve drivetrain declarations ********* /

  /* Path Follower */
  private final SendableChooser<Command> autoChooser;

/// Defintion that was NOT Auto Generated - TEAM 3176 2026
  // Controllers
  private final Controller controller = Controller.getInstance();
  
  // Superstructure0
  private final Superstructure superstructure = Superstructure.getInstance();

  private Trigger visionOverride; 
  private Trigger endMatchAlert = new Trigger(() -> DriverStation.getMatchTime() < MatchConstants.ENDGAMEALERT_Time);
  private Trigger autoAlert = new Trigger (() -> DriverStation.isAutonomous());
  private Trigger ShootingTime = new Trigger(()-> isHubActive());
  
  
  // Implement LEDs
  private LEDSubsystem leds = LEDSubsystem.getInstance();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    thisRobotIMUHandler = new precisionPigeon();
    thisRobotVisionHandler = new precisionVision();
    //private Trigger ShooterIsLockedON = new Trigger (()-> thisPrecePrecisionVision.getTurretLockOn());
     Commands.runOnce(()->thisRobotIMUHandler.setIMUYawToDriverZero());
    






    /// Team 3176 2026
  //Leds default commands
    leds.setDefaultCommand(leds.DefaultLED()); //purple
    autoAlert.onTrue(leds.AutoDriveStart());  //rainbow
    endMatchAlert.onTrue(leds.EndgameStart()); // blue
    ShootingTime.onTrue(leds.GoalShiftActive());  //SOLID_LAWN_GREEN
    ShootingTime.onFalse(leds.turretVisonLost()); // red 
   // ShooterIsLockedON.onTrue(leds.turretLockedOn());  // FIXED_TWINKLES_LAVA
   // ShooterIsLockedON.onFalse(leds.DefaultLED());  // purple
    SmartDashboard.putBoolean("Is Hub Active", isHubActive());

    NamedCommands.registerCommand("ShootFromClimb", superstructure.ShotTwoShooter().withTimeout(.1)   
     .andThen(superstructure.ShooterOn())
     .andThen(Commands.waitSeconds(.5))
     .andThen(superstructure.KickerOn()
     .andThen(Commands.waitSeconds(.1)))
     .andThen(superstructure.SpindexerOn()));   

    NamedCommands.registerCommand("ShootFromTrench", 
    superstructure.ShotThreeShooter().withTimeout(.1)
     .andThen(superstructure.ShotThreeHood())
     .andThen(Commands.waitSeconds(.2))
     .andThen(superstructure.ShooterOn())
     .andThen(Commands.waitSeconds(.5))
     .andThen(superstructure.KickerOn()
     .andThen(Commands.waitSeconds(.1)))
     .andThen(superstructure.SpindexerOn()));
     
    NamedCommands.registerCommand("ShootFromClose", 
    superstructure.ShotTwoShooter().withTimeout(.1)   
     .andThen(superstructure.ShooterOn())
     .andThen(Commands.waitSeconds(.5))
     .andThen(superstructure.KickerOn()
     .andThen(Commands.waitSeconds(.1)))
     .andThen(superstructure.SpindexerOn()));  

    NamedCommands.registerCommand("ShootFromHumanFeed", 
    superstructure.ShotFourShooter().withTimeout(.1)   
     .andThen(superstructure.ShooterOn())
     .andThen(Commands.waitSeconds(.5))
     .andThen(superstructure.KickerOn()
     .andThen(Commands.waitSeconds(.1)))
     .andThen(superstructure.SpindexerOn()));  

    NamedCommands.registerCommand("DeployIntake", 
        superstructure.IntakeExtend().withTimeout(1));  

    NamedCommands.registerCommand("RetractIntake", 
        superstructure.IntakeRetract().withTimeout(1));  
    NamedCommands.registerCommand("TurretCenter",
        superstructure.TurretCenter().withTimeout(.2));
    NamedCommands.registerCommand("TurretRight",
        superstructure.TurretRight().withTimeout(.2));
    NamedCommands.registerCommand("TurretLeft",
        superstructure.TurretRight().withTimeout(.2));
    NamedCommands.registerCommand("IntakeRollerIdle",
        superstructure.IntakeRollerIdle().withTimeout(.2));
    NamedCommands.registerCommand("StopShot",
        superstructure.ShooterOff()
        .andThen(superstructure.SpindexerOff()
        .andThen(superstructure.KickerOff())).withTimeout(.2));

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);
    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand().schedule();

    // Configure the trigger bindings
    configureBindings();
    

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Swerve Drive Command Bindings
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive .withVelocityX(-controller.transStick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                                            .withVelocityY(-controller.transStick.getX() * MaxSpeed) // Drive left with negative X (left)
                                            .withRotationalRate(-controller.rotStick.getX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    controller.transStick.button(2).whileTrue(drivetrain.applyRequest(() -> brake));
    controller.transStick.button(10).whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-controller.transStick.getX(), -controller.transStick.getY()))));

    // Run SysId routines when holding back/start and X/Y.

    //transStick.button(8).whileTrue(Commands.runOnce(()->thisRobotIMUHandler.setIMUYawToDriverZero()));t
    

    controller.transStick.button(8).whileTrue(drivetrain.runOnce(()->drivetrain.seedFieldCentric(Rotation2d.kPi))); //This command resets the robot to front (intake) towards DS


    // Note that each routine should be run exactly once in a single log.
    // transStick.back().and(transStick.button(5)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // transStick.back().and(transStick.button(6)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // transStick.start().and(transStick.button(7)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // transStick.start().and(transStick.button(8)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    // transStick.button(3).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    // //TODO Change this function to yaw rezero

    drivetrain.registerTelemetry(logger::telemeterize);

    /// Team 3176 2026
    ///// SETUP OVERRIDE BOX ////////
    visionOverride = controller.switchBox.button(4);

    ///Test Commands 
    //TODO Remove these following testing
    controller.rotStick.button(2).whileTrue((superstructure.HoodMotor(() -> -controller.rotStick.getRawAxis(3))));
    controller.rotStick.button(1).whileTrue((superstructure.shooterMotorSpeed(() -> -controller.transStick.getRawAxis(3))));
    controller.rotStick.button(3).whileTrue(superstructure.KickerOn().andThen(superstructure.SpindexerOn())).onFalse(superstructure.SpindexerOff().andThen(superstructure.KickerOff()));


    /// Operator Commands
    controller.operator.y().onTrue(superstructure.ShotOneHood());
    controller.operator.x().onTrue(superstructure.ShotTwoHood());
    controller.operator.b().onTrue(superstructure.ShotThreeHood());
    controller.operator.a().onTrue(superstructure.ShotFourHood());

    controller.operator.y().onTrue(superstructure.ShotOneShooter());
    controller.operator.x().onTrue(superstructure.ShotTwoShooter());
    controller.operator.b().onTrue(superstructure.ShotThreeShooter());
    controller.operator.a().onTrue(superstructure.ShotFourShooter());

    controller.operator.leftBumper().onTrue(superstructure.IntakeExtend().withTimeout(1));
    controller.operator.rightBumper().onTrue(superstructure.IntakeRetract().withTimeout(2));

    controller.operator.leftTrigger().whileTrue(superstructure.HoodDown());
    controller.operator.rightTrigger().whileTrue(superstructure.HoodUp());

    controller.operator.pov(0).whileTrue(superstructure.TurretCenter());
    controller.operator.pov(270).whileTrue(superstructure.TurretLeft());
    controller.operator.pov(90).whileTrue(superstructure.TurretRight());

    controller.operator.pov(180).whileTrue((superstructure.IntakePositionMaunal(() -> -controller.operator.getRightY())));

    controller.operator.back().onTrue(superstructure.SpindexerReverse()).onFalse(superstructure.SpindexerOff());
    controller.operator.back().onTrue(superstructure.KickerReverse()).onFalse(superstructure.KickerOff());

    controller.operator.start().onTrue(superstructure.ShooterReverse());


    //Climb Part Deux
    controller
        .operator
        .rightBumper()
        .whileTrue(
            superstructure
                .moveClimbLeftRightPosition(
                    () -> -controller.operator.getRightY(), () -> -controller.operator.getLeftY()))
        .onFalse(superstructure.stopClimbLeftRight());


    /// Driver Commands 
    controller.transStick.button(1).onTrue((superstructure.ShooterOn().andThen(Commands.waitSeconds(.5)).andThen(superstructure.KickerOn().andThen(Commands.waitSeconds(.1))).andThen(superstructure.SpindexerOn())));
    controller.transStick.button(1).onFalse((superstructure.SpindexerOff().andThen(Commands.waitSeconds(.2)).andThen(superstructure.KickerOff().andThen(Commands.waitSeconds(.2))).andThen(superstructure.shooterMotorSpeedIDLE())));

    controller.rotStick.button(2).onTrue(superstructure.runTurretRotFromVisionLocation(() -> (thisRobotVisionHandler.estimateGoalRotationFromChassis()), () ->thisRobotVisionHandler.estimateGoalDistanceFromChassis()));
    controller.rotStick.button(1).onTrue(superstructure.IntakeRollerReverse()).onFalse(superstructure.IntakeRollerResume());


      /// Swtich box Commands
      // TODO do we need anything here?.
      //controller.switchBox.button(4).onTrue(drive.setVisionOverride(true)).onFalse(drive.setVisionOverride(false));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

//TODO 2026 Game specific hub activations
    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
         // If we have no alliance, we cannot be enabled, therefore no hub.
         if (alliance.isEmpty()) {
            return false;
        }
  // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
         }
  // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

  // We're teleop enabled, compute.
         double matchTime = DriverStation.getMatchTime();
         String gameData = DriverStation.getGameSpecificMessage();
  // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }

  // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
    // Transition shift, hub is active.
            return true;
        } 
        else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } 
        else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        }
        else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        }
        else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } 
        else {
            // End game, hub always active.
            return true;
        }
    }

}
