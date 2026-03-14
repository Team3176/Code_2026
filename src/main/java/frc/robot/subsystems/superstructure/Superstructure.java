package frc.robot.subsystems.superstructure;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.leds.LEDSubsystem;


//import frc.robot.constants.FieldConstants;
// import java.util.function.IntSupplier;
import frc.robot.subsystems.superstructure.KickerControl.Kicker;
import frc.robot.subsystems.superstructure.ShooterControl.ShooterControl;
import frc.robot.subsystems.superstructure.TurretRotation.TurretRotation;
import frc.robot.subsystems.superstructure.HoodControl.Hood;
import frc.robot.subsystems.superstructure.IntakeControl.IntakeControl;
import frc.robot.subsystems.superstructure.ClimbControl.ClimbControl;
import frc.robot.subsystems.superstructure.Spindexer.Spindexer;
import frc.robot.util.LoggedTunableNumber;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.constants.SuperStructureConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunablePID;
public class Superstructure {
  


  private static Superstructure instance;


  private Hood hood;
  private ShooterControl shooter;
  private TurretRotation turretRotation;
  private IntakeControl intake;
  private ClimbControl climb;
  private Spindexer spindexer;

  private Kicker kicker;

  public Superstructure() {


    shooter = ShooterControl.getInstance();
    turretRotation = TurretRotation.getInstance();
    intake = IntakeControl.getInstance();
    kicker = Kicker.getInstance();
    hood = Hood.getInstance();
    climb = ClimbControl.getInstance();
    spindexer = Spindexer.getInstance();
  }

  public Command HoodMotor(DoubleSupplier position) {
    return (hood.runHood(() -> position.getAsDouble()));
  }

  public Command IntakePositionMotor(DoubleSupplier position) {
    return (intake.runIntakePosition(() -> position.getAsDouble()));
  }

    public Command IntakePositionMaunal(DoubleSupplier volts) {
    return (intake.runIntakePositionVoltageManual(() -> volts.getAsDouble()));
  }

  public Command IntakeExtend(){
    return (intake.deployFromHomeCmd());
  }

  public Command IntakeRetract(){
    return (intake.retractToHome());
  }
  public Command IntakeRollerReverse(){
    return (intake.runIntakeRollerReverse());
  }
  public Command IntakeRollerResume(){
    return (intake.runIntakeRollerResume());
  }

  public Command IntakeIncrementalRetract(){
    return (intake.retractTowardHome());
  }

  public Command IntakeRollerMotor(DoubleSupplier Speed_DutyCylce) {
    return (intake.runIntakeRoller(() -> Speed_DutyCylce.getAsDouble()));
  }

  public Command IntakeRollerIdle(){
    return (intake.runIntakeRollerIdle());
  }

  public Command SpindexerMotor(DoubleSupplier speed) {
    return (spindexer.runSpindexerSpeed(() -> speed.getAsDouble()));
  }

  
  public Command HoodUp() {
    return (hood.deployFromHomeCmd());
  }
  public Command HoodDown() {
    return (hood.retractTowardHome());
  }

  //turret turn commands
    public Command TurretIncrementLeft() {
    return (turretRotation.moveTurretLeftbyIncrement());
  }
  
  public Command TurretIncrementRight() {
    return (turretRotation.moveTurretRightbyIncrement());
  }

   public Command TurretCenter() {
    return (turretRotation.moveTurretToCenter());
  }
     public Command TurretLeft() {
    return (turretRotation.moveTurretToMaxCounterClockwise());
  }
     public Command TurretRight() {
    return (turretRotation.moveTurretToMaxClockwise());
  }

  public Command ClimbPositionMotor(DoubleSupplier position) {
    return (climb.runClimb(() -> position.getAsDouble()));
  }

  public Command RetractClimb() {
    return (climb.Climb2Home());
  }

  public Command ExtendClimb() {
    return (climb.Climb2Extend());
  }

  public Command moveClimbLeftRightPosition(DoubleSupplier deltaLeft, DoubleSupplier deltaRight) {
    return climb.moveLeftRightPosition(deltaLeft, deltaRight);
  }

  public Command stopClimbLeftRight() {
    return climb.stopLeftRight();
  }

  public Command shooterMotorSpeed(DoubleSupplier Speed_RPS) {
    return (shooter.runDualShooterSpeed(() -> Speed_RPS.getAsDouble()));
  }

  public Command shooterMotorSpeedIDLE () {
    return (shooter.runDualShooterSpeedIDLE());
  }

 public Command toggleShooterStatus () {
    return (shooter.toggleShooterStatus());
  }

  public Command ShooterOn() {
    return (shooter.runShooterOn());
  }

  public Command ShooterReverse() {
    return (shooter.runShooterReverse());
  }


  public Command ShooterOff() {
    return (shooter.runShooterOff());
  }

  public Command ShotOneShooter() {
    return (shooter.setShooterONESpeed());
  }    
  public Command ShotTwoShooter() {
    return (shooter.setShooterTWOSpeed());
  }    
  public Command ShotThreeShooter() {
    return (shooter.setShooterTHREESpeed());
  }    
  public Command ShotFourShooter() {
    return (shooter.setShooterFOURSpeed());
  }

  public Command ShotOneHood() {
    return (hood.setHoodONEPos());
  }    
  public Command ShotTwoHood() {
    return (hood.setHoodTWOPos());
  }    
  public Command ShotThreeHood() {
    return (hood.setHoodTHREEPos());
  }    
  public Command ShotFourHood() {
    return (hood.setHoodFOURPos());
  }

  public Command kickerMotorSpeed(DoubleSupplier Speed_RPS) {
    return (kicker.runkickerSpeed(() -> Speed_RPS.getAsDouble()));
  }
  public Command KickerOn() {
    return (kicker.runkickerOn());
  }

  public Command KickerReverse() {
    return (kicker.runkickerReverse());
  }

  public Command KickerOff() {
    return (kicker.runkickerOff());
  }

  public Command SpindexerOn() {
    return (spindexer.runSpindexerOn());
  }

  public Command SpindexerReverse() {
    return (spindexer.runSpindexerReverse());
  }

    public Command SpindexerOff() {
    return (spindexer.runSpindexerOff());
  }

  public Command runTurretRotationFromVision(DoubleSupplier positionError, BooleanSupplier isTargetLocked, LEDSubsystem leds ) {
    //return (turretRotation.runTurretRotation(() -> positionError.getAsDouble()));
    return (turretRotation.runTurretRotationFromVision(() -> positionError.getAsDouble(), () -> isTargetLocked.getAsBoolean(), leds));
  }

  public Command runTurretRotFromVisionLocation(DoubleSupplier turretRotRaidianToPoint){
    return (turretRotation.runTurretRotationFromVisionLocation(() -> turretRotRaidianToPoint.getAsDouble()));
  }

  public Command runHoodPositionFromVision(DoubleSupplier distance, BooleanSupplier isTargetLocked) {
    return (hood.runHoodFromDistance(distance, isTargetLocked));
  }
  
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
      System.out.println("Superstructure instance created.");
    }
    return instance;
  }
}

