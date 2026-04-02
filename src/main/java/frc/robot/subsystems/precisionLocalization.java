package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
//import swervelib.parser.json.modules.DriveConversionFactorsJson;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.lang.reflect.Array;

import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;



public class precisionLocalization {

    String driveCanBus = "rio";
    private final Pigeon2 pigeon = new Pigeon2(27, driveCanBus);

    // can IDs and canbus Location for the swerve modules, these should be set to match the actual configuration of the robot
    private final CoreTalonFX frontLeftModule = new CoreTalonFX(1, driveCanBus);
    private final CoreTalonFX frontRightModule = new CoreTalonFX(2, driveCanBus);
    private final CoreTalonFX backLeftModule = new CoreTalonFX(3, driveCanBus);
    private final CoreTalonFX backRightModule = new CoreTalonFX(4, driveCanBus);
    private  NetworkTable SwerveDrivePos;
    public DoublePublisher currentXpos;
    private DoublePublisher currentYpos;
    private DoublePublisher currentRotation;
    private Pose2d currentPose;
    
    

    // creates an array of swerve modules
    private final CoreTalonFX[] thisRobotModules = new CoreTalonFX[] {
        frontLeftModule,
        frontRightModule,
        backLeftModule,
        backRightModule
    };

    private  SwerveDrivePoseEstimator poseEstimator;

    // physical dimensions of our vision setup, these should change to match the actual robot dimensions
    final double WHEELBASE_METERS = 0.55245;
    final double TRACKWIDTH_METERS = 0.55245;

    // positions of wheel relativ to robot center
    double halfWheelbase = WHEELBASE_METERS / 2.0;
    double halfTrackwidth = TRACKWIDTH_METERS / 2.0;

    // Define the locations of the swerve modules relative to the robot center
    Translation2d frontLeftLocation = new Translation2d(halfWheelbase, halfTrackwidth);
    Translation2d frontRightLocation = new Translation2d(halfWheelbase, -halfTrackwidth);
    Translation2d backLeftLocation = new Translation2d(-halfWheelbase, halfTrackwidth);
    Translation2d backRightLocation = new Translation2d(-halfWheelbase, -halfTrackwidth);   


    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    );

    //calculates the position of the swerve module based on the encoder readings from the drive motor, this is a very basic implementation and should be replaced with a more accurate method that also takes into account the angle of the module and any gearing between the motor and the wheel
    public SwerveModulePosition getPosition(CoreTalonFX[] modules) {
        StatusSignal<Angle> motorRotationSignal = modules[0].getPosition();
        
        
        double rotations = motorRotationSignal.getValueAsDouble(); // get the motor position in rotations


        double distance = (rotations / 6.75) * (Math.PI * 0.1016); // Distance in meters measured from the rotation and diameter of the wheels

        Rotation2d angle = Rotation2d.fromDegrees(rotations);
        return new SwerveModulePosition(distance, angle); // return the position of the module as a SwerveModulePosition object
    }

    public precisionLocalization(){
        SwerveDrivePos = NetworkTableInstance.getDefault().getTable("Swerve Drive Position");
        currentXpos = SwerveDrivePos.getDoubleTopic("CurrentXPosition").publish();
        currentYpos = SwerveDrivePos.getDoubleTopic("CurrentYPosition").publish();
        currentRotation = SwerveDrivePos.getDoubleTopic("CurrentRotation").publish();

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, pigeon.getRotation2d(), 
        new SwerveModulePosition[] {
            getPosition(thisRobotModules) 
        }, 
        new Pose2d()); // initial pose, if not passed values, sets to 0,0,0
    }

    public void currentPose(){
        double x = 1.0;
       /* 
        poseEstimator.update(pigeon.getRotation2d(), 
        new SwerveModulePosition[] {
            getPosition(thisRobotModules), 
        });
        currentPose = poseEstimator.getEstimatedPosition(); // gets the current estimated position of the robot
        */
         //currentXpos.set(currentPose.getX());
         currentXpos.set(x);
         //currentYpos.set(currentPose.getY());
         //currentRotation.set(currentPose.getRotation().getDegrees());
    }
}
