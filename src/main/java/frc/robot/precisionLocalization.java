package frc.robot;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import swervelib.parser.json.modules.DriveConversionFactorsJson;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;



public class precisionLocalization {
    private final Pigeon2 pigeon = new Pigeon2(27, "rio");
    private final CoreTalonFX driveTalon = new CoreTalonFX(1, "rio");
    private  SwerveDrivePoseEstimator poseEstimator;
    final double WHEELBASE_METERS = 0.6858;
    final double TRACKWIDTH_METERS = 0.6858;

    double halfWheelbase = WHEELBASE_METERS / 2.0;
    double halfTrackwidth = TRACKWIDTH_METERS / 2.0;

    Translation2d frontLeftLocation = new Translation2d(halfWheelbase, halfTrackwidth);
    Translation2d frontRightLocation = new Translation2d(halfWheelbase, -halfTrackwidth);
    Translation2d backLeftLocation = new Translation2d(-halfWheelbase, halfTrackwidth);
    Translation2d backRightLocation = new Translation2d(-halfWheelbase, -halfTrackwidth);   

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    );

    public SwerveModulePosition getPosition() {
        StatusSignal<Angle> motorRotatonSignal = driveTalon.getPosition();
        Angle motorAngle = motorRotatonSignal.getValue();

        double rotations = ((Rotation2d) motorAngle).getRotations();

        double distance = rotations * (Math.PI * 0.0508);

        Rotation2d angle = new Rotation2d(((Rotation2d) motorAngle).getRotations());
        return new SwerveModulePosition(distance, angle);
    }

    public precisionLocalization(){
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, pigeon.getRotation2d(), 
        new SwerveModulePosition[] {
            getPosition(), 
            getPosition(), 
            getPosition(), 
            getPosition()
        }, 
        new Pose2d(null));
    
        


    }

    

}
