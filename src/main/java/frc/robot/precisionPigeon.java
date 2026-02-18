package frc.robot;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix6.hardware.Pigeon2;;

public class precisionPigeon {

    private static int pigeonCanID = 27; //This is the can ID of the IMU set in phoenix tuner
    private static String pigeonCANBusLocation = "rio"; //this is the canbus that the pigeon is installed on (note this method to be deprecated in 2027)
    private Pigeon2 aPigeonIMU;
    private NetworkTable IMUDataNT;
    private DoubleArrayPublisher accelerationsDisp;
    private DoubleArrayPublisher velocitiesDisp;
    private DoubleArrayPublisher positionsDisp;
    private DoubleArrayPublisher rotationalAccelsDisp;
    private DoubleArrayPublisher rotationalVelocitiesDisp;
    private DoubleArrayPublisher rotationalYPRDisp;


    public precisionPigeon(){
        aPigeonIMU = new Pigeon2(pigeonCanID,pigeonCANBusLocation);

        IMUDataNT = NetworkTableInstance.getDefault().getTable("IMU Data");
        accelerationsDisp           = IMUDataNT.getDoubleArrayTopic("Accels").publish();
        velocitiesDisp              = IMUDataNT.getDoubleArrayTopic("Velocities").publish();
        positionsDisp               = IMUDataNT.getDoubleArrayTopic("positions").publish();
        rotationalAccelsDisp        = IMUDataNT.getDoubleArrayTopic("RotAccel").publish();
        rotationalVelocitiesDisp    = IMUDataNT.getDoubleArrayTopic("RotVel").publish();
        rotationalYPRDisp           = IMUDataNT.getDoubleArrayTopic("RotPos").publish();

    }

    public double PeriodicUpdate(){
        double yaw = aPigeonIMU.getYaw().getValueAsDouble();

        double[] ypr =  {yaw,0,0};
        positionsDisp.set(ypr);

        return yaw;
    }

}
