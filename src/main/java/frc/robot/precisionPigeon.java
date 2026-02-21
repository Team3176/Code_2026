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
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix6.hardware.Pigeon2;

import dev.doglog.internal.tunable.entry.ToggleableBooleanSubscriber;;

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
    private DoubleSubscriber manualX;
    private DoublePublisher HomeXpos;
    private DoubleSubscriber manualY;
    private DoubleSubscriber manualYaw;
    private BooleanSubscriber updatePosition;
    private BooleanPublisher ispositionupdating;


    public precisionPigeon(){
        aPigeonIMU = new Pigeon2(pigeonCanID,pigeonCANBusLocation);

        IMUDataNT = NetworkTableInstance.getDefault().getTable("IMU Data");
        accelerationsDisp           = IMUDataNT.getDoubleArrayTopic("Accels").publish();
        velocitiesDisp              = IMUDataNT.getDoubleArrayTopic("Velocities").publish();
        positionsDisp               = IMUDataNT.getDoubleArrayTopic("positions").publish();
        rotationalAccelsDisp        = IMUDataNT.getDoubleArrayTopic("RotAccel").publish();
        rotationalVelocitiesDisp    = IMUDataNT.getDoubleArrayTopic("RotVel").publish();
        rotationalYPRDisp           = IMUDataNT.getDoubleArrayTopic("RotPos").publish();
        manualX                     = IMUDataNT.getDoubleTopic("HomeXpos").subscribe(0);
        manualY                     = IMUDataNT.getDoubleTopic("HomeYpos").subscribe(0);
        manualYaw                   = IMUDataNT.getDoubleTopic("ManualYaw").subscribe(0);
        updatePosition              = IMUDataNT.getBooleanTopic("UpdatePosition").subscribe(true);
        ispositionupdating          = IMUDataNT.getBooleanTopic("UpdatePosition").publish();
        Shuffleboard.getTab("Homexpos")
         .add("HomeXpos", 0)
         .withWidget(BuiltInWidgets.kTextView) // specify the widget here
         .getEntry();
        Shuffleboard.getTab("Homeypos")
         .add("HomeYpos", 0)
         .withWidget(BuiltInWidgets.kTextView) // specify the widget here
         .getEntry();
         Shuffleboard.getTab("ManualYaw")
         .add("ManualYaw", 0)
         .withWidget(BuiltInWidgets.kTextView) // specify the widget here
         .getEntry();
        

    }

    public double PeriodicUpdate(){
        double yaw = aPigeonIMU.getYaw().getValueAsDouble();
        double pitch = aPigeonIMU.getPitch().getValueAsDouble();
        double roll = aPigeonIMU.getRoll().getValueAsDouble();

        double[] ypr =  {yaw,pitch,roll};
        positionsDisp.set(ypr);

        return yaw;
    }

    public void updateCurrentPositionEstimate(double x, double y, double yaw){
        
        aPigeonIMU.setYaw(yaw);


    }

    private void checkForPositionInput(){
        if(updatePosition.getAsBoolean()){

            aPigeonIMU.setYaw(manualYaw.get());
            

        }

    }

}
