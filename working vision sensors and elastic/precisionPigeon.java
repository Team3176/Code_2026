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
    private DoublePublisher rotationalAccelsDisp;
    private DoublePublisher rotationalVelocitiesDisp;
    private DoublePublisher rotationalYPRDisp;
    private DoubleSubscriber manualX;
    private DoublePublisher HomedXpos;
    private DoubleSubscriber manualY;
    private DoubleSubscriber manualYaw;
    private DoublePublisher setyaw;
    private BooleanSubscriber updatePosition;
    private BooleanPublisher ispositionupdating;
    


    public precisionPigeon(){
        aPigeonIMU = new Pigeon2(pigeonCanID,pigeonCANBusLocation);

        IMUDataNT = NetworkTableInstance.getDefault().getTable("IMU Data");
        accelerationsDisp           = IMUDataNT.getDoubleArrayTopic("Accels").publish();
        velocitiesDisp              = IMUDataNT.getDoubleArrayTopic("Velocities").publish();
        positionsDisp               = IMUDataNT.getDoubleArrayTopic("positions").publish();
        rotationalAccelsDisp        = IMUDataNT.getDoubleTopic("RotAccel").publish();
        rotationalVelocitiesDisp    = IMUDataNT.getDoubleTopic("RotVel in Degrees Per Second").publish();
        rotationalYPRDisp           = IMUDataNT.getDoubleTopic("RotPos").publish();
        manualX                     = IMUDataNT.getDoubleTopic("HomedXpos").subscribe(0);
        manualY                     = IMUDataNT.getDoubleTopic("ManualY").subscribe(0);               
        manualYaw                   = IMUDataNT.getDoubleTopic("ManualYaw").subscribe(0);
        setyaw                      = IMUDataNT.getDoubleTopic("SetYaw").publish();
        updatePosition              = IMUDataNT.getBooleanTopic("UpdatePosition").subscribe(false);

        
  /*/
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
        
*/
    }

    public double PeriodicUpdate(){

        double yaw = aPigeonIMU.getYaw().getValueAsDouble();
        double pitch = aPigeonIMU.getPitch().getValueAsDouble();
        double roll = aPigeonIMU.getRoll().getValueAsDouble();

        double SetYaw = manualYaw.get();
        double accelx = aPigeonIMU.getAccelerationX().getValueAsDouble();
        double accely = aPigeonIMU.getAccelerationY().getValueAsDouble();

        double[] accels = { accelx,accely,};
        accelerationsDisp.set(accels);

        double velocitydisplacementx = aPigeonIMU.getAngularVelocityXDevice().getValueAsDouble();
        double velocitydisplacementy = aPigeonIMU.getAngularVelocityYDevice().getValueAsDouble();
        double[] velocitydisplacementotal = { velocitydisplacementx,velocitydisplacementy};

        velocitiesDisp.set(velocitydisplacementotal);
        double rotations = aPigeonIMU.getYaw().getValueAsDouble();

        double rotationalposition = (rotations - yaw);
        double rotationalvelocity = aPigeonIMU.getAngularVelocityZDevice().getValueAsDouble();
        rotationalVelocitiesDisp.set(rotationalvelocity);
        rotationalYPRDisp.set(rotationalposition);

        double rotationalaccleration = aPigeonIMU.getAccelerationZ().getValueAsDouble();
        rotationalAccelsDisp.set(rotationalaccleration);


        if (updatePosition.getAsBoolean()){
         aPigeonIMU.setYaw(SetYaw);
         

         
        }
        

        double[] ypr =  {yaw,pitch,roll};
        positionsDisp.set(ypr);

        return yaw;
    }

    public void updateCurrentPositionEstimate(double x, double y, double yaw){
        
        aPigeonIMU.setYaw(yaw);


    }

    private void checkForPositionInput(){
        if(updatePosition.getAsBoolean()){

            aPigeonIMU.setYaw(manualYaw.get(0));
            

        }

    }

}
