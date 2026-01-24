package team3176.robot.subsystems.superstructure.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;
//import team3176.robot.Constants;
//import team3176.robot.constants.SuperStructureConstants;

/** Template hardware interface for a closed loop subsystem. */
public class IntakeIOSim implements IntakeIO {

 

  public IntakeIOSim() {
      }
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    
  }

  @Override
  public void setPivotVolts(double volts) {
  }
}
