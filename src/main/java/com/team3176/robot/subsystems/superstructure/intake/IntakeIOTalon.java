package com.team3176.robot.subsystems.superstructure.intake;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import com.team3176.robot.constants.Hardwaremap;
import com.team3176.robot.constants.SuperStructureConstants;
import com.team3176.robot.util.TalonUtils;


    /** Template hardware interface for a closed loop subsystem. */
public class IntakeIOTalon implements IntakeIO {
  private TalonFX intakeController;
  private TalonFX intakeRollerController;
  private CANcoder intakeEncoder;
  VelocityVoltage voltVelocity = new VelocityVoltage(0);
  VoltageOut intakeVolts = new VoltageOut(0.0);
  PositionVoltage voltPosition = new PositionVoltage(0);
  private Rotation2d encoderOffset; 
  private double intake_pos_offset = 0;
  
  DigitalInput intakeLinebreak;

  private final StatusSignal<Voltage> intakeAppliedVolts;
  private final StatusSignal<Current> intakeCurrentAmpsStator;
  private final StatusSignal<Current> intakeCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> intakeVelocity;
  private final StatusSignal<Angle> intakePosition;
  private final StatusSignal<Angle> intakeAbsolutePosition;
  private final StatusSignal<Temperature> intakeTemp;

        // constructor if needed for some inputs
    public IntakeIOTalon() {
    TalonFXConfiguration intakeControllerConfigs = new TalonFXConfiguration();
    TalonFXConfiguration intakeRollerControllerConfigs = new TalonFXConfiguration();
 
    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    // CHANGE THE CANID NAMES LATER!! 
    intakeController = new TalonFX(Hardwaremap.genericTalon_CID, Hardwaremap.genericTalon_CBN);
    intakeRollerController = new TalonFX(Hardwaremap.genericTalonSpeed_CID, Hardwaremap.genericTalonSpeed_CBN);
    intakeEncoder = new CANcoder(Hardwaremap.genericTalonCancoder_CID, Hardwaremap.genericTalon_CBN);
 

 
    //var genericTalonEncoderConfig = new CANcoderConfiguration();
    encoderOffset = Rotation2d.fromDegrees(SuperStructureConstants.GenericTalon_ENCODER_OFFSET);
    //genericTalonEncoderConfig.MagnetSensor.MagnetOffset = encoderOffset.getRotations();
    

    //armPivotEncoder.getConfigurator().apply(pivotEncoderConfig);

    intakeControllerConfigs.Slot0.kP = 3; // An error of 1 rotation results in 2.4 V output
    intakeControllerConfigs.Slot0.kI = 0.1; // No output for integrated error
    intakeControllerConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // set max output voltage limits speed - 14V is max output available 
    intakeControllerConfigs.Voltage.PeakForwardVoltage = SuperStructureConstants.GenericTalon_MAX_OUTPUT_VOLTS; 
    intakeControllerConfigs.Voltage.PeakReverseVoltage = SuperStructureConstants.GenericTalon_MAXNeg_OUTPUT_VOLTS;

    intakeControllerConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //TODO if position from Cancoder define which CanCoder / remote sensor to use for position feedback
    //intakeControllerConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.genericTalonCancoder_CID;
    //intakeControllerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    //intakeControllerConfigs.Feedback.SensorToMechanismRatio = 1.0;

    intakeControllerConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    intakeControllerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeControllerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeControllerConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.6;
    intakeControllerConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    intakeControllerConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    intakeControllerConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

    TalonUtils.applyTalonFxConfigs(intakeController, intakeControllerConfigs);
    //intakeController.setPosition(0, 0);

    //SETUP SPEED CONTROL CONFIGS
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    intakeRollerControllerConfigs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    intakeRollerControllerConfigs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    intakeRollerControllerConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    intakeRollerControllerConfigs.Slot0.kI = 0; // No output for integrated error
    intakeRollerControllerConfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    intakeRollerControllerConfigs.Voltage.withPeakForwardVoltage (SuperStructureConstants.GenericTalonSpeed_MAX_OUTPUT_VOLTS)
      .withPeakReverseVoltage(SuperStructureConstants.GenericTalonSpeed_MAXNeg_OUTPUT_VOLTS);

    TalonUtils.applyTalonFxConfigs(intakeRollerController, intakeRollerControllerConfigs);


    intakeAppliedVolts = intakeController.getMotorVoltage();
    intakeCurrentAmpsStator = intakeController.getStatorCurrent();
    intakeCurrentAmpsSupply = intakeController.getSupplyCurrent();
    intakeVelocity = intakeController.getVelocity();
    intakePosition = intakeController.getPosition();
    
    //If you want to use a cancode use this definition 
    //genericTalonPosition = genericTalonEncoder.getPositionSinceBoot();
    intakeAbsolutePosition = intakeEncoder.getAbsolutePosition();
    intakeTemp = intakeController.getDeviceTemp();

    intake_pos_offset = intakeEncoder.getPosition().getValueAsDouble();


    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        intakeAppliedVolts,
        intakeCurrentAmpsStator,
        intakeVelocity,
        intakePosition,
        intakeTemp,
        intakeCurrentAmpsSupply);

    intakeController.optimizeBusUtilization();
    intakeRollerController.optimizeBusUtilization();
    
  }
  
  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
     BaseStatusSignal.refreshAll(
        intakeAppliedVolts,
        intakeCurrentAmpsStator,
        intakeVelocity,
        intakePosition,
        intakeTemp,
        intakeCurrentAmpsSupply
        );

    
    inputs.intakeAppliedVolts = intakeAppliedVolts.getValueAsDouble();
    inputs.intakeAmpsStator = intakeCurrentAmpsStator.getValueAsDouble();
    inputs.intakeAmpsSupply = intakeCurrentAmpsSupply.getValueAsDouble();
    inputs.intakeTempCelcius = intakeTemp.getValueAsDouble();
    inputs.intakePositionDeg = Units.rotationsToDegrees(intakePosition.getValueAsDouble());
    inputs.intake_pos_offset = intake_pos_offset;
    inputs.intakePositionRot = intakeController.getPosition().getValueAsDouble();
    //Use if using cancoder
    //inputs.genericTalonPositionRot = genericTalonEncoder.getPosition().getValueAsDouble() - genericTalon_pos_offset;
    inputs.intakePositionRotREAL = intakeEncoder.getPosition().getValueAsDouble(); 
    inputs.intakeVelocityRadPerSec = Units.rotationsToRadians(intakeVelocity.getValueAsDouble());

    inputs.intakeAbsolutePositionDegrees =
        MathUtil.inputModulus(
            Rotation2d.fromRotations(intakeEncoder.getAbsolutePosition().getValueAsDouble())
                .minus(encoderOffset)
                .getDegrees(),
            -180,
            180);

  }

  //Use this to provide a speed based on voltage - it is not "controlling to speed"
  @Override
  public void setIntakeVolts(double volts) {
    intakeController.setControl(intakeVolts.withOutput(volts));
  }

  //Offset would be used when we need 
  @Override
  public void setIntakeVoltagePos(double position) {
    intakeController.setControl(voltPosition.withPosition(position + intake_pos_offset));
  }


  @Override
  public void setIntakeBrakeMode(boolean enable) {
    if (enable) {
      intakeController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      intakeController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void setIntakeRollerBrakeMode(boolean enable) {
    if (enable) {
      intakeRollerController.setNeutralMode(NeutralModeValue.Brake);
    } else {
      intakeRollerController.setNeutralMode(NeutralModeValue.Coast);
    }
  }

    //Offset would be used when we need 
  @Override
  public void setIntakeRollerVelocity(double speed_RPS) {
    intakeRollerController.setControl(voltVelocity.withVelocity(speed_RPS));
  }

}
