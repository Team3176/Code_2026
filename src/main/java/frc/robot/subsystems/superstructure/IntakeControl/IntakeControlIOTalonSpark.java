// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.IntakeControl;

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

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.Hardwaremap;
import frc.robot.constants.SuperStructureConstants;
import frc.robot.util.TalonUtils;


/** Template hardware interface for a closed loop subsystem. */
public class IntakeControlIOTalonSpark implements IntakeControlIO {

  private TalonFX IntakePositionController;
  //private CANcoder IntakePositionEncoder;
  
  private SparkFlex IntakeRollerMotor;
  private SparkClosedLoopController IntakeRollerController;
  private RelativeEncoder IntakeRollerEncoder;
  
  VelocityVoltage voltVelocity = new VelocityVoltage(0);
  VoltageOut IntakePositionVolts = new VoltageOut(0.0);
  PositionVoltage voltIntakePositionSlot0 = new PositionVoltage(0).withSlot(0);
  PositionVoltage voltIntakePositionSlot1 = new PositionVoltage(0).withSlot(1);
  private Rotation2d encoderOffset; 
  
  DigitalInput HoodLinebreak;

  private final StatusSignal<Voltage> IntakePositionAppliedVolts;
  private final StatusSignal<Current> IntakePositionCurrentAmpsStator;
  private final StatusSignal<Current> IntakePositionCurrentAmpsSupply;
  private final StatusSignal<AngularVelocity> IntakePositionVelocity;
  private final StatusSignal<Angle> IntakePositionPosition;
  private final StatusSignal<Angle> IntakePositionAbsolutePosition;
  private final StatusSignal<Temperature> IntakePositionTemp;


  public IntakeControlIOTalonSpark() {

 
    TalonFXConfiguration IntakePositionConfigs = new TalonFXConfiguration();
    
    SparkFlexConfig IntakeRollerConfigs = new SparkFlexConfig();
    IntakeRollerMotor = new SparkFlex(Hardwaremap.IntakeRoller_CID, MotorType.kBrushless);
    IntakeRollerEncoder = IntakeRollerMotor.getEncoder();

    // voltVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    // voltPosition = new PositionVoltage(0, 0, true, 0, 0, false, false, false);

    IntakePositionController = new TalonFX(Hardwaremap.IntakePosition_CID, Hardwaremap.Intake_CBN);
    
   // IntakePositionEncoder = new CANcoder(Hardwaremap.IntakePositionCancoder_CID, Hardwaremap.Intake_CBN);
 

    //Intake Postition
    encoderOffset = Rotation2d.fromDegrees(SuperStructureConstants.IntakePosition_ENCODER_OFFSET);
    //Gains for Deploy 
    IntakePositionConfigs.Slot0.kP = .6; // An error of 1 rotation results in 2.4 V output
    IntakePositionConfigs.Slot0.kI = 0.2; // No output for integrated error
    IntakePositionConfigs.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output

    //Gains for Retract
    IntakePositionConfigs.Slot1.kP = 1.4; // An error of 1 rotation results in 2.4 V output
    IntakePositionConfigs.Slot1.kI = 0.2; // No output for integrated error
    IntakePositionConfigs.Slot1.kD = 0; // A velocity of 1 rps results in 0.1 V output

    // set max output voltage limits speed - 14V is max output available 
    IntakePositionConfigs.Voltage.PeakForwardVoltage = SuperStructureConstants.IntakePosition_MAX_OUTPUT_VOLTS; 
    IntakePositionConfigs.Voltage.PeakReverseVoltage = SuperStructureConstants.IntakePosition_MAXNeg_OUTPUT_VOLTS;

    IntakePositionConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //TODO if position from Cancoder define which CanCoder / remote sensor to use for position feedback
    //HoodConfigs.Feedback.FeedbackRemoteSensorID = Hardwaremap.HoodCancoder_CID;
    //HoodConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    //HoodConfigs.Feedback.SensorToMechanismRatio = 1.0;

    IntakePositionConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    IntakePositionConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
    IntakePositionConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    IntakePositionConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.6;
    IntakePositionConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    IntakePositionConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    IntakePositionConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

    TalonUtils.applyTalonFxConfigs(IntakePositionController, IntakePositionConfigs);
    //IntakePositionController.setPosition(0, 0);

    // Intake Roller Configuartion 
     //SETUP SPEED CONTROL CONFIGS
        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    IntakeRollerConfigs.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    IntakeRollerConfigs.closedLoop.p(0.003); 
    IntakeRollerConfigs.closedLoop.i(0.002);
    IntakeRollerConfigs.closedLoop.d(0.001); 
  
    // set max output current limits TODO check stall current of speed / roller
    IntakeRollerConfigs.smartCurrentLimit(60);

    IntakeRollerConfigs.inverted(false);
    IntakeRollerConfigs.idleMode(IdleMode.kCoast);
    IntakeRollerConfigs.encoder.velocityConversionFactor(1);

    //Apply the configuration to the Spark Flex Controller
    IntakeRollerMotor.configure(IntakeRollerConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    IntakeRollerController = IntakeRollerMotor.getClosedLoopController();



    IntakePositionAppliedVolts = IntakePositionController.getMotorVoltage();
    IntakePositionCurrentAmpsStator = IntakePositionController.getStatorCurrent();
    IntakePositionCurrentAmpsSupply = IntakePositionController.getSupplyCurrent();
    IntakePositionVelocity = IntakePositionController.getVelocity();
    IntakePositionPosition = IntakePositionController.getPosition();
    
    //If you want to use a cancode use this definition 
    //HoodPosition = HoodEncoder.getPositionSinceBoot();
    IntakePositionAbsolutePosition = IntakePositionController.getPosition();
    IntakePositionTemp = IntakePositionController.getDeviceTemp();

   // Hood_pos_offset = HoodEncoder.getPosition().getValueAsDouble();


    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        IntakePositionAppliedVolts,
        IntakePositionCurrentAmpsStator,
        IntakePositionVelocity,
        IntakePositionPosition,
        IntakePositionTemp,
        IntakePositionCurrentAmpsSupply);

    IntakePositionController.optimizeBusUtilization();
    
  }



  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(IntakeControlIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        IntakePositionAppliedVolts,
        IntakePositionCurrentAmpsStator,
        IntakePositionVelocity,
        IntakePositionPosition,
        IntakePositionTemp,
        IntakePositionCurrentAmpsSupply
        );


    inputs.IntakeAppliedVolts = IntakePositionAppliedVolts.getValueAsDouble();
    inputs.IntakeAmpsStator = IntakePositionCurrentAmpsStator.getValueAsDouble();
    inputs.IntakeAmpsSupply = IntakePositionCurrentAmpsSupply.getValueAsDouble();
    inputs.IntakeTempCelcius = IntakePositionTemp.getValueAsDouble();
    inputs.IntakePositionDeg = Units.rotationsToDegrees(IntakePositionPosition.getValueAsDouble());
    //inputs.Hood_pos_offset = Hood_pos_offset;
    inputs.IntakePositionRot = IntakePositionController.getPosition().getValueAsDouble();
    //Use if using cancoder
    //inputs.HoodPositionRot = HoodEncoder.getPosition().getValueAsDouble() - Hood_pos_offset;
    inputs.IntakePositionRotREAL = IntakeRollerMotor.getEncoder().getPosition(); 
    inputs.IntakeVelocityRadPerSec = IntakeRollerMotor.getEncoder().getVelocity();
    inputs.IntakeAbsolutePositionDegrees =
        MathUtil.inputModulus(
            Rotation2d.fromRotations(IntakeRollerMotor.getEncoder().getPosition())
                .minus(encoderOffset)
                .getDegrees(),
            -180,
            180);

  }


  
  //Offset would be used when we need 
  @Override
  public void 
  setIntakePositionVoltagePosSlot0(double position) {
    IntakePositionController.setControl(voltIntakePositionSlot0.withPosition(position + SuperStructureConstants.Intake_pos_offset));
  }

  @Override
  public void 
  setIntakePositionVoltagePosSlot1(double position) {
    IntakePositionController.setControl(voltIntakePositionSlot1.withPosition(position + SuperStructureConstants.Intake_pos_offset));
  }
  
  @Override
  public  void setIntakePositionVolts(double volts) {
    IntakePositionController.setVoltage(volts * SuperStructureConstants.Intake_Position_MULTIPLIER);
  }


  //Offset would be used when we need 
  @Override
  public void setIntakeRollerVelocity(double percentDutyCycle) {
   // sparkSpeedController.setSetpoint(speed_RPM, ControlType.kVelocity);
   IntakeRollerMotor.set(percentDutyCycle * SuperStructureConstants.IntakeRollerMaxDutyCycle);
  }

 
}
