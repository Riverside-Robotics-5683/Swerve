package ravenrobotics.swerve.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import ravenrobotics.swerve.Constants.SwerveModuleConstants;
import ravenrobotics.swerve.util.BetterCSOptimization;
import ravenrobotics.swerve.util.SwerveModuleConfig;

public class SwerveModule
{
    //Motors
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final boolean isDriveMotorInverted;
    private final boolean isAngleMotorInverted;
    //CANcoder; absolute encoder used for getting position.
    private final CANcoder absoluteEncoder;
    private final double absoluteEncoderOffset;
    private final boolean isAbsoluteEncoderInverted;

    private final PIDController angleController = new PIDController(SwerveModuleConstants.kAngleP, SwerveModuleConstants.kAngleI, SwerveModuleConstants.kAngleD);

    //Voltage calculators for the motors
    private PositionVoltage angleVoltage = new PositionVoltage(0);
    private VelocityVoltage driveVoltage = new VelocityVoltage(0);

    //Current state of the module
    private SwerveModulePosition currentState = new SwerveModulePosition();
    //StatusSignals for current state
    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> anglePosition;
    private StatusSignal<Double> angleVelocity;
    private BaseStatusSignal[] moduleSignals;

    private final String id;

    /**
     * Creates a new SwerveModule for driving a singular module.
     * @param swerveConfig The configuration for the SwerveModule.
     * @param CANbus The CANbus to use, e.g. "rio", "canivore"
     */
    public SwerveModule(SwerveModuleConfig swerveConfig, String CANbus, String id)
    {
        //Setup absolute encoder
        absoluteEncoder = new CANcoder(swerveConfig.absoluteEncoderID, CANbus);
        this.absoluteEncoderOffset = swerveConfig.absoluteEncoderOffset;
        this.isAbsoluteEncoderInverted = swerveConfig.isAbsoluteEncoderInverted;
        configAbsoluteEncoder();

        //Setup angle motor
        angleMotor = new TalonFX(swerveConfig.driveMotorID, CANbus);
        this.isAngleMotorInverted = swerveConfig.isAngleMotorInverted;
        configAngleMotor();

        //Setup drive motor
        driveMotor = new TalonFX(swerveConfig.angleMotorID, CANbus);
        this.isDriveMotorInverted = swerveConfig.isDriveMotorInverted;
        configDriveMotor();

        //Setup the signals
        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        anglePosition = angleMotor.getPosition();
        angleVelocity = angleMotor.getVelocity();

        moduleSignals = new BaseStatusSignal[4];
        moduleSignals[0] = drivePosition;
        moduleSignals[1] = driveVelocity;
        moduleSignals[2] = anglePosition;
        moduleSignals[3] = angleVelocity;

        this.id = id;

        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Set the module to the target state.
     * @param targetState The target state to reach.
     */
    public void setTargetState(SwerveModuleState targetState)
    {
        //Optimize the target state
        targetState = BetterCSOptimization.optimize(targetState, currentState.angle);
        //Set the angle motor state
        double steerVoltage = angleController.calculate(getAngleMotorRotations(true), targetState.angle.getRadians()) * 10;
        //double targetAngle = targetState.angle.getRotations();
        steerVoltage = MathUtil.clamp(steerVoltage, -10, 10);
        angleMotor.setVoltage(steerVoltage);
        //angleMotor.setControl(angleVoltage.withPosition(targetAngle).withSlot(0));
        SmartDashboard.putNumber(id + " IE", angleMotor.getPosition().refresh().getValue());
        //Set the drive motor state
        double targetDrive = targetState.speedMetersPerSecond;
        driveMotor.setControl(driveVoltage.withVelocity(targetDrive));
    }

    /**
     * Immediately stop the motors.
     */
    public void stopMotors()
    {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    /**
     * Get the current position of the module.
     * @param toRefresh Should the signals be refreshed before returning the current position.
     * @return The current position of the module as a SwerveModulePosition.
     */
    public SwerveModulePosition getModulePosition(boolean toRefresh)
    {
        //Refresh the signals if asked
        if (toRefresh)
        {
            drivePosition.refresh();
            driveVelocity.refresh();
            anglePosition.refresh();
            angleVelocity.refresh();
        }

        //Get the drive and angle motor rotations (latency-compenstated with velocity)
        double driveRotations = BaseStatusSignal.getLatencyCompensatedValue(drivePosition, driveVelocity);
        double angleRotations = BaseStatusSignal.getLatencyCompensatedValue(anglePosition, angleVelocity);

        //Update the currentState object.
        currentState.distanceMeters = driveRotations;
        currentState.angle = Rotation2d.fromRotations(angleRotations);

        //Return the currentState
        return currentState;
    }

    /**
     * Get the angle motor position in rotations.
     * @param toRefresh Should the method refresh the StatusSignal before returning the value.
     * @return The rotations as a double.
     */
    public double getAngleMotorRotations(boolean toRefresh)
    {
        if (toRefresh)
        {
            anglePosition.refresh();
        }

        return anglePosition.getValue();
    }

    /**
     * Get the module's current state.
     * @param toRefresh Should the signals be refreshed before returning the current state.
     * @return The current state of the module as a SwerveModuleState.
     */
    public SwerveModuleState getModuleState(boolean toRefresh)
    {
        if (toRefresh)
        {
            drivePosition.refresh();
            driveVelocity.refresh();
        }

        return new SwerveModuleState(driveVelocity.getValue(), Rotation2d.fromRotations(anglePosition.getValue()));
    }

    /**
     * Get the current motor positions and velocities as StatusSignals.
     * @return The signals as an array.
     */
    public BaseStatusSignal[] getStatusSignals()
    {
        return moduleSignals;
    }

    /**
     * Configure the absolute encoder.
     */
    private void configAbsoluteEncoder()
    {
        //Factory reset encoder
        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
        var absoluteEncoderConfig = new CANcoderConfiguration();
        //Set offset, whether it's inverted, and set the range
        absoluteEncoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        absoluteEncoderConfig.MagnetSensor.SensorDirection = isAbsoluteEncoderInverted ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
        absoluteEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        //Apply config
        absoluteEncoder.getConfigurator().apply(absoluteEncoderConfig);
    }

    /**
     * Get the angle from the absolute encoder.
     * @return A Rotation2d with the angle.
     */
    public Rotation2d getEncoderAngle()
    {
        return Rotation2d.fromRotations(absoluteEncoder.getPosition().refresh().getValue());
    }

    /**
     * Get the angle from the angle motor.
     * @return A Rotation2d with the angle.
     */
    public Rotation2d getAngle()
    {
        return Rotation2d.fromRotations(angleMotor.getPosition().refresh().getValue());
    }

    /**
     * Get the speed from the drive motor.
     * @return The speed as a double.
     */
    public double getSpeed()
    {
        return driveMotor.getVelocity().refresh().getValue();
    }

    public double getAbsoluteEncoderRotations()
    {
        return absoluteEncoder.getPosition().refresh().getValue();
    }

    /**
     * Configure the angle motor.
     */
    private void configAngleMotor()
    {
        //Factory reset motor
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());
        var angleMotorConfig = new TalonFXConfiguration();
        //Feedforward Config (not used)
        angleMotorConfig.Slot0.kS = 0.0;
        angleMotorConfig.Slot0.kV = 0.0;
        //PID Config
        angleMotorConfig.Slot0.kP = SwerveModuleConstants.kAngleP;
        angleMotorConfig.Slot0.kI = SwerveModuleConstants.kAngleI;
        angleMotorConfig.Slot0.kD = SwerveModuleConstants.kAngleD;
        angleMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
        //Current Limiting
        angleMotorConfig.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants.kAngleContinuousCurrentLimit;
        angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        angleMotorConfig.CurrentLimits.SupplyCurrentThreshold = SwerveModuleConstants.kAnglePeakCurrentLimit;
        angleMotorConfig.CurrentLimits.SupplyTimeThreshold = SwerveModuleConstants.kAnglePeakCurrentDuration;
        angleMotorConfig.Voltage.PeakForwardVoltage = 10;
        angleMotorConfig.Voltage.PeakReverseVoltage = -10;
        //Feedback (encoder)
        angleMotorConfig.Feedback.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID();
        angleMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        angleMotorConfig.Feedback.FeedbackRotorOffset = absoluteEncoderOffset;
        angleMotorConfig.Feedback.RotorToSensorRatio = 1;
        angleMotorConfig.Feedback.SensorToMechanismRatio = SwerveModuleConstants.kAngleGearRatio;
        //Output Settings
        angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        angleMotorConfig.MotorOutput.Inverted = isAngleMotorInverted? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        //Apply new config
        angleMotor.getConfigurator().apply(angleMotorConfig);
        //Wait for config to apply, then set the motor's current read position to the encoder's.
        Timer.delay(0.5);
        angleMotor.setPosition(absoluteEncoder.getPosition().refresh().getValue());
    }

    /**
     * Configure the drive motor.
     */
    private void configDriveMotor()
    {
        //Factory reset motor
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        var driveMotorConfig = new TalonFXConfiguration();
        //Feedforward Config
        driveMotorConfig.Slot0.kV = SwerveModuleConstants.kDriveV;
        driveMotorConfig.Slot0.kS = SwerveModuleConstants.kDriveS;
        //PID Config
        driveMotorConfig.Slot0.kP = SwerveModuleConstants.kDriveP;
        driveMotorConfig.Slot0.kI = SwerveModuleConstants.kDriveI;
        driveMotorConfig.Slot0.kD = SwerveModuleConstants.kDriveD;
        //Current Limiting
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants.kDriveContinuousCurrentLimit;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentThreshold = SwerveModuleConstants.kDrivePeakCurrentLimit;
        driveMotorConfig.CurrentLimits.SupplyTimeThreshold = SwerveModuleConstants.kDrivePeakCurrentDuration;
        //Gear Ratio
        driveMotorConfig.Feedback.SensorToMechanismRatio = SwerveModuleConstants.kDriveGearRatio;
        //Output Settings
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveMotorConfig.MotorOutput.Inverted = isDriveMotorInverted? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        //Apply new config
        driveMotor.getConfigurator().apply(driveMotorConfig);
        //Set encoder to 0
        driveMotor.setPosition(0.0);
    }
}
