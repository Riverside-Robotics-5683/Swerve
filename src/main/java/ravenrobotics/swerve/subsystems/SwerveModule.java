package ravenrobotics.swerve.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import ravenrobotics.swerve.Constants;
import ravenrobotics.swerve.Constants.MotorConstants;
import ravenrobotics.swerve.Constants.SwerveConstants;
import ravenrobotics.swerve.util.CTREConversions;

public class SwerveModule
{
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final CANcoder absoluteEncoder;
    private final double absEncoderStartPosition;

    private final PIDController turningController;

    private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    private final boolean isDriveReversed;
    private final boolean isTurnReversed;

    private final PositionVoltage angleSetter = new PositionVoltage(0);

    public SwerveModule(int driveID, int turnID, int absoluteID, boolean isDriveReversed, boolean isTurnReversed, double absEncoderStartPosition)
    {
        this.driveMotor = new TalonFX(driveID);
        this.turnMotor = new TalonFX(turnID);
        this.isDriveReversed = isDriveReversed;
        this.isTurnReversed = isTurnReversed;
    
        this.turnConfig.Slot0.kP = SwerveConstants.kP;
        this.turnConfig.Slot0.kI = SwerveConstants.kI;
        this.turnConfig.Slot0.kD = SwerveConstants.kD;
        this.turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        this.turnConfig.Feedback.FeedbackRemoteSensorID = absoluteID;
        this.turnConfig.Feedback.FeedbackRotorOffset = absEncoderStartPosition;

        this.turnConfig.Voltage.PeakForwardVoltage = MotorConstants.kMaxVoltage;
        this.turnConfig.Voltage.PeakReverseVoltage = -MotorConstants.kMaxVoltage;
        this.turnConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kAngleGearRatio;

        this.driveConfig.Voltage.PeakForwardVoltage = MotorConstants.kMaxVoltage;
        this.driveConfig.Voltage.PeakReverseVoltage = -MotorConstants.kMaxVoltage;
        this.driveConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kDriveGearRatio;

        this.turnMotor.getConfigurator().apply(turnConfig);
        this.driveMotor.getConfigurator().apply(driveConfig);

        this.absoluteEncoder = new CANcoder(absoluteID);
        this.absEncoderStartPosition = absEncoderStartPosition;

        this.turningController = new PIDController(Constants.SwerveConstants.kP, Constants.SwerveConstants.kI, Constants.SwerveConstants.kD);
        this.turningController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setup()
    {
        driveMotor.setInverted(isDriveReversed);
        turnMotor.setInverted(isTurnReversed);
    }

    public void setState(SwerveModuleState state)
    {
        if (Math.abs(state.speedMetersPerSecond) < 0.001)
        {
            stopMotors();
            return;
        }
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(CTREConversions.talonFXToDegrees(getTurnMotorPosition())));

        double driveVoltage = state.speedMetersPerSecond / SwerveConstants.kPhysialMaxSpeedMPS * MotorConstants.kMaxVoltage;
        driveVoltage = MathUtil.clamp(driveVoltage, -MotorConstants.kMaxVoltage,MotorConstants.kMaxVoltage);

        driveMotor.setVoltage(driveVoltage);
        turnMotor.setControl(angleSetter.withPosition(state.angle.getRotations()));
    }

    public double getDriveMotorPosition()
    {
        return driveMotor.getPosition().getValue();
    }

    public double getTurnMotorPosition()
    {
        return turnMotor.getPosition().getValue();
    }

    public double getDriveMotorVelocity()
    {
        return driveMotor.getVelocity().getValue();
    }

    public double getTurnMotorVelocity()
    {
        return turnMotor.getVelocity().getValue();
    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(getDriveMotorVelocity(), Rotation2d.fromDegrees(CTREConversions.talonFXToDegrees(getTurnMotorPosition())));
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(getDriveMotorPosition(), Rotation2d.fromDegrees(CTREConversions.talonFXToDegrees(getTurnMotorPosition())));
    }

    public double getCanCoderPosition()
    {
        return absoluteEncoder.getPosition().getValue();
    }

    public void setPIDController(double p, double i, double d)
    {
        turningController.setPID(p, i, d);
    }

    public void resetTurnMotorPosition()
    {
        turnMotor.setRotorPosition(CTREConversions.canCodertoTalonFX(getCanCoderPosition()));
        angleSetter.Position = CTREConversions.canCodertoTalonFX(getCanCoderPosition());
    }

    public void resetCanCoder()
    {
        absoluteEncoder.setPosition(0);
    }

    public void stopMotors()
    {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}
