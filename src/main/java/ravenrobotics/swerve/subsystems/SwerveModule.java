package ravenrobotics.swerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import ravenrobotics.swerve.Constants;
import ravenrobotics.swerve.Constants.SwerveConstants;
import ravenrobotics.swerve.util.CTREConversions;

public class SwerveModule
{
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final CANCoder absoluteEncoder;
    private final boolean isAbsEncoderReversed;

    private final PIDController turningController;

    private final boolean isDriveReversed;
    private final boolean isTurnReversed;

    public SwerveModule(int driveID, int turnID, int absoluteID, boolean isDriveReversed, boolean isTurnReversed, boolean isAbsEncoderReversed)
    {
        this.driveMotor = new TalonFX(driveID);
        this.turnMotor = new TalonFX(turnID);
        this.isDriveReversed = isDriveReversed;
        this.isTurnReversed = isTurnReversed;
        
        this.absoluteEncoder = new CANCoder(absoluteID);
        this.isAbsEncoderReversed = isAbsEncoderReversed;

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

        double turnVoltage = turningController.calculate(getTurnMotorPosition(), state.angle.getRadians()) * 12;
        double driveVoltage = state.speedMetersPerSecond / SwerveConstants.kPhysialMaxSpeedMPS * 12;

        driveVoltage = MathUtil.clamp(driveVoltage, -12, 12);
        turnVoltage = MathUtil.clamp(turnVoltage, -12, 12);

        driveMotor.set(ControlMode.Current, driveVoltage);
        turnMotor.set(ControlMode.Current, turnVoltage);
    }

    public double getDriveMotorPosition()
    {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getTurnMotorPosition()
    {
        return turnMotor.getSelectedSensorPosition();
    }

    public double getDriveMotorVelocity()
    {
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurnMotorVelocity()
    {
        return turnMotor.getSelectedSensorVelocity();
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
        return absoluteEncoder.getPosition();
    }

    public void resetTurnMotorPosition()
    {
        turnMotor.setSelectedSensorPosition(CTREConversions.canCodertoTalonFX(getCanCoderPosition()));
    }

    public void stopMotors()
    {
        driveMotor.set(ControlMode.Current, 0);
        turnMotor.set(ControlMode.Current, 0);
    }
}
