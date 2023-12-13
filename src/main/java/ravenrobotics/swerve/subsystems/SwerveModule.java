package ravenrobotics.swerve.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

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

    private final CANcoder absoluteEncoder;
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
        
        this.absoluteEncoder = new CANcoder(absoluteID);
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

        double turnVoltage = turningController.calculate(getTurnMotorPosition(), state.angle.getRadians()) * 5;
        double driveVoltage = state.speedMetersPerSecond / SwerveConstants.kPhysialMaxSpeedMPS * 5;

        driveVoltage = MathUtil.clamp(driveVoltage, -5, 5);
        turnVoltage = MathUtil.clamp(turnVoltage, -5, 5);

        driveMotor.setVoltage(driveVoltage);
        turnMotor.setVoltage(turnVoltage);
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
    }

    public void stopMotors()
    {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }
}
