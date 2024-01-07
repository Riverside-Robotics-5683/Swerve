package ravenrobotics.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import ravenrobotics.swerve.Constants.SwerveModuleConstants;
import ravenrobotics.swerve.subsystems.SwerveSubsystem;

public class DriveCommand extends Command
{
    private final SwerveSubsystem swerveSubsystem;
    
    private final DoubleSupplier translationX, translationY, rotation, maxSpeed;
    private final BooleanSupplier isFieldRelative;

    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    public DriveCommand(SwerveSubsystem subsystem, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier isFieldRelative, DoubleSupplier maxSpeed)
    {
        this.swerveSubsystem = subsystem;

        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;

        this.isFieldRelative = isFieldRelative;
        this.maxSpeed = maxSpeed;

        this.xLimiter = new SlewRateLimiter(100.0);
        this.yLimiter = new SlewRateLimiter(100.0);
        this.turnLimiter = new SlewRateLimiter(100.0);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute()
    {
        if (isFieldRelative.getAsBoolean())
        {
            swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * SwerveModuleConstants.kMaxSpeedMPS,
                modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * SwerveModuleConstants.kMaxSpeedMPS,
                modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * SwerveModuleConstants.kMaxSpeedMPS,
                Rotation2d.fromDegrees(swerveSubsystem.getYaw())
                ));
        }
        else
        {
            swerveSubsystem.drive(new ChassisSpeeds(
                modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * SwerveModuleConstants.kMaxSpeedMPS,
                modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * SwerveModuleConstants.kMaxSpeedMPS,
                modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * SwerveModuleConstants.kMaxSpeedMPS
            ));
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    private double modifyAxis(double axisValue, double speedModifier, SlewRateLimiter limiter)
    {
        axisValue = MathUtil.applyDeadband(axisValue, 0.01);
        //no idea what this does lmao
        axisValue = Math.copySign(axisValue * axisValue, axisValue);
        axisValue *= speedModifier;

        axisValue = limiter.calculate(axisValue);
        if (Math.abs(axisValue) * SwerveModuleConstants.kMaxSpeedMPS <= SwerveModuleConstants.kMaxSpeedMPS * 0.01)
        {
            axisValue = 0.0;
        }

        return axisValue;
    }
}
