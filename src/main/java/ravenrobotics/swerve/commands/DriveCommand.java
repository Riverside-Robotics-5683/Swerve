package ravenrobotics.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import ravenrobotics.swerve.Constants.SwerveConstants;
import ravenrobotics.swerve.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase
{
    private final DriveSubsystem subsystem;
    private final DoubleSupplier xInput, yInput, rInput;
    private final BooleanSupplier isFieldRelative;
    private final SlewRateLimiter xLimiter, yLimiter, rLimiter;
    
    public DriveCommand(DriveSubsystem subsystem, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rInput, BooleanSupplier isFieldRelative)
    {
        this.subsystem = subsystem;

        this.xInput = xInput;
        this.yInput = yInput;
        this.rInput = rInput;

        this.isFieldRelative = isFieldRelative;

        xLimiter = new SlewRateLimiter(3);
        yLimiter = new SlewRateLimiter(3);
        rLimiter = new SlewRateLimiter(3);

        addRequirements(subsystem);
    }

    @Override
    public void execute()
    {
        double x = xInput.getAsDouble();
        double y = yInput.getAsDouble();
        double r = rInput.getAsDouble();

        x *= SwerveConstants.kPhysialMaxSpeedMPS;
        y *= SwerveConstants.kPhysialMaxSpeedMPS;
        r *= SwerveConstants.kPhysialMaxSpeedMPS;

        x = xLimiter.calculate(x);
        y = yLimiter.calculate(y);
        r = rLimiter.calculate(r);

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, r);

        if (isFieldRelative.getAsBoolean())
        {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, subsystem.getYaw());
        }

        subsystem.setModuleStates(SwerveConstants.kDrivetrainKinematics.toSwerveModuleStates(speeds));
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.stopModules();
    }
}
