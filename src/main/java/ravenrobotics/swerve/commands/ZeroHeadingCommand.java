package ravenrobotics.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import ravenrobotics.swerve.subsystems.DriveSubsystem;

public class ZeroHeadingCommand extends CommandBase
{
    private final DriveSubsystem subsystem;
    
    private boolean isFinished = false;

    public ZeroHeadingCommand(DriveSubsystem subsystem)
    {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {
        subsystem.zeroYaw();

        isFinished = true;
    }

    @Override
    public boolean isFinished()
    {
        return isFinished;
    }
}
