package ravenrobotics.swerve.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import ravenrobotics.swerve.subsystems.DriveSubsystem;

public class ZeroCanCodersCommand extends CommandBase
{
    private final DriveSubsystem subsystem;

    private boolean isFinished = false;

    public ZeroCanCodersCommand(DriveSubsystem subsystem)
    {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {
        subsystem.zeroCancoders();

        isFinished = true;
    }

    @Override
    public boolean isFinished()
    {
        return isFinished;
    }
}
