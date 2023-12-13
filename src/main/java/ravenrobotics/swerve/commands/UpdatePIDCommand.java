package ravenrobotics.swerve.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import ravenrobotics.swerve.subsystems.DriveSubsystem;

public class UpdatePIDCommand extends CommandBase
{
    private final DriveSubsystem subsystem;

    private boolean isFinished = false;

    public UpdatePIDCommand(DriveSubsystem subsystem)
    {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {
        double p, i, d;

        p = SmartDashboard.getNumber("P", 4);
        i = SmartDashboard.getNumber("I", 0);
        d = SmartDashboard.getNumber("D", 0);

        subsystem.updatePIDs(p, i, d);
        isFinished = true;
    }

    @Override
    public boolean isFinished()
    {
        return isFinished;
    }
}
