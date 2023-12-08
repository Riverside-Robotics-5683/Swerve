package ravenrobotics.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import ravenrobotics.swerve.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase
{
    private final DriveSubsystem subsystem;
    private final DoubleSupplier xInput, yInput, rInput;
    private final BooleanSupplier isFieldRelative;

    
    public DriveCommand(DriveSubsystem subsystem, DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rInput, BooleanSupplier isFieldRelative)
    {
        this.subsystem = subsystem;

        this.xInput = xInput;
        this.yInput = yInput;
        this.rInput = rInput;

        this.isFieldRelative = isFieldRelative;

        addRequirements(subsystem);
    }
}
