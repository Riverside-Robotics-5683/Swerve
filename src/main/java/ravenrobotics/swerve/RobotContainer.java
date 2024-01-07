// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import ravenrobotics.swerve.Constants.DriverStationConstants;
import ravenrobotics.swerve.Constants.SwerveModuleConstants;
import ravenrobotics.swerve.commands.DriveCommand;
import ravenrobotics.swerve.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.kDriverPort);

    public RobotContainer()
    {
        swerveSubsystem.setDefaultCommand(new DriveCommand(
            swerveSubsystem,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> true,
            () -> SwerveModuleConstants.kMaxSpeedMPS));

        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /** Use this method to define your trigger->command mappings. */
    private void configureBindings()
    {
        driverController.y().onTrue(new InstantCommand(() -> swerveSubsystem.zeroYaw(), swerveSubsystem));
        driverController.b().onTrue(new InstantCommand(() -> swerveSubsystem.stopModules(), swerveSubsystem));
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // TODO: Implement properly
        return null;
    }
}
