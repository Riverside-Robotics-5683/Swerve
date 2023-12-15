// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import ravenrobotics.swerve.Constants.DSConstants;
import ravenrobotics.swerve.commands.DriveCommand;
import ravenrobotics.swerve.commands.UpdatePIDCommand;
import ravenrobotics.swerve.commands.ZeroCanCodersCommand;
import ravenrobotics.swerve.commands.ZeroHeadingCommand;
import ravenrobotics.swerve.subsystems.DriveSubsystem;

public class RobotContainer
{
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final CommandXboxController driveController = new CommandXboxController(DSConstants.DRIVER_PORT);

  public RobotContainer()
  {
    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, () -> -driveController.getLeftY(), driveController::getLeftX, driveController::getRightX, () -> true));
    SmartDashboard.putData(new UpdatePIDCommand(driveSubsystem));

    configureBindings();
  }

  private void configureBindings()
  {
    driveController.leftBumper().onTrue(new ZeroHeadingCommand(driveSubsystem));
    //driveController.rightBumper().onTrue(new ZeroCanCodersCommand(driveSubsystem));
  }

  public Command getZeroEncoders()
  {
    return new ZeroCanCodersCommand(driveSubsystem);
  }
}
