// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import ravenrobotics.swerve.Constants.DSConstants;
import ravenrobotics.swerve.subsystems.DriveSubsystem;

public class RobotContainer
{
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final CommandXboxController driveController = new CommandXboxController(DSConstants.DRIVER_PORT);
}
