// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ArmSubsystem.Modes; 

/** An example command that uses an example subsystem. */
public class SequentialCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DrivebaseSubsystem m_DrivebaseSubsystem;
  private ArmSubsystem m_ArmSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public SequentialCommand(DrivebaseSubsystem drivebaseSubsystem, ArmSubsystem armSubsystem, Command selectedCommand) {
    m_DrivebaseSubsystem = drivebaseSubsystem;
    m_ArmSubsystem = armSubsystem;
    addCommands(selectedCommand, new TimedDrivebaseCommand(m_DrivebaseSubsystem, 0.3, 0, 0, 1.5), 
    new TimedDrivebaseCommand(m_DrivebaseSubsystem, 0.25, 0.3, 0.15, 3), Commands.parallel(
    new TimedDrivebaseCommand(m_DrivebaseSubsystem, -0.1, -0.1, 0, 1), new ArmCommand(m_ArmSubsystem, 90, 0.5, Modes.ON))
    );
  }
}