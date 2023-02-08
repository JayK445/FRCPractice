// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem; 

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
  
  public SequentialCommand(DrivebaseSubsystem drivebaseSubsystem, ArmSubsystem armSubsystem) {
    m_DrivebaseSubsystem = drivebaseSubsystem;
    m_ArmSubsystem = armSubsystem;
    
    addCommands(new TimedDrivebaseCommand(m_DrivebaseSubsystem, 0.1, 0.5), new ToggleMotorInvert(m_DrivebaseSubsystem), 
    new TimedDrivebaseCommand(m_DrivebaseSubsystem, 0.25, 0.25), new WaitCommand(0.5), new ToggleMotorInvert(m_DrivebaseSubsystem), 
    parallel(new TimedDrivebaseCommand(m_DrivebaseSubsystem, -0.1, 0.5), new ArmCommand(m_ArmSubsystem, 90, 0.5)));
  }
}