// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand; 

/** An example command that uses an example subsystem. */
public class SequentialCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private TimedDrivebaseCommand m_TimedDrivebaseCommand;
  private InvertMotors m_invertMotors;
  private ArmCommand m_armCommand;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public SequentialCommand(TimedDrivebaseCommand m_TimedDrivebaseCommand, InvertMotors m_invertMotors, ArmCommand m_armCommand) {
    this.m_TimedDrivebaseCommand = m_TimedDrivebaseCommand;
    this.m_invertMotors = m_invertMotors;
    this.m_armCommand = m_armCommand;
    
    addCommands(m_TimedDrivebaseCommand, m_invertMotors, new WaitCommand(0.5), m_TimedDrivebaseCommand, m_armCommand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}