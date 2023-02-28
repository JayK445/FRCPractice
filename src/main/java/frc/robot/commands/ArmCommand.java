// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Modes;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

// An example command that uses an example subsystem.
public class ArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ArmSubsystem m_subsystem;
  private double target;
  private double duration;
  private Timer m_Timer;
  private Modes mode;
  /**
   * Creates a new ExampleCommand.
   * @param subsystem The subsystem used by this command.
   */

  public ArmCommand(ArmSubsystem subsystem, double target, double duration, Modes mode) {
    m_subsystem = subsystem;
    this.target = target;
    this.duration = duration;
    this.mode = mode;
    m_Timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setMode(mode);
    m_subsystem.setDesiredAngle(target);
    m_Timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
  }  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() >= duration;
  }
}
