// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

// An example command that uses an example subsystem.
public class ArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ArmSubsystem m_subsystem;
  private double target;
  private double duration;
  private Timer m_Timer;
  //private double m_Time;

    
  /**
   * Creates a new ExampleCommand.
   * @param subsystem The subsystem used by this command.
   */

  public ArmCommand(ArmSubsystem subsystem, double target, double duration) {
    //m_Time = Timer.getFPGATimestamp();
    m_subsystem = subsystem;
    this.target = target;
    this.duration = duration;
    m_Timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    //return Timer.getFPGATimestamp() >= m_Time + duration;
  }
}
