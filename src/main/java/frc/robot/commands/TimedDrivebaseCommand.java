// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivebaseSubsystem;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TimedDrivebaseCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DrivebaseSubsystem m_subsystem;
  //private double m_Time;
  private double duration;
  private Timer m_Timer;
  private double xSpeed, ySpeed, zRotation;
  private PathPlannerTrajectory path;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */ 
  
  public TimedDrivebaseCommand(DrivebaseSubsystem subsystem, double xSpeed, double ySpeed, double zRotation, double duration) {
    m_subsystem = subsystem;
    this.duration = duration;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;

    m_Timer = new Timer();
    path = PathPlanner.loadPath("New Path", null);
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    m_subsystem.drive(xSpeed, ySpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() >= duration;
  }
}