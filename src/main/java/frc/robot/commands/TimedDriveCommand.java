// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TimedDriveCommande extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivebaseSubsystem m_subsystem;
  private double xSpeed;
  private double ySpeed;
  private double zRotation;
  private double time;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TimedDriveCommand(DrivebaseSubsystem subsystem, double ySpeed, double xSpeed, double zRotation, double time) {
    m_subsystem = subsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.zRotation = zRotation;
    this.time = Time; 

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);

  }


  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Time.getFPGATimestamp();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(ySpeed, xSpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Time.getFPGATimestamp() - 5 >= time;
  }
}
