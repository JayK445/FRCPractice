// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivebaseSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup; 

/** An example command that uses an example subsystem. */
public class DrivebaseCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DrivebaseSubsystem m_subsystem;
  private DoubleSupplier ySpeed;
  private DoubleSupplier xSpeed;
  private DoubleSupplier zRotation;
  private double m_Time;
  private double duration;
  private ShuffleboardTab tab = Shuffleboard.getTab("Drivebase");

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public DrivebaseCommand(DrivebaseSubsystem subsystem, DoubleSupplier ySpeed, DoubleSupplier xSpeed, DoubleSupplier zRotation, double duration) {
    m_subsystem = subsystem;
    this.ySpeed = ySpeed;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    this.duration = duration;

    m_Time = Timer.getFPGATimestamp();
    tab = Shuffleboard.getTab("Drivebase");
    tab.addNumber("ySpeed", this.ySpeed);
    tab.addNumber("xSpeed", this.xSpeed);
    tab.addNumber("zRotation", this.zRotation);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  public void InvertMotors(Boolean isPressed){
    if (isPressed){
      m_subsystem.InvertMotors();
    }
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    m_subsystem.drive(ySpeed.getAsDouble(), xSpeed.getAsDouble(), zRotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() >= m_Time +  duration;
  }
}