// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem.DrivebaseModes;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DrivebaseCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DrivebaseSubsystem m_subsystem;
  private DoubleSupplier ySpeed;
  private DoubleSupplier xSpeed;
  private DoubleSupplier zRotation;
  private double statorLimit;
  private ShuffleboardTab tab = Shuffleboard.getTab("Drivebase");

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public DrivebaseCommand(DrivebaseSubsystem subsystem, DoubleSupplier ySpeed, DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    m_subsystem = subsystem;
    this.ySpeed = ySpeed;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    statorLimit = 1.5;
    m_subsystem.setInputs(xSpeed, ySpeed, zRotation);

    tab = Shuffleboard.getTab("Drivebase");
    tab.addNumber("ySpeed", xSpeed);
    tab.addNumber("xSpeed", ySpeed);
    tab.addNumber("zRotation", zRotation);
    tab.addNumber("Gyro", () -> m_subsystem.getGyro().getAngle());
    tab.addNumber("Stator Limit", () -> statorLimit);
    tab.addNumber("Stator Current", () -> m_subsystem.getFLStatorCurrent());
    tab.addNumber("Filter Output", () -> m_subsystem.getFilterOutput());
    tab.addString("Mode", () -> m_subsystem.getMode().toString());

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setStatorLimit(statorLimit);
    m_subsystem.setMode(DrivebaseModes.REVERSING);
  }

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