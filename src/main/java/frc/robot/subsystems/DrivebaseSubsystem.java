// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivebaseSubsystem extends SubsystemBase {
  
  private WPI_TalonSRX m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  private MecanumDrive drivebase;
  public DrivebaseSubsystem() {
    m_frontLeft = new WPI_TalonSRX(3);
    m_frontRight = new WPI_TalonSRX(1);
    m_backLeft = new WPI_TalonSRX(6);
    m_backRight = new WPI_TalonSRX(7);
    drivebase = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);
  }

  public void drive(double ySpeed) {
   MecanumDrive.driveCartesian()
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
