// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivebaseSubsystem extends SubsystemBase {
  
  private MecanumDrive mecanumDrive;
  private PWMVictorSPX m_frontLeft, m_frontRight, m_backLeft, m_backRight;

  public DrivebaseSubsystem() {
    // construct motors
    
    m_frontLeft = new PWMVictorSPX(3);//port numbers in parentheses
    m_frontRight = new PWMVictorSPX(1);
    m_backLeft = new PWMVictorSPX(6);
    m_backRight = new PWMVictorSPX(7);
    
    mecanumDrive = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);

  }

  public void driveCartesian (double ySpeed, double xSpeed, double zRotation){
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
