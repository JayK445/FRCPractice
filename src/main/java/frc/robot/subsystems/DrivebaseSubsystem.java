// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.

public class DrivebaseSubsystem extends SubsystemBase {
  
  private MecanumDrive mecanumDrive;
  private TalonSRX m_frontLeft, m_frontRight, m_backLeft, m_backRight;

  public DrivebaseSubsystem(){
    m_frontLeft = new TalonSRX(3);
    m_backLeft = new TalonSRX(6);
    m_frontRight = new TalonSRX(1);
    m_backRight = new TalonSRX(7);
    mecanumDrive = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);
  }
  public void MecanumDriveMethod(){
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
  }
  

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
