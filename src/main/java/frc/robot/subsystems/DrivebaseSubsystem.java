// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivebaseSubsystem extends SubsystemBase {
  
  private MecanumDrive mecanumDrive;
  private MotorController m_frontLeft, m_frontRight, m_backLeft, m_backRight;

  public DrivebaseSubsystem() {

    mecanumDrive = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);

  }

  

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
