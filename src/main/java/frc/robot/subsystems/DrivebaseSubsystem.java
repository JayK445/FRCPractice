// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DrivebaseSubsystem extends SubsystemBase {
  
  private MecanumDrive mecanumDrive;
  private TalonSRX m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  private boolean invertedMotor;
  public DrivebaseSubsystem() {
    // construct motors
    
    m_frontLeft = new TalonSRX(3);//port numbers in parentheses
    m_frontRight = new TalonSRX(1);
    m_backLeft = new TalonSRX(6);
    m_backRight = new TalonSRX(7);
    
  }

  public void driveCartesian (double ySpeed, double xSpeed, double zRotation){
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);

    m_frontLeft.set(ControlMode.PercentOutput, ySpeed);
    m_frontRight.set(ControlMode.PercentOutput, ySpeed);
    m_backLeft.set(ControlMode.PercentOutput, ySpeed);
    m_backRight.set(ControlMode.PercentOutput, ySpeed);
  }
  
  public void invertMotors(){
    invertedMotor = m_frontLeft.getInverted();
    m_frontRight.setInverted(invertedMotor);
    m_backRight.setInverted(invertedMotor);
    
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
