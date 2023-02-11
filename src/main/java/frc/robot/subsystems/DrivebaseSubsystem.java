// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivebaseSubsystem extends SubsystemBase {
  private TalonSRX m_frontLeft, m_frontRight, m_backLeft, m_backRight;

  public DrivebaseSubsystem() {
    // construct motors
    m_frontLeft = new TalonSRX(3);
    m_frontRight = new TalonSRX(1);
    m_backLeft = new TalonSRX(6);
    m_backRight = new TalonSRX(7);
    //mecanumDrive = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);
  }

  public void drive (double ySpeed, double xSpeed, double zRotation){
    m_frontLeft.set(ControlMode.PercentOutput, ySpeed);
    m_frontRight.set(ControlMode.PercentOutput, ySpeed);
    m_backLeft.set(ControlMode.PercentOutput, ySpeed);
    m_backRight.set(ControlMode.PercentOutput, ySpeed);
  }

  public void InvertMotors(){
    m_frontRight.setInverted(true);
    m_backRight.setInverted(true);
  }

  public void UninvertMotors(){
    m_frontRight.setInverted(false);
    m_backRight.setInverted(false);
  }

  public void ToggleInvert(){
    m_frontRight.setInverted(m_frontRight.getInverted());
    m_backRight.setInverted(m_backRight.getInverted());
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
