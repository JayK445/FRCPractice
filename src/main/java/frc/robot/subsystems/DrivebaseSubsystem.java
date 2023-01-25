// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivebaseSubsystem extends SubsystemBase {

  private TalonSRX m_frontLeft, m_frontRight, m_backLeft, m_backRight;

  public DrivebaseSubsystem(){
    m_frontLeft = new TalonSRX(3);
    m_backLeft = new TalonSRX(6);
    m_frontRight = new TalonSRX(1);
    m_backRight = new TalonSRX(7);
    m_frontRight.setInverted(true);
    m_backRight.setInverted(true);
  }
  public void drive(double speed){
    m_frontLeft.set(ControlMode.PercentOutput, speed);
    m_backLeft.set(ControlMode.PercentOutput, speed);
    m_frontRight.set(ControlMode.PercentOutput, speed);
    m_backRight.set(ControlMode.PercentOutput, speed);
  }
  

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
