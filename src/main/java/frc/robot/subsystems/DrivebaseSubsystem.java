// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivebaseSubsystem extends SubsystemBase {
  
  private TalonSRX m_frontLeft, m_frontRight, m_backLeft, m_backRight;

  public DrivebaseSubsystem() {

    m_frontLeft = new TalonSRX(3);
    m_frontRight = new TalonSRX(1);
    m_backLeft = new TalonSRX(6);
    m_backRight = new TalonSRX(7);

  }

  public void driveMecanum(double speedY, double speedX, double rotX) {

    if(speedY >= 0.1 || speedY <= 0.1) {
      m_frontLeft.set(TalonSRXControlMode.PercentOutput, speedY);
      m_frontRight.set(TalonSRXControlMode.PercentOutput, -speedY);
      m_backLeft.set(TalonSRXControlMode.PercentOutput, speedY);
      m_backRight.set(TalonSRXControlMode.PercentOutput, -speedY);
    } else if(speedX >= 0.1 || speedX <= 0.1) {
      m_frontLeft.set(TalonSRXControlMode.PercentOutput, speedY);
      m_frontRight.set(TalonSRXControlMode.PercentOutput, -speedY);
      m_backLeft.set(TalonSRXControlMode.PercentOutput, -speedY);
      m_backRight.set(TalonSRXControlMode.PercentOutput, speedY);
    } else if(rotX >= 0.1 || rotX <= 0.1) {
      m_frontLeft.set(TalonSRXControlMode.PercentOutput, rotX);
      m_frontRight.set(TalonSRXControlMode.PercentOutput, rotX);
      m_backLeft.set(TalonSRXControlMode.PercentOutput, rotX);
      m_backRight.set(TalonSRXControlMode.PercentOutput, rotX);
    }
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
