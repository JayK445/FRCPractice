// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivebaseSubsystem extends SubsystemBase {
  private MecanumDrive drivebase;  private WPI_PigeonIMU gyro;
  private WPI_TalonSRX frontLeft, frontRight, backLeft, backRight;

  public DrivebaseSubsystem() {
    gyro = new WPI_PigeonIMU(frontLeft);
    // construct motors
    frontLeft = new WPI_TalonSRX(3);
    frontRight = new WPI_TalonSRX(1);
    backLeft = new WPI_TalonSRX(6);
    backRight = new WPI_TalonSRX(7);

    drivebase = new MecanumDrive(frontLeft, backRight, frontRight, backLeft);
  }

  public void drive (double ySpeed, double xSpeed, double zRotation){
    drivebase.driveCartesian(xSpeed, ySpeed, zRotation, gyro.getRotation2d());
    /*m_frontLeft.set(ControlMode.PercentOutput, ySpeed);
    m_frontRight.set(ControlMode.PercentOutput, ySpeed);
    m_backLeft.set(ControlMode.PercentOutput, ySpeed);
    m_backRight.set(ControlMode.PercentOutput, ySpeed);*/
  }

  public void InvertMotors(){
    frontRight.setInverted(true);
    backRight.setInverted(true);
  }

  public void UninvertMotors(){
    frontRight.setInverted(false);
    backRight.setInverted(false);
  }

  public void ToggleInvert(){
    frontRight.setInverted(frontRight.getInverted());
    backRight.setInverted(backRight.getInverted());
  }
  public WPI_PigeonIMU getGyro(){
    return gyro;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
