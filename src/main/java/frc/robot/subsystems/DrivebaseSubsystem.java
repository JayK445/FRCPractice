// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class DrivebaseSubsystem extends SubsystemBase {
  private final ShuffleboardTab driveBaseShuffleboard = Shuffleboard.getTab("Drive");
  private MecanumDrive drivebase;  
  private WPI_PigeonIMU gyro;
  private WPI_TalonSRX frontLeft, frontRight, backLeft, backRight;
  private LinearFilter filter;
  private double filterOutput;
  private double statorCurrent;

  public DrivebaseSubsystem() {
    // construct motors
    frontLeft = new WPI_TalonSRX(Ports.FRONT_LEFT_MOTOR_PORT);
    frontRight = new WPI_TalonSRX(Ports.FRONT_RIGHT_MOTOR_PORT);
    backLeft = new WPI_TalonSRX(Ports.BACK_LEFT_MOTOR_PORT);
    backRight = new WPI_TalonSRX(Ports.BACK_RIGHT_MOTOR_PORT);
    gyro = new WPI_PigeonIMU(frontLeft);
    filter = LinearFilter.movingAverage(5);
    statorCurrent = (frontLeft.getStatorCurrent() + frontRight.getStatorCurrent() + 
    backLeft.getStatorCurrent() + backRight.getStatorCurrent())/4;
    filterOutput = 0;

    drivebase = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
  
    driveBaseShuffleboard.addNumber("Stator Current", () -> statorCurrent);
    driveBaseShuffleboard.addNumber("FilterOutput", () -> filterOutput);
    
  }

  public void drive (double xSpeed, double ySpeed, double zRotation){
    drivebase.driveCartesian(xSpeed, ySpeed, zRotation, gyro.getRotation2d());
  }

  public WPI_PigeonIMU getGyro(){
    return gyro;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
