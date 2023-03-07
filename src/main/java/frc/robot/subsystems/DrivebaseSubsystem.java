// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants.Ports;

public class DrivebaseSubsystem extends SubsystemBase {
  private MecanumDrive drivebase;  
  private WPI_PigeonIMU gyro;
  private WPI_TalonSRX frontLeft, frontRight, backLeft, backRight;
  private DoubleSupplier xDoubleSupplier, yDoubleSupplier, zDoubleSupplier;
  private LinearFilter lowPassFilter;
  private DrivebaseModes mode;
  private ShuffleboardTab shuffleboard;
  private double statorLimit;
  public enum DrivebaseModes{MANUAL, REVERSING}

  public DrivebaseSubsystem() {
    // construct motors
    frontLeft = new WPI_TalonSRX(Ports.FRONT_LEFT_MOTOR_PORT);
    frontRight = new WPI_TalonSRX(Ports.FRONT_RIGHT_MOTOR_PORT);
    backLeft = new WPI_TalonSRX(Ports.BACK_LEFT_MOTOR_PORT);
    backRight = new WPI_TalonSRX(Ports.BACK_RIGHT_MOTOR_PORT);
    gyro = new WPI_PigeonIMU(frontLeft);
    drivebase = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    lowPassFilter = LinearFilter.movingAverage(5)
    statorCurrent = motor.getStatorCurrent();
    filterOutput = lowPassFilter.calculate(0.02);

    statorLimit = 0.5;
    shuffleboard = Shuffleboard.getTab("Drivebase Subsystem");
    shuffleboard.add("Stator Current Limit", statorLimit);
  }

  public void drivePeriodic(double xSpeed, double ySpeed, double zRotation){
    drivebase.driveCartesian(xSpeed, ySpeed, zRotation, gyro.getRotation2d());
  }

  public WPI_PigeonIMU getGyro(){
    return gyro;
  }

  public void setMode(DrivebaseModes mode){
    this.mode = mode;
  }

  public void setInputs(DoubleSupplier xDoubleSupplier, DoubleSupplier yDoubleSupplier, DoubleSupplier zDoubleSupplier){
    this.xDoubleSupplier = xDoubleSupplier;
    this.yDoubleSupplier = yDoubleSupplier;
    this.zDoubleSupplier = zDoubleSupplier;
  }

  public DrivebaseModes advanceMode(){
    if (lowPassFilter.calculate(0.02) >= statorLimit){
        return DrivebaseModes.REVERSING;
    }

    switch(mode){
        case MANUAL:
            return DrivebaseModes.MANUAL;
        case REVERSING:
            return DrivebaseModes.REVERSING;
        default:
            return DrivebaseModes.MANUAL;
    }
  }

  private void applyMode(DrivebaseModes modes){
    switch(modes){
      case MANUAL:
        drivePeriodic(xDoubleSupplier.getAsDouble(), yDoubleSupplier.getAsDouble(), zDoubleSupplier.getAsDouble());
      case REVERSING:
        drivePeriodic(statorLimit, statorLimit, statorLimit);
      default:
        drivePeriodic(xDoubleSupplier.getAsDouble(), yDoubleSupplier.getAsDouble(), zDoubleSupplier.getAsDouble());
    }
  }

  @Override
  public void periodic() {
    mode = advanceMode();
    applyMode(mode);
  }

  @Override
  public void simulationPeriodic() {}
}
