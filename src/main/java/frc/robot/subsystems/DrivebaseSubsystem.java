// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class DrivebaseSubsystem extends SubsystemBase {
  private MecanumDrive drivebase;  
  private AHRS gyro;
  private WPI_TalonSRX frontLeft, frontRight, backLeft, backRight;
  private DoubleSupplier xDoubleSupplier, yDoubleSupplier, zDoubleSupplier;
  private LinearFilter lowPassFilter;
  private DrivebaseModes mode;
  private double statorLimit;
  private double filterOutput;
  public enum DrivebaseModes{MANUAL, REVERSING}

  public DrivebaseSubsystem() {
    // construct motors
    frontLeft = new WPI_TalonSRX(Ports.FRONT_LEFT_MOTOR_PORT);
    frontRight = new WPI_TalonSRX(Ports.FRONT_RIGHT_MOTOR_PORT);
    backLeft = new WPI_TalonSRX(Ports.BACK_LEFT_MOTOR_PORT);
    backRight = new WPI_TalonSRX(Ports.BACK_RIGHT_MOTOR_PORT);
    frontRight.setInverted(true);
    backRight.setInverted(true);
    gyro = new AHRS();
    drivebase = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    lowPassFilter = LinearFilter.movingAverage(3);
  }

  public void drivePeriodic(double xSpeed, double ySpeed, double zRotation){
    drivebase.driveCartesian(xSpeed, ySpeed, zRotation, gyro.getRotation2d());
  }

  public void reversingPeriodic(){
    double xReverse = 0;
    double yReverse = 0;
    
    if (xDoubleSupplier.getAsDouble() < 0){
      xReverse = -0.2;
    }
    else if (xDoubleSupplier.getAsDouble() > 0){
      xReverse  = 0.2;
    }

    if (yDoubleSupplier.getAsDouble() < 0){
      yReverse = -0.2;
    }
    else if (yDoubleSupplier.getAsDouble() > 0){
      yReverse  = 0.2;
    }
    
    drivebase.driveCartesian(xReverse, yReverse, 0);
  }

  public AHRS getGyro(){
    return gyro;
  }

  public WPI_TalonSRX getFrontLeftMotor(){
    return frontLeft;
  }

  public double getFLStatorCurrent(){
    return frontLeft.getStatorCurrent();
  }

  public LinearFilter getFilter(){
    return lowPassFilter;
  }

  public double getFilterOutput(){
    return filterOutput;
  }

  public void setStatorLimit(double statorLimit){
    this.statorLimit = statorLimit;
  }

  public void setMode(DrivebaseModes mode){
    this.mode = mode;
  }

  public DrivebaseModes getMode(){
    return mode;
  }

  public void setInputs(DoubleSupplier xDoubleSupplier, DoubleSupplier yDoubleSupplier, DoubleSupplier zDoubleSupplier){
    this.xDoubleSupplier = xDoubleSupplier;
    this.yDoubleSupplier = yDoubleSupplier;
    this.zDoubleSupplier = zDoubleSupplier;
  }

  public DrivebaseModes advanceMode(){
    if (filterOutput >= 2/*statorLimit*/){
      return DrivebaseModes.REVERSING;
    }
    else{
      return DrivebaseModes.MANUAL;
    }
    /*
    switch(mode){
    case REVERSING:
        return DrivebaseModes.REVERSING;
    default:
        return DrivebaseModes.MANUAL;
    }
    */
  }

  private void applyMode(DrivebaseModes modes){
    switch(modes){
      case REVERSING:
        reversingPeriodic();
      default:
        drivePeriodic(xDoubleSupplier.getAsDouble(), yDoubleSupplier.getAsDouble(), zDoubleSupplier.getAsDouble());
    }
  }

  @Override
  public void periodic() {
    filterOutput = lowPassFilter.calculate(frontLeft.getStatorCurrent());
    mode = advanceMode();
    applyMode(mode);
  }

  @Override
  public void simulationPeriodic() {}
}
