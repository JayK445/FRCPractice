// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.counter.Tachometer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.AdvancedSwerveTrajectoryFollower;

public class DrivebaseSubsystem extends SubsystemBase {
  private MecanumDrive drivebase;  
  private WPI_PigeonIMU gyro;
  private WPI_TalonSRX frontLeft, frontRight, backLeft, backRight;
  private Encoder encoderfl, encoderfr, encoderbl, encoderbr;
  private MecanumDriveOdometry mecanumDriveOdometry;
  private MecanumDriveKinematics kinematics;
  private Tachometer tachometer;
  private DigitalInput digitalInput;
  private final AdvancedSwerveTrajectoryFollower follower =
      new AdvancedSwerveTrajectoryFollower(
          new PIDController(0.4, 0.0, 0.025),
          new PIDController(0.4, 0.0, 0.025),
          new ProfiledPIDController(
              .147,
              0,
              0,
              new TrapezoidProfile.Constraints(1, 1)));

  public DrivebaseSubsystem() {
    // construct motors
    frontLeft = new WPI_TalonSRX(3);
    frontRight = new WPI_TalonSRX(1);
    backLeft = new WPI_TalonSRX(6);
    backRight = new WPI_TalonSRX(7);

    /*
    These encoders will not work, as the channels are not plugged in
    The Tachometer may be of use, but has no guarantee that it is even working, and has no function that gets the distance that the motor 
    has traveled
    */
    encoderfl = new Encoder(0, 1);
    encoderbl = new Encoder(2, 3);
    encoderfr = new Encoder(4, 5);
    encoderbr = new Encoder(6, 7);
    digitalInput = new DigitalInput(0);
    tachometer = new Tachometer(digitalInput);
    gyro = new WPI_PigeonIMU(frontLeft);
    drivebase = new MecanumDrive(frontLeft, backRight, frontRight, backLeft);
    kinematics = new MecanumDriveKinematics(new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d());
    mecanumDriveOdometry = new MecanumDriveOdometry(kinematics, gyro.getRotation2d(), /*The robot's encoders and kinematics must be adjusted*/
    new MecanumDriveWheelPositions(encoderfl.getDistance(), encoderfr.getDistance(), encoderbl.getDistance(), encoderbr.getDistance()));
  }

  public void drive (double ySpeed, double xSpeed, double zRotation){
    drivebase.driveCartesian(xSpeed, ySpeed, zRotation, gyro.getRotation2d());
  }

  public WPI_PigeonIMU getGyro(){
    return gyro;
  }

  public AdvancedSwerveTrajectoryFollower getFollower(){
    return follower;
  }
  public void resetOdometryToPose(Pose2d pose){
    gyro.reset();
    mecanumDriveOdometry.resetPosition(gyro.getRotation2d(), new MecanumDriveWheelPositions(), pose);

  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
