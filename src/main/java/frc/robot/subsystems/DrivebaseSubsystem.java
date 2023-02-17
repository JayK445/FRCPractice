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
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.AdvancedSwerveTrajectoryFollower;

public class DrivebaseSubsystem extends SubsystemBase {
  private MecanumDrive drivebase;  
  private WPI_PigeonIMU gyro;
  private WPI_TalonSRX frontLeft, frontRight, backLeft, backRight;
  private MecanumDriveOdometry mecanumDriveOdometry;
  private MecanumDriveKinematics kinematics;
  private Pose2d pose2d = new Pose2d();
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
    gyro = new WPI_PigeonIMU(frontLeft);

    drivebase = new MecanumDrive(frontLeft, backRight, frontRight, backLeft);
    kinematics = new MecanumDriveKinematics(new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d());
    mecanumDriveOdometry = new MecanumDriveOdometry(kinematics, gyro.getRotation2d(), 
    new MecanumDriveWheelPositions(frontLeft.get(), frontRight.get(), backLeft.get(), backRight.get()));
  }

  public void drive (double ySpeed, double xSpeed, double zRotation){
    drivebase.driveCartesian(xSpeed, ySpeed, zRotation , gyro.getRotation2d());
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
