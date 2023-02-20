// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.DrivebaseCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.SequentialCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final CommandXboxController controller_1 = new CommandXboxController(0);
  private final DrivebaseSubsystem m_drivebaseSubsystem = new DrivebaseSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_drivebaseSubsystem.setDefaultCommand(new DrivebaseCommand(
      m_drivebaseSubsystem, controller_1::getLeftY, controller_1::getLeftX, controller_1::getRightX));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * The ToggleMotorInvert command may have been fixed
   */

  private void configureButtonBindings() {
    controller_1.x().onTrue(new FollowTrajectoryCommand(PathPlanner.loadPath("New Path", 1, 1), m_drivebaseSubsystem));
    controller_1.a().onTrue(new ArmCommand(m_armSubsystem, 90, 0.5));
    controller_1.rightBumper().onTrue((new SequentialCommand(m_drivebaseSubsystem, m_armSubsystem)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}