package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class AngleDriveCommand extends CommandBase{

    DrivebaseSubsystem subsystem;
    DoubleSupplier leftX, leftY, rightX, rightY;
    double angle;
    
    public AngleDriveCommand(DrivebaseSubsystem subsystem, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier rightY) {
        
        this.subsystem = subsystem;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

        angle = subsystem.getGyroRotation().getDegrees();

    }

    @Override
    public void execute() {

        if( > 0.7) {

        }

    }

    @Override 
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
