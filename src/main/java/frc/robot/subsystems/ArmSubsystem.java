package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private PIDController m_PIDController;
    private TalonFX ArmMotor;
    private double ArmPosition;
    private double DesiredAngle;

    public ArmSubsystem(){
        m_PIDController = new PIDController(0.05, 0, 0);
    }

    public double getCurrentPos(){
        return ArmPosition;
    }

    public void setDesiredAngle(double desiredAngle){
        DesiredAngle = desiredAngle;
    }

    public void initalize(){}

    public void periodic(){
        ArmPosition = ArmMotor.getSelectedSensorPosition();
        ArmMotor.set(ControlMode.PercentOutput, m_PIDController.calculate(ArmPosition, DesiredAngle));
    }

    public void simulationPeriodic(){}
}

