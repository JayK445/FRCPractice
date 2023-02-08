package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private PIDController m_PIDController;
    private TalonFX ArmMotor;
    private double DesiredAngle;
    private ShuffleboardTab ArmShuffleboard = Shuffleboard.getTab("Arm");

    public ArmSubsystem(){
        ArmMotor = new TalonFX(13);
        m_PIDController = new PIDController(0.0001, 0, 0.05);
        ArmShuffleboard.add("PID", m_PIDController);
        ArmShuffleboard.addNumber("Arm Angle", ArmMotor::getSelectedSensorPosition);
        ArmShuffleboard.add("Target Angle", DesiredAngle);
    }

    public double getCurrentPos(){
        return ArmMotor.getSelectedSensorPosition();
    }

    public void setDesiredAngle(double desiredAngle){
        DesiredAngle = desiredAngle;
    }

    public void initialize(){}

    public void periodic(){
        ArmMotor.set(TalonFXControlMode.PercentOutput, 
        MathUtil.clamp(m_PIDController.calculate(ArmMotor.getSelectedSensorPosition(), DesiredAngle), -0.1, 0.1));
    }

    public void simulationPeriodic(){}
}