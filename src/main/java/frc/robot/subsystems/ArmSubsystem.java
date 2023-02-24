package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private PIDController m_PIDController;
    private TalonFX armMotor;
    private double desiredAngle;
    private ShuffleboardTab armShuffleboard = Shuffleboard.getTab("Arm");
    private Modes mode;
    public enum Modes{ON, HOLD_POSITION}

    public ArmSubsystem(){
        armMotor = new TalonFX(13);
        m_PIDController = new PIDController(0.0004, 0, 0);
        armShuffleboard.add("PID", m_PIDController);
        armShuffleboard.addNumber("Arm Angle", armMotor::getSelectedSensorPosition);
        armShuffleboard.add("Target Angle", desiredAngle);
    }

    public double getCurrentPos(){
        return armMotor.getSelectedSensorPosition();
    }

    public void setDesiredAngle(double desiredAngle){
        this.desiredAngle = desiredAngle;
    }

    public void setMode(Modes mode){
        this.mode = mode;
    }

    public void PIDPeriodic(){
        armMotor.set(TalonFXControlMode.PercentOutput, 
        MathUtil.clamp(m_PIDController.calculate(armMotor.getSelectedSensorPosition(), desiredAngle), -0.1, 0.1));
    }
    
    public void holdPosition(){
        armMotor.setNeutralMode(NeutralMode.Brake);
    }

    public Modes advanceMode(){
        switch(mode){
            case ON:
                return Modes.ON;
            case HOLD_POSITION:
                return Modes.HOLD_POSITION;
        }
        return null;
    }
    public void applyMode(Modes mode){
        switch(mode){
            case ON:
                PIDPeriodic();
                break;
            case HOLD_POSITION:
                holdPosition();
                break;
        }
    }

    public void initialize(){}

    public void periodic(){
        mode = advanceMode();
        applyMode(mode);
    }

    public void simulationPeriodic(){}
}