package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final CANSparkMax intakeBottom = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax intakeTop = new CANSparkMax(11, MotorType.kBrushless);
    private DigitalInput noteSensor;
    private DigitalInput secondaryNoteSensor;

    public Intake(DigitalInput noteSensor, DigitalInput secondaryNoteSensor) {
        // Runs when calling new Launcher()
        this.noteSensor = noteSensor;
        this.secondaryNoteSensor = secondaryNoteSensor;
        configureMotors();
    }

    @Override
    public void periodic() {
        // Stuff to run repeatedly
    }

    private void configureMotors() {
        intakeBottom.setSmartCurrentLimit(80);
        intakeTop.setSmartCurrentLimit(80);

        intakeBottom.setOpenLoopRampRate(0.25);
        intakeTop.setOpenLoopRampRate(0.25);

        intakeBottom.setIdleMode(IdleMode.kBrake);
        intakeTop.setIdleMode(IdleMode.kBrake);
        
        intakeTop.follow(intakeBottom, true);
    }

    public void intake(double speed) {
        intakeBottom.set(speed);
    }

    public void setIntakeVelocity(double velocity) {
        if (noteSensor.get()) {
            if(!secondaryNoteSensor.get()) {
                getIntakePID().setReference(velocity * 0.05, ControlType.kVelocity);    
            } else {
                getIntakePID().setReference(velocity, ControlType.kVelocity);
            }
        } else if (!noteSensor.get()){
            intakeBottom.set(-0.25);
        }
    }

    public RelativeEncoder getIntakeEncoder() {
        return intakeBottom.getEncoder();
    }

    public SparkPIDController getIntakePID() {
        return intakeBottom.getPIDController();
    }

    public DigitalInput getNoteSensor() {
        return noteSensor;
    }

    public DigitalInput getSecondaryNoteSensor() {
        return secondaryNoteSensor;
    }
}