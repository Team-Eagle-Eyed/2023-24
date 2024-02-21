package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private final CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(2, MotorType.kBrushless);

    public Arm() {
        // Runs when calling new Template()
        configureMotors();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Arm Motor", leftMotor.get());
        SmartDashboard.putNumber("Right Arm Motor", rightMotor.get());
        SmartDashboard.putNumber("Left Motor position", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Motor absolute position", leftMotor.getAbsoluteEncoder().getPosition());
    }

    private void configureMotors() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        rightMotor.follow(leftMotor, true);

        leftMotor.getEncoder().setPosition(leftMotor.getAbsoluteEncoder().getPosition() - 0.686); //TODO: Is this right?

        leftMotor.setSoftLimit(SoftLimitDirection.kForward, 10); //TODO: Tune this
        leftMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

        leftMotor.setSmartCurrentLimit(7);
        rightMotor.setSmartCurrentLimit(7);

        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    public void drive(double speed) {
        leftMotor.set(speed);
    }
}