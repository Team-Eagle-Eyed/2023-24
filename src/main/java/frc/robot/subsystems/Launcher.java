package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

    private final CANSparkMax intakeBottom = new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax intakeTop = new CANSparkMax(11, MotorType.kBrushless);

    private final CANSparkFlex outtakeBottom = new CANSparkFlex(20, MotorType.kBrushless);
    private final CANSparkFlex outtakeTop = new CANSparkFlex(21, MotorType.kBrushless);

    public Launcher() {
        // Runs when calling new Launcher()
        configureMotors();
    }

    @Override
    public void periodic() {
        // Stuff to run repeatedly
        SmartDashboard.putNumber("Launcher velocity", outtakeBottom.getEncoder().getVelocity());
    }

    private void configureMotors() {
        intakeBottom.setSmartCurrentLimit(20);
        intakeTop.setSmartCurrentLimit(20);
        outtakeBottom.setSmartCurrentLimit(20);
        outtakeTop.setSmartCurrentLimit(20);

        intakeBottom.setOpenLoopRampRate(0.25);
        intakeTop.setOpenLoopRampRate(0.25);
        outtakeBottom.setOpenLoopRampRate(1);
        outtakeTop.setOpenLoopRampRate(1);

        intakeTop.follow(intakeBottom, true);

        outtakeTop.follow(outtakeBottom, true);
    }

    public void intake(double speed) {
        intakeBottom.set(speed);
        // intakeTop.set(-speed);
    }

    public void outtake(double speed) {
        outtakeBottom.set(speed);
        // outtakeTop.set(-speed);
    }

    public RelativeEncoder getOuttakeEncoder() {
        return intakeBottom.getEncoder();
    }

    public SparkPIDController getOuttakePID() {
        return outtakeBottom.getPIDController();
    }
}