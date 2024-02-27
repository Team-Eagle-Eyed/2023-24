package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
    private final CANSparkFlex outtakeBottom = new CANSparkFlex(20, MotorType.kBrushless);
    private final CANSparkFlex outtakeTop = new CANSparkFlex(21, MotorType.kBrushless);

    public Outtake() {
        // Runs when calling new Launcher()
        configureMotors();
    }

    @Override
    public void periodic() {
        // Stuff to run repeatedly
        // SmartDashboard.putNumber("Launcher velocity", outtakeBottom.getEncoder().getVelocity());
    }

    private void configureMotors() {
        outtakeBottom.setSmartCurrentLimit(40);
        outtakeTop.setSmartCurrentLimit(40);

        outtakeBottom.setOpenLoopRampRate(0.25);
        outtakeTop.setOpenLoopRampRate(0.25);

        outtakeTop.setIdleMode(IdleMode.kCoast);
        outtakeBottom.setIdleMode(IdleMode.kCoast);

        outtakeTop.follow(outtakeBottom, true);
    }

    public void outtake(double speed) {
        outtakeBottom.set(speed);
        // outtakeTop.set(-speed);
    }

    public RelativeEncoder getOuttakeEncoder() {
        return outtakeBottom.getEncoder();
    }

    public SparkPIDController getOuttakePID() {
        return outtakeBottom.getPIDController();
    }
}