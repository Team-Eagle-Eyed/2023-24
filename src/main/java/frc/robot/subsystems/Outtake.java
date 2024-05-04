package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase {
    private final CANSparkFlex outtakeBottom = new CANSparkFlex(20, MotorType.kBrushless);
    private final CANSparkFlex outtakeTop = new CANSparkFlex(21, MotorType.kBrushless);
    private final RelativeEncoder outtakeEncoder = outtakeBottom.getEncoder();

    private double velocitySetpoint;

    public boolean atSpeed;

    public Outtake() {
        // Runs when calling new Launcher()
        configureMotors();
        velocitySetpoint = 0;
    }

    @Override
    public void periodic() {
        // Stuff to run repeatedly
        SmartDashboard.putNumber("Launcher velocity", getVelocity());
        if(outtakeBottom.getEncoder().getVelocity() > velocitySetpoint - 200) {
            atSpeed = true;
        } else {
            atSpeed = false;
        }
    }

    private void configureMotors() {
        outtakeBottom.setSmartCurrentLimit(80);
        outtakeTop.setSmartCurrentLimit(80);

        outtakeBottom.setOpenLoopRampRate(0.25);
        outtakeTop.setOpenLoopRampRate(0.25);

        outtakeTop.setIdleMode(IdleMode.kCoast);
        outtakeBottom.setIdleMode(IdleMode.kCoast);

        outtakeTop.follow(outtakeBottom, true);

        /*
         * Closed loop PID for launcher
         */
        getOuttakePID().setP(0.0002);
        getOuttakePID().setI(0);
        getOuttakePID().setD(0);
        getOuttakePID().setFF(0.000175);
        getOuttakePID().setOutputRange(0, 1);
    }

    public void outtake(double speed) {
        outtakeBottom.set(speed);
        // outtakeTop.set(-speed);
    }

    public void setOuttakeVelocity(double velocity) {
        outtakeBottom.getPIDController().setReference(velocity, ControlType.kVelocity);
        velocitySetpoint = velocity;
    }

    public RelativeEncoder getOuttakeEncoder() {
        return outtakeEncoder;
    }

    public double getVelocity() {
        return outtakeEncoder.getVelocity();
    }

    public SparkPIDController getOuttakePID() {
        return outtakeBottom.getPIDController();
    }
}