package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private final CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(2, MotorType.kBrushless);

    private ApriltagCamera camera;

    public boolean hasOptimalAngle = false;
    public double optimalAngle = 0;

    public boolean atSetpoint = false;

    public Arm(ApriltagCamera camera) {
        // Runs when calling new Template()
        configureMotors();
        this.camera = camera;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Arm Motor", leftMotor.get());
        SmartDashboard.putNumber("Right Arm Motor", rightMotor.get());
        SmartDashboard.putNumber("Left Motor position", leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Right Motor position", rightMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Left Motor absolute position", getAbsoluteAdjustedPosition());

        SmartDashboard.putBoolean("armAtSetpoint", atSetpoint);


        if(camera.rangeToSpeaker.isPresent()) {
            optimalAngle = MathUtil.clamp(
                (2.9873 * (Math.pow(camera.rangeToSpeaker.get(), 2))) - (30.058 * camera.rangeToSpeaker.get()) + 100.453,
                23,
                90
                );
            hasOptimalAngle = true;
        } else {
            hasOptimalAngle = false;
        }
    }

    private void configureMotors() {
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.getAbsoluteEncoder().setPositionConversionFactor(360);
        leftMotor.getAbsoluteEncoder().setInverted(true);
        //leftMotor.getEncoder().setPosition(leftMotor.getAbsoluteEncoder().getPosition()); //TODO: Is this right? //0.143

        leftMotor.getPIDController().setFeedbackDevice(leftMotor.getAbsoluteEncoder());

        leftMotor.setSmartCurrentLimit(35);
        rightMotor.setSmartCurrentLimit(35);

        rightMotor.follow(leftMotor, true);

        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    public void drive(double speed) {
        if ((getAbsoluteAdjustedPosition() > 175 && speed > 0) || (getAbsoluteAdjustedPosition() < 27 && speed < 0)) {
            leftMotor.set(0);
        } else if (speed == 0) {
            leftMotor.set(Math.cos(Units.degreesToRadians(getAbsoluteEncoder().getPosition())) * 0.015);
        } else {
            leftMotor.set(-speed);
        }
        // rightMotor.set(speed);
    }

    public SparkPIDController getPIDController() {
        return leftMotor.getPIDController();
    }

    public AbsoluteEncoder getAbsoluteEncoder() {
        return leftMotor.getAbsoluteEncoder();
    }

    public RelativeEncoder getRelativeEncoder() {
        return leftMotor.getEncoder();
    }

    public Double getAbsoluteAdjustedPosition() {
        return leftMotor.getAbsoluteEncoder().getPosition() - 103; // -106
    }
}