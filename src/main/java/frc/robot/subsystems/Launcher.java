package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

    private final CANSparkFlex intakeBottom = new CANSparkFlex(10, MotorType.kBrushless);
    private final CANSparkFlex intakeTop = new CANSparkFlex(11, MotorType.kBrushless);

    public Launcher() {
        // Runs when calling new Launcher()
        configureMotors();
    }

    @Override
    public void periodic() {
        // Stuff to run repeatedly
    }

    private void configureMotors() {

    }
}