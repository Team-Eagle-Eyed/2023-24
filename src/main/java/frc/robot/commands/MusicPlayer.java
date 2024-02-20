package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;


public class MusicPlayer extends Command {
    private Orchestra orchestra;
    private Swerve m_Swerve;

    public MusicPlayer(Swerve m_swerve) {
        addRequirements(m_swerve);
        this.m_Swerve = m_swerve;
    }
    
    @Override
    public void initialize() {
        orchestra = new Orchestra();
        orchestra.loadMusic("imperial_march.chrp");
        for (TalonFX motor : m_Swerve.getModuleDriveMotors()) {
            orchestra.addInstrument(motor, motor.getDeviceID() - 1);
        }
        for (TalonFX motor : m_Swerve.getModuleAngleMotors()) {
            orchestra.addInstrument(motor, motor.getDeviceID() - 1);
        }
    }

    @Override
    public void execute() {
        orchestra.play();
    }

    public void stopMusic() {
        orchestra.stop();
    }
}