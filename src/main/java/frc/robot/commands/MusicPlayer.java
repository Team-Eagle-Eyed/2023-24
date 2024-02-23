package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;


public class MusicPlayer extends Command {
    private Orchestra orchestra;
    private SendableChooser<String> track;

    public MusicPlayer(Swerve m_swerve, SendableChooser<String> track) {
        addRequirements(m_swerve);
        this.orchestra = m_swerve.getOrchestra();
        this.track = track;
        SmartDashboard.putBoolean("isPlaying", false);
    }
    
    @Override
    public void initialize() {
        orchestra.loadMusic(track.getSelected());
    }

    @Override
    public void execute() {
        orchestra.play();
        SmartDashboard.putBoolean("isPlaying", orchestra.isPlaying());
    }

    @Override
    public void end(boolean interrupted) {
        orchestra.stop();
        SmartDashboard.putBoolean("isPlaying", orchestra.isPlaying());
    }

    @Override
    public boolean isFinished() {
        return !orchestra.isPlaying();
    }
}