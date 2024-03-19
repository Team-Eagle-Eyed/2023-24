package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteCamera;
import frc.robot.subsystems.Swerve;


public class GoToNote extends Command {
    private Swerve m_swerve;
    private NoteCamera m_NoteCamera;

    private PIDController turnController;

    public GoToNote(Swerve m_swerve, NoteCamera m_NoteCamera) {
        addRequirements(m_swerve);
        this.m_swerve = m_swerve;
        this.m_NoteCamera = m_NoteCamera;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        turnController = new PIDController(0.15, 0, 0); // I 0.3?
        turnController.setSetpoint(0);
        turnController.setTolerance(1);
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        double rotationOutput = turnController.calculate(m_NoteCamera.getLatestResult().getBestTarget().getYaw());
        m_swerve.drive(new Translation2d(), rotationOutput, false, true);
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        turnController.close();
        m_swerve.drive(new Translation2d(), 0, false, true);
    }

    @Override
    public boolean isFinished() {
        return false;
        // Whether or not the command is finished
    }
}