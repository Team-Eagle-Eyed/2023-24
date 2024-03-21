package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NoteCamera;
import frc.robot.subsystems.Swerve;


public class GoToNote extends Command {
    private Swerve m_swerve;
    private NoteCamera m_NoteCamera;
    private Intake m_intake;

    private PIDController strafeController;
    private PIDController driveController;

    private boolean noteCentered;

    public GoToNote(Swerve m_swerve, Intake m_intake, NoteCamera m_NoteCamera) {
        addRequirements(m_swerve, m_intake);
        this.m_swerve = m_swerve;
        this.m_intake = m_intake;
        this.m_NoteCamera = m_NoteCamera;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        strafeController = new PIDController(0.05, 0, 0);
        strafeController.setSetpoint(0);
        strafeController.setTolerance(3);

        driveController = new PIDController(0.2, 0, 0);
        driveController.setSetpoint(-10);
        driveController.setTolerance(3);

        noteCentered = false;
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        double translationOutput;
        double strafeOutput;
        boolean photonHasTarget = m_NoteCamera.getLatestResult().hasTargets();

        if(photonHasTarget) {
            translationOutput = -driveController.calculate(m_NoteCamera.getLatestResult().getBestTarget().getPitch());
        } else {
            translationOutput = 0;
        }
        if(photonHasTarget && !noteCentered) {
            strafeOutput = strafeController.calculate(m_NoteCamera.getLatestResult().getBestTarget().getYaw());
        } else {
            strafeOutput = 0;
        }
        if(strafeController.atSetpoint() && driveController.atSetpoint()) {
            noteCentered = true;
        }
        if(noteCentered == true) {
            translationOutput = 2;
        }
        m_swerve.drive(
            new Translation2d(translationOutput, strafeOutput),
            0,
            false,
            true
        );
        if(noteCentered == true) {
            m_intake.setIntakeVelocity(4000);
        } else {
            m_intake.intake(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        strafeController.close();
        driveController.close();
        m_swerve.drive(new Translation2d(), 0, false, true);
        m_intake.intake(0);
    }

    @Override
    public boolean isFinished() {
        return !m_intake.getNoteSensor().get();
        // Whether or not the command is finished
    }
}