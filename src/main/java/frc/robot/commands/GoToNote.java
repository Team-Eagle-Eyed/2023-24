package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

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
    private double finishedCount;

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
        strafeController.setTolerance(2);

        driveController = new PIDController(0.2, 0, 0);
        driveController.setSetpoint(-15);
        driveController.setTolerance(10);
        noteCentered = false;
        finishedCount = 0;
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
        if(strafeController.atSetpoint()/*  && driveController.atSetpoint() */) {
            noteCentered = true;
        }
        if(noteCentered == true && m_intake.getSecondaryNoteSensor().get()) {
            translationOutput = 2;
        } else if (!m_intake.getSecondaryNoteSensor().get()) {
            translationOutput = 0;
        }
        m_swerve.drive(
            new Translation2d(translationOutput, strafeOutput),
            0,
            false,
            true
        );

        boolean intakeSensorTripped = !m_intake.getNoteSensor().get();
        boolean outtakeSensorTripped = !m_intake.getSecondaryNoteSensor().get();
        if(noteCentered == true && !(!intakeSensorTripped && outtakeSensorTripped)) {
            m_intake.setIntakeVelocity(4000);
        } else if (!intakeSensorTripped && outtakeSensorTripped) {
            finishedCount = finishedCount + 1;
            m_intake.setIntakeVelocity(0);
        } else {
            m_intake.setIntakeVelocity(0);
        }

        /* if(m_intake.getNoteSensor().get() && !m_intake.getSecondaryNoteSensor().get()) {
            finishedCount = finishedCount + 1;
            m_intake.setIntakeVelocity(0);
        } */

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
        // return (m_intake.getNoteSensor().get() && !m_intake.getSecondaryNoteSensor().get());
        return finishedCount > 10;
        // I set a flag that the first sensor was tripped then set it to reverse
        //did not work. Still finished with the note in the shooter.
        //
        //return !m_intake.getNoteSensor().get(); // Brian, I am trying using the other sensor to 
        // stop it from finishing with the note jammed into the shooter wheels... Did not work.
        // Whether or not the command is finished
    }
}