package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

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

    private PIDController turnController;
    private PIDController driveController;

    private boolean noteCentered;
    private boolean targetAcquired;
    private double finishedCount;

    private PhotonTrackedTarget target;

    public GoToNote(Swerve m_swerve, Intake m_intake, NoteCamera m_NoteCamera) {
        addRequirements(m_swerve, m_intake);
        this.m_swerve = m_swerve;
        this.m_intake = m_intake;
        this.m_NoteCamera = m_NoteCamera;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        turnController = new PIDController(0.1, 0, 0);
        turnController.setSetpoint(0);
        turnController.setTolerance(0.5);

        driveController = new PIDController(0.2, 0, 0);
        driveController.setSetpoint(-20);
        driveController.setTolerance(10);
        noteCentered = false;
        targetAcquired = false;
        finishedCount = 0;
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        double translationOutput;
        double turnOutput;
        boolean photonHasTarget = m_NoteCamera.getLatestResult().hasTargets();

        if(photonHasTarget) {
            target = m_NoteCamera.getLatestResult().getBestTarget();
            targetAcquired = true;
        }

        if(targetAcquired) {
            translationOutput = -driveController.calculate(target.getPitch());
        } else {
            translationOutput = 0;
        }

        if(photonHasTarget) {
            turnController.setSetpoint(m_swerve.getHeading().getDegrees() - target.getYaw());
        }
        if(targetAcquired) {
            turnOutput = turnController.calculate(m_swerve.getHeading().getDegrees());
        } else {
            turnOutput = 0;
        }
        if(turnController.atSetpoint()/*  && driveController.atSetpoint() */) {
            noteCentered = true;
        }
        if(noteCentered == true && m_intake.getSecondaryNoteSensor().get()) {
            translationOutput = 2;
        } else if (!m_intake.getSecondaryNoteSensor().get()) {
            translationOutput = 0;
        }
        m_swerve.drive(
            new Translation2d(translationOutput, 0),
            turnOutput,
            false,
            true
        );

        boolean intakeSensorTripped = !m_intake.getNoteSensor().get();
        boolean outtakeSensorTripped = !m_intake.getSecondaryNoteSensor().get();
        if(noteCentered == true && !(!intakeSensorTripped && outtakeSensorTripped)) {
            m_intake.setIntakeVelocity(2700);
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
        turnController.close();
        driveController.close();
        m_swerve.drive(new Translation2d(), 0, false, true);
        m_intake.intake(0);
    }

    @Override
    public boolean isFinished() {
        // return (m_intake.getNoteSensor().get() && !m_intake.getSecondaryNoteSensor().get());
        return finishedCount > 1;
        // I set a flag that the first sensor was tripped then set it to reverse
        //did not work. Still finished with the note in the shooter.
        //
        //return !m_intake.getNoteSensor().get(); // Brian, I am trying using the other sensor to 
        // stop it from finishing with the note jammed into the shooter wheels... Did not work.
        // Whether or not the command is finished
    }
}