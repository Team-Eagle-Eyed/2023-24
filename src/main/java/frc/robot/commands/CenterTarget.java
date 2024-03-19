package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ApriltagCamera;
import frc.robot.subsystems.Swerve;

public class CenterTarget extends Command {

    private Swerve m_Swerve;
    private ApriltagCamera photonvision;

    // PID constants should be tuned per robot
    private final double ANGULAR_P = 0.1;
    private final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    public CenterTarget(Swerve m_swerve, ApriltagCamera photonvision) {
        addRequirements(m_swerve, photonvision);
        this.m_Swerve = m_swerve;
        this.photonvision = photonvision;
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double rotationSpeed;
        PhotonPipelineResult result = photonvision.getLatestResult();

        if(photonvision.getLatestResult().hasTargets()) {

            // Also calculate angular power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        } else {
            rotationSpeed = 0;
        }

        m_Swerve.drive(
            new Translation2d(0, 0), // new Translation2d(0, forwardSpeed),
            rotationSpeed,
            false,
            true);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}