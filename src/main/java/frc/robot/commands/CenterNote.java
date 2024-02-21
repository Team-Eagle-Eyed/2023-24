package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Swerve;

public class CenterNote extends Command {

    private Swerve m_Swerve;
    private Photonvision photonvision;
    private PhotonTrackedTarget firstTarget;
    private Translation2d translation;

    private Double translationSetpoint;
    private Double strafeSetpoint;

    private Double maxOutput = 0.5;

    private Boolean hasTarget;

    private PIDController translationController;
    private PIDController strafeController;

    public CenterNote(Swerve m_swerve, Photonvision photonvision) {
        addRequirements(m_swerve, photonvision);
        this.m_Swerve = m_swerve;
        this.photonvision = photonvision;

        translationController = new PIDController(0.4, 0, 0);
        strafeController = new PIDController(0.4, 0, 0);
    }
    
    @Override
    public void initialize() {
        translationController.setSetpoint(3);
        strafeController.setSetpoint(0);

        translationController.setTolerance(4);
        strafeController.setTolerance(4);
    }

    @Override
    public void execute() {
        firstTarget = photonvision.getTargets().get(0);
        hasTarget = !photonvision.getTargets().isEmpty();

        translation = new Translation2d(
            MathUtil.clamp(translationController.calculate(hasTarget ? firstTarget.getArea() : translationSetpoint), -maxOutput, maxOutput),
            MathUtil.clamp(strafeController.calculate(hasTarget ? firstTarget.getYaw() : strafeSetpoint), -maxOutput, maxOutput)
        );

        m_Swerve.drive(translation, 0, false, true);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}