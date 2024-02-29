package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Swerve;

public class CenterTarget extends Command {

    private Swerve m_Swerve;
    private Photonvision photonvision;
    private PhotonTrackedTarget firstTarget;

    private Double strafeSetpoint;

    private Double maxOutput = 0.75;

    private Boolean hasTarget;
    private PIDController rotationController;

    public CenterTarget(Swerve m_swerve, Photonvision photonvision) {
        addRequirements(m_swerve, photonvision);
        this.m_Swerve = m_swerve;
        this.photonvision = photonvision;
        rotationController = new PIDController(0.25, 0, 0.02);
    }
    
    @Override
    public void initialize() {
        rotationController.setTolerance(0.5);
        rotationController.setSetpoint(0);
    }

    @Override
    public void execute() {
        rotationController.setP(SmartDashboard.getNumber("ctp", 0));
        rotationController.setI(SmartDashboard.getNumber("cti", 0));
        rotationController.setD(SmartDashboard.getNumber("ctd", 0));

        hasTarget = !photonvision.getTargets().isEmpty();
        if(!hasTarget) return;
        firstTarget = photonvision.getTargets().get(0);

        m_Swerve.drive(
            new Translation2d(),
            MathUtil.clamp(rotationController.calculate(hasTarget ? firstTarget.getYaw() : strafeSetpoint), -maxOutput, maxOutput),
            false,
            true
            );
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if(rotationController.atSetpoint()) {
            return true;
        } else {
            return false;
        }
    }
}