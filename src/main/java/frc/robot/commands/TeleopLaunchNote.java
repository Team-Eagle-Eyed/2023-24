package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Photonvision;


public class TeleopLaunchNote extends Command {

    private Outtake launcher;
    private DoubleSupplier velocity;
    private Intake intake;
    private Photonvision photonvision;
    private Swerve swerve;

    private final double ANGULAR_P = 0.1;
    private final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    public TeleopLaunchNote(Swerve swerve, Outtake launcher, Intake intake, Photonvision photonvision, DoubleSupplier velocity) {
        addRequirements(swerve, launcher, intake, photonvision);
        this.swerve = swerve;
        this.launcher = launcher;
        this.intake = intake;
        this.photonvision = photonvision;
        this.velocity = velocity;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        launcher.getOuttakePID().setP(0.0002);
        launcher.getOuttakePID().setI(0);
        launcher.getOuttakePID().setD(0);
        launcher.getOuttakePID().setFF(0.000175);
        launcher.getOuttakePID().setOutputRange(0, 1);

        turnController.setTolerance(1);
        turnController.setSetpoint(0);
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        double targetVelocity = velocity.getAsDouble();
        launcher.getOuttakePID().setReference(targetVelocity, ControlType.kVelocity);

        PhotonPipelineResult result = photonvision.getLatestResult();
        double rotationSpeed;
        if(result.hasTargets() && !turnController.atSetpoint()) { //if there is a target, and you aren't already within the tolerance continue
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
            //TODO: Find the correct ID number
        } else {
            rotationSpeed = 0;
        }

        swerve.drive(
            new Translation2d(),
            rotationSpeed,
            false,
            true
            );

        boolean launcherAtSpeed = launcher.getVelocity() > targetVelocity - 200 && launcher.getVelocity() < targetVelocity + 200;
        boolean targetCentered = turnController.atSetpoint();

        if(launcherAtSpeed && targetCentered) {
            intake.intake(1);
        } else {
            intake.intake(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        launcher.getOuttakePID().setP(0);
    }

    @Override
    public boolean isFinished() {
        return false;
        // Whether or not the command is finished
    }
}