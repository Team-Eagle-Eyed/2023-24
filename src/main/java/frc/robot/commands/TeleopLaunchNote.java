package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
        double rotationSpeed = 0;
        boolean validTarget = false;

        if(result.hasTargets()) { // if a target is acquired
            PhotonTrackedTarget target = result.getBestTarget(); // get best target
            List<PhotonTrackedTarget> targets = result.getTargets(); // get list of targets
            
            if(targets.get(0).getFiducialId() == 4 || targets.get(0).getFiducialId() == 7) { // if the first target is 4 or 7, 
                target = targets.get(0); // set the target to use as the first target
                validTarget = true; // and mark as having a valid target
            } else if (targets.get(1).getFiducialId() == 4 || targets.get(1).getFiducialId() == 7) { // otherwise if the second target is 4 or 7 (assuming the camera will only see the two targets at the same time)
                target = targets.get(1); // set the target to use as the second target
                validTarget = true; // and mark as having a valid target
            } else { // otherwise
                validTarget = false; // mark as having no valid target
            }

            if (validTarget && !turnController.atSetpoint()) { // if we have a valid target and the PID controller for rotation isn't already within the tolerance
                rotationSpeed = -turnController.calculate(target.getYaw(), 0); // set the rotation speed based on the PID calculations
            } else { // otherwise
                rotationSpeed = 0; // don't rotate
            }
        } else { // if no target acquired
            rotationSpeed = 0; // don't rotate
        }

        swerve.drive( // drive with the rotationSpeed from the if statements and PID controllers above
            new Translation2d(),
            rotationSpeed,
            false,
            true
            );

        boolean launcherAtSpeed = launcher.getVelocity() > targetVelocity - 200 && launcher.getVelocity() < targetVelocity + 200;
        boolean targetCentered = turnController.atSetpoint();

        if(launcherAtSpeed && targetCentered) { // if the launcher is within the tolerance of the specified RPMs and our target is within the tolerance
            intake.intake(1); // run the intake to push it into the launcher
        } else { // otherwise
            intake.intake(0); // don't run the intake
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        launcher.getOuttakePID().setP(0); // set p to 0 as to not brake the motor
    }

    @Override
    public boolean isFinished() {
        return false;
        // Whether or not the command is finished
    }
}