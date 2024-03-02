package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Photonvision;


public class LaunchNote extends Command {

    private Outtake launcher;
    private DoubleSupplier velocity;
    private Intake intake;
    private Photonvision photonvision;
    private Swerve swerve;
    private Arm arm;

    private double ANGULAR_P = 0.15;
    private double ANGULAR_I = 0.3;
    private double ANGULAR_D = 0;

    private double ARM_P = 0.03;
    private double ARM_I = 0.0005;
    private double ARM_D = 0;

    private PIDController turnController;
    private PIDController armPositionController;

    public LaunchNote(Swerve swerve, Arm arm, Outtake launcher, Intake intake, Photonvision photonvision, DoubleSupplier velocity) {
        addRequirements(swerve, arm, launcher, intake, photonvision);
        this.swerve = swerve;
        this.arm = arm;
        this.launcher = launcher;
        this.intake = intake;
        this.photonvision = photonvision;
        this.velocity = velocity;

        /*
         * Create PID controllers for the arm and rotation
         */
        armPositionController = new PIDController(ARM_P, ARM_I, ARM_D);
        turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);
    }
    
    @Override
    public void initialize() {
        // Runs once on start
    
        /*
         * Closed loop PID for launcher
         */
        launcher.getOuttakePID().setP(0.0002);
        launcher.getOuttakePID().setI(0);
        launcher.getOuttakePID().setD(0);
        launcher.getOuttakePID().setFF(0.000175);
        launcher.getOuttakePID().setOutputRange(0, 1);


        /*
         * Configure arm and rotation PID controllers
         */
        turnController.setTolerance(1);
        turnController.setSetpoint(2.15);

        armPositionController.setTolerance(1);
        armPositionController.setSetpoint(Units.radiansToDegrees(Math.atan(1.45 / photonvision.getTargetRange())));

    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        double targetVelocity = velocity.getAsDouble(); // Get velocity

        PhotonPipelineResult result = photonvision.getLatestResult(); // Get latest result from photonvision
        double rotationSpeed = 0; // Define rotationSpeed as a default of 0
        boolean validTarget = false; // Define validTarget as a default of false

        /*
         * Logic for choosing an apriltag to use
         */
        if(result.hasTargets()) { // if a target is acquired
            PhotonTrackedTarget target = result.getBestTarget(); // get best target
            List<PhotonTrackedTarget> targets = result.getTargets(); // get list of targets
            
            if(targets.get(0).getFiducialId() == 4 || targets.get(0).getFiducialId() == 7) { // if the first target is 4 or 7, 
                target = targets.get(0); // set the target to use as the first target
                validTarget = true; // and mark as having a valid target
            } else if (targets.size() > 1) { // otherwise if the list of targets is more than 1
                if(targets.get(1).getFiducialId() == 4 || targets.get(1).getFiducialId() == 7) { // and if the second target is 4 or 7 (assuming the camera will only see the two targets at the same time)
                    target = targets.get(1); // set the target to use as the second target
                    validTarget = true; // and mark as having a valid target
                }
            } else { // otherwise
                validTarget = false; // mark as having no valid target
            }

            /*
             * Calculating the rotation output
             */
            if (validTarget/*  && !turnController.atSetpoint() */) { // if we have a valid target and the PID controller for rotation isn't already within the tolerance
                rotationSpeed = turnController.calculate(target.getYaw()); // set the rotation speed based on the PID calculations
            } else { // otherwise
                turnController.calculate(target.getYaw());
                rotationSpeed = 0; // don't rotate
            }
        } else { // if no target acquired
            rotationSpeed = 0; // don't rotate
        }



        /*
         * Outputs
         */
        swerve.drive( // drive with the rotationSpeed from the if statements and PID controllers above
            new Translation2d(),
            rotationSpeed,
            false,
            true
            );

        arm.drive(MathUtil.clamp(armPositionController.calculate(arm.getAbsoluteAdjustedPosition()), -1, 1));

        launcher.getOuttakePID().setReference(targetVelocity, ControlType.kVelocity);


        /*
         * Checking conditions before launch
         */
        boolean launcherAtSpeed = launcher.getVelocity() > targetVelocity - 200;

        if(launcherAtSpeed && turnController.atSetpoint() && armPositionController.atSetpoint()) { // if the launcher is within the tolerance of the specified RPMs and our target is within the tolerance
            intake.intake(1); // run the intake to push it into the launcher
        } else { // otherwise
            intake.intake(0); // don't run the intake
        }


        /*
         * Put checks to dashboard
         */
        SmartDashboard.putBoolean("validTarget", validTarget);
        SmartDashboard.putBoolean("launcherAtSpeed", launcherAtSpeed);
        SmartDashboard.putBoolean("rotationAtSetpoint", turnController.atSetpoint());
        SmartDashboard.putBoolean("armAtSetpoint", armPositionController.atSetpoint());
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