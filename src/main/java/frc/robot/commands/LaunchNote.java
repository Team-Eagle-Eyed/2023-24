package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.ApriltagCamera;
import frc.robot.subsystems.Swerve;


public class LaunchNote extends Command {

    private Outtake launcher;
    private DoubleSupplier velocity;
    private Intake intake;
    private ApriltagCamera photonvision;
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

    private double rotationSpeed = 0; // Define rotationSpeed as a default of 0
    boolean validTarget = false; // Define validTarget as a default of false
    PhotonTrackedTarget target;

    private Timer finishedTimer = new Timer();

    public LaunchNote(Swerve swerve, Arm arm, Outtake launcher, Intake intake, ApriltagCamera photonvision, DoubleSupplier velocity) {
        addRequirements(swerve, arm, launcher, intake);
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
        finishedTimer.start();
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
        armPositionController.setTolerance(1);
        
        
        /*
        * Logic for choosing an apriltag to use
        */
        PhotonPipelineResult result = photonvision.getLatestResult(); // Get latest result from photonvision
        if(result.hasTargets()) { // if a target is acquired
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

        } else { // if no target acquired
            validTarget = false;
            turnController.setSetpoint(0); // Turncontroller isn't used if validTarget is false anyway. I set this in case turnController.calculate() would throw an error without it.
        }

        if(validTarget) {
            turnController.setSetpoint(swerve.getGyroYaw().getDegrees() - target.getYaw() + 2.15);
        }

    }

    @Override
    public void execute() {
        PhotonPipelineResult result = photonvision.getLatestResult(); // Get latest result from photonvision
        if(result.hasTargets()) { // if a target is acquired
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
                // validTarget = false; // mark as having no valid target
            }
        }
        /*
         * Configure the turning PID based on the target and gyroscope, plus an offset.
         */
        if(validTarget) {
            // double targetRange = photonvision.getSpecificTargetRange(target);
            double targetRange = PhotonUtils.getDistanceToPose(swerve.getPose(), photonvision.getSpeakerPose());
            /* double targetHeight = SmartDashboard.getNumber("targetHeight", 84);
            double armSetpoint = MathUtil.clamp(
                            Units.radiansToDegrees(Math.atan(Units.inchesToMeters(targetHeight) / targetRange)), // 85 inches
                            23,
                            90); */
            double armSetpoint = MathUtil.clamp(
                (0.0038 * (Math.pow(targetRange, 4))) - (0.1996 * Math.pow(targetRange, 3)) + (3.9121 * Math.pow(targetRange, 2)) - (35.256 * targetRange) + 159.93,
                23,
                90
                );
            SmartDashboard.putNumber("armLaunchSetpoint", armSetpoint);
            armPositionController.setSetpoint(armSetpoint);
        } else {
            turnController.setSetpoint(0); // Turncontroller isn't used if validTarget is false anyway. I set this in case turnController.calculate() would throw an error without it.

        }

        /*
         * Calculating the rotation output
         */
        if (validTarget/*  && !turnController.atSetpoint() */) { // if we have a valid target and the PID controller for rotation isn't already within the tolerance
            rotationSpeed = turnController.calculate(swerve.getGyroYaw().getDegrees()); // set the rotation speed based on the PID calculations
        } else { // otherwise
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

        if(validTarget) {
            arm.drive( // move the arm to it
                    MathUtil.clamp(armPositionController.calculate(arm.getAbsoluteAdjustedPosition()), -1, 1));

            /* if(!armPositionController.atSetpoint()) { // if we arent at the arm setpoint
                arm.drive( // move the arm to it
                    MathUtil.clamp(armPositionController.calculate(arm.getAbsoluteAdjustedPosition()), -1, 1));
            } else { // otherwise feedforward to stop the arm from drooping
                arm.drive(Math.cos(Units.degreesToRadians(arm.getAbsoluteEncoder().getPosition() - 180)) * 0.015);
                armPositionController.calculate(arm.getAbsoluteAdjustedPosition());
            } */
        }

        double targetVelocity = velocity.getAsDouble(); // Get velocity
        if(validTarget) {
            launcher.getOuttakePID().setReference(targetVelocity, ControlType.kVelocity);
        }


        /*
         * Checking conditions before launch
         */
        boolean launcherAtSpeed = launcher.getVelocity() > targetVelocity - 200;

        /*
         * Checking if we can launch
         * 1. Launcher is within the speed tolerance (greater than 200 less than the setpoint)
         * 2. The turncontroller is within the tolerance for facing the target, OR we didn't have a valid target to turn to
         * 3. The arm is at the correct angle
         */
        if(launcherAtSpeed && turnController.atSetpoint() && armPositionController.atSetpoint() && validTarget == true) {
            intake.intake(1); // run the intake to push it into the launcher
        } else { // otherwise
            intake.intake(0); // don't run the intake
            finishedTimer.restart();
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
        launcher.outtake(0);
        intake.intake(0);
        finishedTimer.stop();
    }

    @Override
    public boolean isFinished() {
        if(finishedTimer.get() > 1) {
            return true;
        } else {
            return false;
        }
        // Whether or not the command is finished
    }
}