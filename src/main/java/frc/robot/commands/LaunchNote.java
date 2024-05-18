package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Swerve;


public class LaunchNote extends Command {

    private Swerve swerve;
    private Intake intake;
    private Outtake outtake;
    private Arm arm;

    boolean targetPresent;

    boolean targetCentered;
    boolean launcherAtSpeed;
    boolean armAtSetpoint;

    private Timer finishedTimer = new Timer();

    public LaunchNote(Swerve swerve, Intake intake, Outtake outtake, Arm arm) {
        addRequirements(intake);
        this.swerve = swerve;
        this.intake = intake;
        this.outtake = outtake;
        this.arm = arm;
    }

    @Override
    public void initialize() {
        // Runs once on start

        targetPresent = false;
        finishedTimer.start();

        targetCentered = false;
        launcherAtSpeed = false;
        armAtSetpoint = false;


    }

    @Override
    public void execute() {

        /*
         * Checking if we can launch
         * 1. Launcher is within the speed tolerance (greater than 200 less than the setpoint)
         * 2. The turncontroller is within the tolerance for facing the target, OR we didn't have a valid target to turn to
         * 3. The arm is at the correct angle
         */
        /* if(launcherAtSpeed) {
            intake.intake(1); // run the intake to push it into the launcher
        } else { // otherwise
            intake.intake(0); // don't run the intake
            finishedTimer.restart();
        } */
        
        if(swerve.hasOptimalAngle) {
            // Target is centered when the robot angle is within +- 0.5 degrees of the optimal angle;
            targetCentered = swerve.getHeading().getDegrees() < swerve.optimalAngle + 1 && swerve.getHeading().getDegrees() > swerve.optimalAngle - 1;
        } else {
            targetCentered = false;
        }

        if(arm.hasOptimalAngle) {
            // Arm is at the setpoint when it is within 1 degree of the optimal angle;
            armAtSetpoint = arm.getAbsoluteAdjustedPosition() < arm.optimalAngle + 1 && arm.getAbsoluteAdjustedPosition() > arm.optimalAngle - 1;
        } else {
            armAtSetpoint = false;
        }
        
        launcherAtSpeed = outtake.getOuttakeEncoder().getVelocity() > SmartDashboard.getNumber("Launcher set velocity", 4500) - 200;

        if(targetCentered && launcherAtSpeed && armAtSetpoint) {
            intake.intake(1);
        } else {
            intake.intake(0);
            finishedTimer.restart();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        intake.intake(0);
        finishedTimer.stop();
    }

    @Override
    public boolean isFinished() {
        /* if(finishedTimer.get() > 0.25) {
            return true;
        } else {
            return false;
        } */
        return false;
        // Whether or not the command is finished
    }
}