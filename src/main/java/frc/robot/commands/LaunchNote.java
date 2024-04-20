package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;


public class LaunchNote extends Command {

    private Outtake launcher;
    private DoubleSupplier velocity;
    private Intake intake;

    boolean targetPresent;
    PhotonTrackedTarget target;

    private Timer finishedTimer = new Timer();

    public LaunchNote(Outtake launcher, Intake intake, DoubleSupplier velocity) {
        addRequirements(launcher, intake);
        this.launcher = launcher;
        this.intake = intake;
        this.velocity = velocity;
    }

    @Override
    public void initialize() {
        // Runs once on start

        targetPresent = false;
        finishedTimer.start();
    
        /*
         * Closed loop PID for launcher
         */
        launcher.getOuttakePID().setP(0.0002);
        launcher.getOuttakePID().setI(0);
        launcher.getOuttakePID().setD(0);
        launcher.getOuttakePID().setFF(0.000175);
        launcher.getOuttakePID().setOutputRange(0, 1);

    }

    @Override
    public void execute() {
        /*
         * Outputs
         */

        double targetVelocity = velocity.getAsDouble(); // Get velocity
        launcher.getOuttakePID().setReference(targetVelocity, ControlType.kVelocity);


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
        if(launcherAtSpeed) {
            intake.intake(1); // run the intake to push it into the launcher
        } else { // otherwise
            intake.intake(0); // don't run the intake
            finishedTimer.restart();
        }


        /*
         * Put checks to dashboard
         */
        SmartDashboard.putBoolean("launcherAtSpeed", launcherAtSpeed);
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
        if(finishedTimer.get() > 0.25) {
            return true;
        } else {
            return false;
        }
        // Whether or not the command is finished
    }
}