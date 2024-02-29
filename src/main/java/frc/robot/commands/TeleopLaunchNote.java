package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;


public class TeleopLaunchNote extends Command {

    private Outtake launcher;
    private DoubleSupplier velocity;
    private Intake intake;

    public TeleopLaunchNote(Outtake launcher, Intake intake, DoubleSupplier velocity) {
        addRequirements(launcher, intake);
        this.launcher = launcher;
        this.intake = intake;
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
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        double targetVelocity = velocity.getAsDouble();
        launcher.getOuttakePID().setReference(targetVelocity, ControlType.kVelocity);

        if(launcher.getVelocity() > targetVelocity - 200 && launcher.getVelocity() < targetVelocity + 200) {
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