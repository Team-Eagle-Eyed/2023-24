package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;


public class SetLauncherVelocity extends Command {

    private Launcher launcher;
    private DoubleSupplier velocity;

    public SetLauncherVelocity(Launcher launcher, DoubleSupplier velocity) {
        addRequirements(launcher);
        this.launcher = launcher;
        this.velocity = velocity;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        // Bang bang
        if(launcher.getOuttakeEncoder().getVelocity() <= velocity.getAsDouble()) {
            launcher.outtake(1);
        } else if(launcher.getOuttakeEncoder().getVelocity() > velocity.getAsDouble()) {
            launcher.outtake(0);
        }
        // launcher.getOuttakePID().setReference(velocity.getAsDouble(), ControlType.kVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        launcher.outtake(0);
        launcher.getOuttakePID().setReference(0, ControlType.kVelocity);
    }

    @Override
    public boolean isFinished() {
        return false;
        // Whether or not the command is finished
    }
}