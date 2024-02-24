package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;


public class SetLauncherVelocity extends Command {

    private Outtake launcher;
    private DoubleSupplier velocity;

    public SetLauncherVelocity(Outtake launcher, DoubleSupplier velocity) {
        addRequirements(launcher);
        this.launcher = launcher;
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
        // Bang bang
        /* if(launcher.getOuttakeEncoder().getVelocity() <= velocity.getAsDouble()) {
            launcher.outtake(1);
        } else if(launcher.getOuttakeEncoder().getVelocity() > velocity.getAsDouble()) {
            launcher.outtake(velocity.getAsDouble() / 7000);
        } */
        launcher.getOuttakePID().setReference(velocity.getAsDouble(), ControlType.kVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        launcher.getOuttakePID().setP(0);
        // launcher.getOuttakePID().setReference(0, ControlType.kVelocity);
    }

    @Override
    public boolean isFinished() {
        return false;
        // Whether or not the command is finished
    }
}