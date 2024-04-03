package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;

public class Estop extends Command {

    Optional<PhotonTrackedTarget> target;

    public Estop(Optional<PhotonTrackedTarget> target) {
        addRequirements();
        this.target = target;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        target.get().getYaw();
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
    }

    @Override
    public boolean isFinished() {
        return false;
        // Whether or not the command is finished
    }
}