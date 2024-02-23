package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher;


public class TeleopIntake extends Command {

    private Launcher launcher;
    private DoubleSupplier intakeSpeed;
    private DoubleSupplier outtakeSpeed;

    public TeleopIntake(Launcher launcher, DoubleSupplier intakeSpeed, DoubleSupplier outtakeSpeed) {
        addRequirements(launcher);
        this.launcher = launcher;
        this.intakeSpeed = intakeSpeed;
        this.outtakeSpeed = outtakeSpeed;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        launcher.intake(MathUtil.applyDeadband(intakeSpeed.getAsDouble(), Constants.stickDeadband));
        launcher.outtake(MathUtil.applyDeadband(outtakeSpeed.getAsDouble(), Constants.stickDeadband));
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        launcher.intake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
        // Whether or not the command is finished
    }
}