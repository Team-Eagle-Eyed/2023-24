package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;


public class TeleopIntake extends Command {

    private Intake intake;
    private DoubleSupplier intakeSpeed;
    private boolean useRPMs;

    public TeleopIntake(Intake intake, DoubleSupplier intakeSpeed, boolean useRPMs) {
        addRequirements(intake);
        this.intake = intake;
        this.intakeSpeed = intakeSpeed;
        this.useRPMs = useRPMs;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        intake.getIntakePID().setP(0.0002);
        intake.getIntakePID().setI(0);
        intake.getIntakePID().setD(0);
        intake.getIntakePID().setFF(0.000175);
        intake.getIntakePID().setOutputRange(0, 1);
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        double output = MathUtil.applyDeadband(intakeSpeed.getAsDouble(), Constants.stickDeadband);

        // intake.intake(!noteSensor.get() && output > 0 ? 0 : output);
        if(useRPMs) {
            intake.setIntakeVelocity(output);
        } else {
            intake.intake(output);
        }
        // intake.intake(output);
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        intake.intake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
        // Whether or not the command is finished
    }
}