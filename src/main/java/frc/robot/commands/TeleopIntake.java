package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;


public class TeleopIntake extends Command {

    private Intake intake;
    private DoubleSupplier intakeSpeed;
    private DigitalInput noteSensor;
    private Boolean objectPresent;

    public TeleopIntake(Intake intake, DoubleSupplier intakeSpeed) {
        addRequirements(intake);
        this.intake = intake;
        this.intakeSpeed = intakeSpeed;
        this.noteSensor = intake.getNoteSensor();
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        objectPresent = false;
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        // intake.intake(MathUtil.applyDeadband(intakeSpeed.getAsDouble(), Constants.stickDeadband));
        double output = MathUtil.applyDeadband(intakeSpeed.getAsDouble(), Constants.stickDeadband);
        if(noteSensor.get() || objectPresent) {
            intake.intake(0);
            objectPresent = true;
        } else {
            intake.intake(output);
        }
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