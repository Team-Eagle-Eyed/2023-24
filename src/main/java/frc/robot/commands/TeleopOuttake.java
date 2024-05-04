package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;


public class TeleopOuttake extends Command {

    private Outtake outtake;
    private Intake intake;
    private DoubleSupplier outtakeSpeed;
    private Boolean useRPMs;

    public TeleopOuttake(Outtake outtake, Intake intake, DoubleSupplier outtakeSpeed, Boolean useRPMs) {
        addRequirements(outtake);
        this.outtake = outtake;
        this.intake = intake;
        this.outtakeSpeed = outtakeSpeed;
        this.useRPMs = useRPMs;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        if(useRPMs) {
            if(!intake.getSecondaryNoteSensor().get()) {
                outtake.setOuttakeVelocity(outtakeSpeed.getAsDouble());
            } else {
                outtake.outtake(0);
            }
        } else {
            outtake.outtake(MathUtil.applyDeadband(outtakeSpeed.getAsDouble(), Constants.stickDeadband));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        outtake.outtake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
        // Whether or not the command is finished
    }
}