package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;


public class TeleopOuttake extends Command {

    private Outtake outtake;
    private Intake intake;
    private Arm arm;
    private DoubleSupplier outtakeSpeed;
    private Boolean useRPMs;

    private Timer timer;
    private Timer spinDownTimer;

    public TeleopOuttake(Outtake outtake, Intake intake, Arm arm, DoubleSupplier outtakeSpeed, Boolean useRPMs) {
        addRequirements(outtake);
        this.outtake = outtake;
        this.intake = intake;
        this.arm = arm;
        this.outtakeSpeed = outtakeSpeed;
        this.useRPMs = useRPMs;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        timer = new Timer();
        timer.restart();

        spinDownTimer = new Timer();
        spinDownTimer.restart();
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        if(useRPMs) {
            if(intake.getSecondaryNoteSensor().get()) {
                timer.restart();
            } else {
                spinDownTimer.restart();
            }
            if((!intake.getSecondaryNoteSensor().get() && timer.get() > 0.4) || (intake.getSecondaryNoteSensor().get() && spinDownTimer.get() < 0.5) && arm.getAbsoluteAdjustedPosition() < 90) {
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