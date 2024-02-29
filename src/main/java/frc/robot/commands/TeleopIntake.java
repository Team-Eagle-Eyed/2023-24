package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;


public class TeleopIntake extends Command {

    private Intake intake;
    private DoubleSupplier intakeSpeed;
    private DigitalInput noteSensor;
    private boolean useRPMs;

    public TeleopIntake(Intake intake, DoubleSupplier intakeSpeed, boolean useRPMs) {
        addRequirements(intake);
        this.intake = intake;
        this.intakeSpeed = intakeSpeed;
        this.noteSensor = intake.getNoteSensor();
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
        if(useRPMs && noteSensor.get()) {
            intake.getIntakePID().setReference(intakeSpeed.getAsDouble(), ControlType.kVelocity);
        } else if (noteSensor.get()) {
            intake.intake(output);
        } else if (output < 0 && useRPMs) {
            intake.intake(output);
        } else if (!noteSensor.get()){
            intake.intake(-0.25);
        } else {
            intake.intake(0);
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