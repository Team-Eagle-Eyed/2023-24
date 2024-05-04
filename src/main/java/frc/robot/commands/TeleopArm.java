package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;


public class TeleopArm extends Command {

    private Arm arm;
    private Intake intake;
    private DoubleSupplier speed;

    private PIDController positionController;
    private Timer timer = new Timer();

    public TeleopArm(Arm arm, Intake intake, DoubleSupplier speed) {
        addRequirements(arm);
        this.arm = arm;
        this.intake = intake;
        this.speed = speed;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        positionController = new PIDController(0.03, 0.0005, 0);
        positionController.setTolerance(1);
        timer.start();
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        double output = MathUtil.applyDeadband(speed.getAsDouble(), Constants.stickDeadband);
        if(output != 0) {
            timer.restart();
            arm.drive(output);
        } else if (timer.get() < 1) {
            arm.drive(0);
        } else if (timer.get() > 1 && !intake.getSecondaryNoteSensor().get()) {
            if(arm.hasOptimalAngle) {
                positionController.setSetpoint(arm.optimalAngle);
                arm.drive(positionController.calculate(arm.getAbsoluteAdjustedPosition()));
            } else {
                arm.drive(0);
            }
        } else if (timer.get() > 1 && intake.getSecondaryNoteSensor().get()) {
            arm.drive(-0.2);
        }
        
        arm.atSetpoint = positionController.atSetpoint();
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