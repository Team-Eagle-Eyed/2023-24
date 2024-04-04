package frc.robot.commands;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;


public class AmpShot extends Command {

    private Outtake launcher;
    private Arm arm;
    private Intake intake;

    private double ARM_P = 0.03;
    private double ARM_I = 0.0005;
    private double ARM_D = 0;

    private PIDController armPositionController;

    private Timer finishedTimer = new Timer();

    public AmpShot(Arm arm, Intake intake, Outtake launcher) {
        addRequirements(arm, intake, launcher);
        this.arm = arm;
        this.launcher = launcher;
        this.intake = intake;

        /*
         * Create PID controllers for the arm and rotation
         */
        armPositionController = new PIDController(ARM_P, ARM_I, ARM_D);
    }

    @Override
    public void initialize() {
        // Runs once on start
        finishedTimer.start();
    
        /*
         * Closed loop PID for launcher
         */
        launcher.getOuttakePID().setP(0.0002);
        launcher.getOuttakePID().setI(0);
        launcher.getOuttakePID().setD(0);
        launcher.getOuttakePID().setFF(0.000175);
        launcher.getOuttakePID().setOutputRange(0, 1);

        armPositionController.setTolerance(1);
        armPositionController.setSetpoint(132);

    }

    @Override
    public void execute() {
        arm.drive( // move the arm to it
                MathUtil.clamp(armPositionController.calculate(arm.getAbsoluteAdjustedPosition()), -1, 1));
        if(armPositionController.atSetpoint()) {
            intake.intake(1);
            launcher.getOuttakePID().setReference(500, ControlType.kVelocity);
        } else {
            intake.intake(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        launcher.getOuttakePID().setP(0); // set p to 0 as to not brake the motor
        launcher.outtake(0);
        finishedTimer.stop();
    }

    @Override
    public boolean isFinished() {
        /* if(finishedTimer.get() > 1) {
            return true;
        } else {
            return false;
        } */
        return false;
        // Whether or not the command is finished
    }
}