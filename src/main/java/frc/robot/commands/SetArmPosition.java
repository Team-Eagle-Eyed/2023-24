package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class SetArmPosition extends Command {

    private Arm arm;
    private DoubleSupplier position;
    private PIDController positionController;

    public SetArmPosition(Arm arm, DoubleSupplier position) {
        addRequirements(arm);
        this.arm = arm;
        this.position = position;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        positionController = new PIDController(0.03, 0.0005, 0);
        positionController.setSetpoint(position.getAsDouble());
        positionController.setTolerance(1);
    }
    
    @Override
    public void execute() {
        arm.drive(MathUtil.clamp(positionController.calculate(arm.getAbsoluteAdjustedPosition()), -1, 1));
        SmartDashboard.putBoolean("armAtSetpoint", positionController.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        positionController.close();
        // arm.getPIDController().setReference(0, ControlType.kVelocity);
    }

    @Override
    public boolean isFinished() {
        // Whether or not the command is finished
        return positionController.atSetpoint();
        // return false;
    }
}