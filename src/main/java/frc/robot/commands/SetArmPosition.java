package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class SetArmPosition extends Command {

    private Arm arm;
    private DoubleSupplier position;

    public SetArmPosition(Arm arm, DoubleSupplier position) {
        addRequirements(arm);
        this.arm = arm;
        this.position = position;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        arm.getPIDController().setP(0.0002);
        arm.getPIDController().setI(0);
        arm.getPIDController().setD(0);
        arm.getPIDController().setOutputRange(0, 1);
    }
    
    @Override
    public void execute() {
        arm.getPIDController().setFF(Math.cos(Units.degreesToRadians(arm.getAbsoluteEncoder().getPosition() - 180)) * 0.015);
        arm.getPIDController().setReference(position.getAsDouble(), ControlType.kVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        arm.getPIDController().setP(0);
        // arm.getPIDController().setReference(0, ControlType.kVelocity);
    }

    @Override
    public boolean isFinished() {
        return false;
        // Whether or not the command is finished
    }
}