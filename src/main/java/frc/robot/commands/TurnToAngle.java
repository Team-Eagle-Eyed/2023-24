package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;



public class TurnToAngle extends Command {

    private Swerve m_Swerve;
    private PIDController rotationController;
    private DoubleSupplier angle;

    public TurnToAngle(Swerve m_swerve, DoubleSupplier angle) {
        addRequirements(m_swerve);
        this.m_Swerve = m_swerve;
        this.angle = angle;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
        rotationController = new PIDController(0.1, 0, 0);
        rotationController.setTolerance(0.5);
        rotationController.setSetpoint(angle.getAsDouble());
        /* if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            rotationController.setSetpoint(-90);
        } else {
            rotationController.setSetpoint(90);
        } */
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        m_Swerve.drive(
            new Translation2d(),
            rotationController.calculate(m_Swerve.getHeading().getDegrees()),
            false,
            false
            );
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        m_Swerve.drive(new Translation2d(), 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return rotationController.atSetpoint();
        // Whether or not the command is finished
    }
}