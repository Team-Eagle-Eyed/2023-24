package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Swerve;

public class Estop extends Command {

    private Swerve m_swerve;
    private Arm m_arm;
    private Intake m_intake;
    private Outtake m_outtake;

    public Estop(Swerve m_swerve, Arm m_arm, Intake m_intake, Outtake m_outtake) {
        addRequirements(m_swerve, m_arm, m_intake, m_outtake);
        this.m_swerve = m_swerve;
        this.m_arm = m_arm;
        this.m_intake = m_intake;
        this.m_outtake = m_outtake;
    }
    
    @Override
    public void initialize() {
        // Runs once on start
    }

    @Override
    public void execute() {
        // Runs repeatedly after initialization
        m_swerve.drive(new Translation2d(0, 0), 0, false, true);
        m_arm.drive(0);
        m_intake.intake(0);
        m_outtake.outtake(0);
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