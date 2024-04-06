package frc.robot.commands;

import org.photonvision.PhotonUtils;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Swerve;


public class RelayShot extends Command {

    private Swerve swerve;
    private Outtake launcher;
    private Arm arm;
    private Intake intake;

    private double ARM_P = 0.03;
    private double ARM_I = 0.0005;
    private double ARM_D = 0;

    private double ANGULAR_P = 0.1; // 0.15
    private double ANGULAR_I = 0.2; // 0.3
    private double ANGULAR_D = 0;

    private PIDController armPositionController;
    private PIDController rotationController;

    private double armSetpoint = 50;
    private double launcherSetpoint = 2850;

    public RelayShot(Swerve swerve, Arm arm, Intake intake, Outtake launcher) {
        addRequirements(swerve, arm, intake, launcher);
        this.arm = arm;
        this.launcher = launcher;
        this.intake = intake;
        this.swerve = swerve;

        /*
         * Create PID controllers for the arm and rotation
         */
        armPositionController = new PIDController(ARM_P, ARM_I, ARM_D);
        rotationController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);
    }

    @Override
    public void initialize() {
        // Runs once on start
    
        /*
         * Closed loop PID for launcher
         */
        launcher.getOuttakePID().setP(0.0002);
        launcher.getOuttakePID().setI(0);
        launcher.getOuttakePID().setD(0);
        launcher.getOuttakePID().setFF(0.000175);
        launcher.getOuttakePID().setOutputRange(0, 1);

        armPositionController.setTolerance(1);
        armPositionController.setSetpoint(armSetpoint);

        launcher.getOuttakePID().setReference(launcherSetpoint, ControlType.kVelocity);

        rotationController.setTolerance(2);
        if(DriverStation.getAlliance().isPresent()) {
            if(DriverStation.getAlliance().get() == Alliance.Blue) {
                rotationController.setSetpoint(-30);
            } else {
                rotationController.setSetpoint(30);
            }
        } else {
            rotationController.setSetpoint(0);
        }

    }

    @Override
    public void execute() {
        arm.drive( // move the arm to it
                MathUtil.clamp(armPositionController.calculate(arm.getAbsoluteAdjustedPosition()), -1, 1));
                
        swerve.drive(
            new Translation2d(),
            rotationController.calculate(swerve.getHeadingModulus()),
            false,
            false
            );
        if(armPositionController.atSetpoint() && launcher.getOuttakeEncoder().getVelocity() > launcherSetpoint - 300 && rotationController.atSetpoint()) {
            intake.intake(1);
        } else {
            intake.intake(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        launcher.getOuttakePID().setP(0); // set p to 0 as to not brake the motor
        launcher.outtake(0);
        intake.intake(0);
        swerve.drive(new Translation2d(), 0, false, false);
        arm.drive(0);
    }

    @Override
    public boolean isFinished() {
        // Whether or not the command is finished
        return false;
    }
}