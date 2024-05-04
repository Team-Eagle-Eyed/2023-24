package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ApriltagCamera;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private Intake s_Intake;
    private ApriltagCamera camera;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private PIDController rotationController;

    private Timer timer = new Timer();

    public TeleopSwerve(Swerve s_Swerve, Intake s_Intake, ApriltagCamera camera, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier targetCentricSup) {
        this.s_Swerve = s_Swerve;
        this.s_Intake = s_Intake;
        this.camera = camera;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void initialize() {
        rotationController = new PIDController(0.02, 0, 0.001); // 0.3 I?
        rotationController.setTolerance(0.5);
        timer.start();
    }

    @Override
    public void execute() {
        // SmartDashboard.putNumber("rangeToTarget", Units.metersToFeet(camera.getEstimatedRangePose(s_Swerve.getPose())));

        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        if(rotationVal != 0) {
            timer.restart();
        } else if (timer.get() < 1) {
            rotationVal = 0;
        } else if (timer.get() > 1 && !s_Intake.getSecondaryNoteSensor().get()) {
            if(s_Swerve.hasOptimalAngle) {
                rotationController.setSetpoint(s_Swerve.optimalAngle);
                rotationVal = rotationController.calculate(s_Swerve.getHeading().getDegrees());
            } else {
                rotationVal = 0;
            }
        }
        s_Swerve.rotationAtSetpoint = rotationController.atSetpoint();

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );

        /* if(camera.getEstimatedGlobalPose().isPresent()) {
            s_Swerve.getPoseEstimatior().addVisionMeasurement(camera.getEstimatedGlobalPose().get().estimatedPose.toPose2d(), camera.getEstimatedGlobalPose().get().timestampSeconds);
        } */
    }
}