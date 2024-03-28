package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ApriltagCamera;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {
    private Swerve s_Swerve;    
    private ApriltagCamera camera;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier targetCentricSup;

    private PIDController rotationController;

    public TeleopSwerve(Swerve s_Swerve, ApriltagCamera camera, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier targetCentricSup) {
        this.s_Swerve = s_Swerve;
        this.camera = camera;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.targetCentricSup = targetCentricSup;
    }

    @Override
    public void initialize() {
        rotationController = new PIDController(0.15, 0, 0); // 0.3 I?
        double setpoint;
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
            Pose2d pose = s_Swerve.getPose();
            double xDistance = pose.getX() - (-0.04);
            double yDistance = pose.getY() - 5.55;
        }
    }

    @Override
    public void execute() {
        // SmartDashboard.putNumber("rangeToTarget", Units.metersToFeet(camera.getEstimatedRangePose(s_Swerve.getPose())));

        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
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