package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ApriltagCamera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private Arm s_Arm;
    private Intake s_Intake;
    private ApriltagCamera camera;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private PIDController rotationController;

    private Orchestra orchestra;

    private Timer timer = new Timer();

    private Timer offTime = new Timer();

    public TeleopSwerve(Swerve s_Swerve, Arm s_Arm, Intake s_Intake, ApriltagCamera camera, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier targetCentricSup) {
        this.s_Swerve = s_Swerve;
        this.s_Arm = s_Arm;
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
        rotationController = new PIDController(0.017, 0, 0.0025); // 0.3 I?
        rotationController.setTolerance(1.9);
        timer.restart();
        offTime.restart();

        orchestra = s_Swerve.getOrchestra();

        SmartDashboard.putNumber("rotationP", rotationController.getP());
        SmartDashboard.putNumber("rotationI", rotationController.getI());
        SmartDashboard.putNumber("rotationD", rotationController.getD());
    }

    @Override
    public void execute() {
        // SmartDashboard.putNumber("rangeToTarget", Units.metersToFeet(camera.getEstimatedRangePose(s_Swerve.getPose())));

        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        rotationController.setP(SmartDashboard.getNumber("rotationP", 0));
        rotationController.setI(SmartDashboard.getNumber("rotationI", 0));
        rotationController.setD(SmartDashboard.getNumber("rotationD", 0));

        /* Drive */
        if(rotationVal != 0) {
            timer.restart();
            rotationController.reset();
        } else if (timer.get() < 1) {
            rotationVal = 0;
        } else if (timer.get() > 1 && !s_Intake.getSecondaryNoteSensor().get() && s_Arm.getAbsoluteAdjustedPosition() < 90) {
            offTime.restart();
            if(s_Swerve.hasOptimalAngle) {
                rotationController.setSetpoint(s_Swerve.optimalAngle);
                rotationVal = rotationController.calculate(s_Swerve.getHeading().getDegrees());
            } else {
                rotationVal = 0;
            }
        }
        s_Swerve.rotationAtSetpoint = rotationController.atSetpoint();

        
        SmartDashboard.putNumber("currentRotationP", rotationController.getP());
        SmartDashboard.putNumber("currentRotationError", rotationController.getPositionError());
        SmartDashboard.putNumber("currentRotationOutput", rotationVal);

        if(translationVal > 0 || strafeVal > 0 || rotationVal > 0) {
            offTime.restart();
        }

        if(offTime.get() > 30) {
            if(!orchestra.isPlaying()) {
                orchestra.loadMusic("william_tell_overture_finale.chrp");
                orchestra.play();
            }
        } else {
            if(orchestra.isPlaying()) {
                orchestra.stop();
            }
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
        }

        /* if(camera.getEstimatedGlobalPose().isPresent()) {
            s_Swerve.getPoseEstimatior().addVisionMeasurement(camera.getEstimatedGlobalPose().get().estimatedPose.toPose2d(), camera.getEstimatedGlobalPose().get().timestampSeconds);
        } */
    }
}