package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDrivePoseEstimator mPoseEstimator; 
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    Orchestra orchestra = new Orchestra();

    private ApriltagCamera camera;

    private Field2d field;

    public boolean hasOptimalAngle = false;
    public double optimalAngle = 0;

    public boolean rotationAtSetpoint = false;

    public Swerve(ApriltagCamera camera) {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "CANivore");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        this.camera = camera;

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        for (TalonFX motor : getModuleDriveMotors()) {
            orchestra.addInstrument(motor, motor.getDeviceID() - 1);
        }
        for (TalonFX motor : getModuleAngleMotors()) {
            orchestra.addInstrument(motor, motor.getDeviceID() - 1);
        }

        mPoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d()
            );

        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    Constants.Swerve.maxSpeed, // Max module speed, in m/s
                    //TODO: Check math below
                    Math.sqrt(Math.pow(Constants.Swerve.trackWidth, 2) + Math.pow(Constants.Swerve.wheelBase, 2)), // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig(
                        true,
                        true
                        ) // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    
    public void driveRobotRelative(ChassisSpeeds translation) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                translation
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    private TalonFX[] getModuleDriveMotors() {
        TalonFX[] motors = new TalonFX[4];
        for (SwerveModule mod : mSwerveMods) {
            motors[mod.moduleNumber] = mod.getDriveMotor();
        }
        return motors;
    }

    private TalonFX[] getModuleAngleMotors() {
        TalonFX[] motors = new TalonFX[4];
        for (SwerveModule mod : mSwerveMods) {
            motors[mod.moduleNumber] = mod.getAngleMotor();
        }
        return motors;
    }

    public Orchestra getOrchestra() {
        return orchestra;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return mPoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        mPoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public SwerveDrivePoseEstimator getPoseEstimatior() {
        return mPoseEstimator;
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    /**
     * Gets degree heading wrapped to -180-180
     * @return Heading in degrees
     * 
     */
    public double getHeadingModulus() {
        return MathUtil.inputModulus(getHeading().getDegrees(), -180, 180);
    }

    public void setHeading(Rotation2d heading){
        mPoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        mPoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        mPoseEstimator.update(getGyroYaw(), getModulePositions());

        if(camera.targetAngle.isPresent()) {
            optimalAngle = getHeading().getDegrees() - camera.targetAngle.get() + 2.15;
            hasOptimalAngle = true;
        } else {
            optimalAngle = MathUtil.inputModulus(
                PhotonUtils.getYawToPose(getPose(), camera.getSpeakerPose()).getDegrees() - (getHeading().getDegrees() * -1) + 180,
                -180,
                180
                );
            hasOptimalAngle = true;
        }

        if(hasOptimalAngle) {
            SmartDashboard.putNumber("driveOptimalAngle", optimalAngle);
        }
        SmartDashboard.putNumber("CurrentHeading", getHeading().getDegrees());
        SmartDashboard.putNumber("yawToSpeaker", PhotonUtils.getYawToPose(getPose(), camera.getSpeakerPose()).getDegrees());

        SmartDashboard.putBoolean("rotationAtSetpoint", rotationAtSetpoint);

        Optional<EstimatedRobotPose> estimatedPose = camera.getEstimatedGlobalPose();
        if(estimatedPose.isPresent() && !DriverStation.isAutonomous()) {
            mPoseEstimator.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), estimatedPose.get().timestampSeconds);
            // setHeading(estimatedPose.get().estimatedPose.toPose2d().getRotation());
        }

        for(SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        field.setRobotPose(getPose());
    }
}