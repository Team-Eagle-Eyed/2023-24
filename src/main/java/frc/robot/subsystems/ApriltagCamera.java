package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PositionListener;

public class ApriltagCamera extends SubsystemBase {

    PhotonCamera camera;

    private double CAMERA_HEIGHT_METERS;
    private double TAG_HEIGHT_METERS;

    // Angle between horizontal and the camera.
    private double CAMERA_PITCH_RADIANS;

    private AprilTagFieldLayout layout;
    private PhotonPoseEstimator estimator;

    private final List<PositionListener> listeners = new ArrayList<>();

    public ApriltagCamera(String cameraID, double CAMERA_HEIGHT_INCHES, double TAG_HEIGHT_METERS, double CAMERA_PITCH_DEGREES) {
        this.camera = new PhotonCamera(cameraID);
        this.CAMERA_HEIGHT_METERS = Units.inchesToMeters(CAMERA_HEIGHT_INCHES);
        this.TAG_HEIGHT_METERS = TAG_HEIGHT_METERS;
        this.CAMERA_PITCH_RADIANS = Units.degreesToRadians(CAMERA_PITCH_DEGREES);
        this.layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        this.estimator = new PhotonPoseEstimator(
                                layout,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                this.camera,
                                new Transform3d(
                                    new Translation3d(
                                        Units.inchesToMeters(8),
                                        Units.inchesToMeters(0),
                                        Units.inchesToMeters(25)
                                        ),
                                    new Rotation3d(
                                        Units.degreesToRadians(0),
                                        Units.degreesToRadians(CAMERA_PITCH_DEGREES), // 17.5
                                        Units.degreesToRadians(180) // 180?
                                    )
                                )
                            );
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has target", camera.getLatestResult().hasTargets());
        SmartDashboard.putNumber("rangeToTarget", getTargetRange());
        // Removed vision contribution to pose estimation.
        /* if(getEstimatedGlobalPose().isPresent()) {
            notifyPositionUpdate(getEstimatedGlobalPose());
        } */
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public double getTargetRange() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TAG_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
            return range;
        } else {
            return 0;
        }
    }

    public double getEstimatedRangePose(Pose2d robotPose) {
        return PhotonUtils.getDistanceToPose(robotPose, getSpeakerPose());
    }

    public double getSpecificTargetRange(PhotonTrackedTarget target) {
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TAG_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(target.getPitch()));
        return range;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return estimator.update(getLatestResult());
    }

    public Pose2d getSpeakerPose() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) { // if alliance is red
            return layout.getTagPose(4).get().toPose2d(); // return position of tag 4 (red speaker middle)
        } else { // or if alliance is blue or no alliance
            return layout.getTagPose(7).get().toPose2d(); // return position of tag 7 (blue speaker middle)
        }
    }

    public void addPositionListener(PositionListener listener) {
        listeners.add(listener);
    }

    private void notifyPositionUpdate(Optional<EstimatedRobotPose> position) {
        for (PositionListener listener : listeners) {
            listener.onPositionUpdate(position);
        }
    }
}