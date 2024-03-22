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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PositionListener;

public class ApriltagCamera extends SubsystemBase {

    PhotonCamera camera;

    private double CAMERA_HEIGHT_METERS;
    private double TARGET_HEIGHT_METERS;

    // Angle between horizontal and the camera.
    private double CAMERA_PITCH_RADIANS;

    private AprilTagFieldLayout layout;
    private PhotonPoseEstimator estimator;

    private final List<PositionListener> listeners = new ArrayList<>();

    public ApriltagCamera(String cameraID, double CAMERA_HEIGHT_INCHES, double TARGET_HEIGHT_METERS, double CAMERA_PITCH_DEGREES) {
        this.camera = new PhotonCamera(cameraID);
        this.CAMERA_HEIGHT_METERS = Units.inchesToMeters(CAMERA_HEIGHT_INCHES);
        this.TARGET_HEIGHT_METERS = TARGET_HEIGHT_METERS;
        this.CAMERA_PITCH_RADIANS = Units.degreesToRadians(CAMERA_PITCH_DEGREES);
        this.layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        this.estimator = new PhotonPoseEstimator(
                                layout,
                                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                new Transform3d(
                                    new Translation3d(0, 0, 0),
                                    new Rotation3d(0, 0, 0)
                                    )
                                );
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has target", camera.getLatestResult().hasTargets());
        SmartDashboard.putNumber("rangeToTarget", Units.metersToInches(getTargetRange()));
        notifyPositionUpdate(getEstimatedGlobalPose());
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public double getTargetRange() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
            return range;
        } else {
            return 0;
        }
    }

    public double getSpecificTargetRange(PhotonTrackedTarget target) {
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(target.getPitch()));
        return range;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return estimator.update(getLatestResult());
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