package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photonvision extends SubsystemBase {

    PhotonCamera camera;

    private double CAMERA_HEIGHT_METERS;
    private double TARGET_HEIGHT_METERS;

    // Angle between horizontal and the camera.
    private double CAMERA_PITCH_RADIANS;

    public Photonvision(String cameraID, double CAMERA_HEIGHT_INCHES, double TARGET_HEIGHT_METERS, double CAMERA_PITCH_DEGREES) {
        this.camera = new PhotonCamera(cameraID);
        this.CAMERA_HEIGHT_METERS = Units.inchesToMeters(CAMERA_HEIGHT_INCHES);
        this.TARGET_HEIGHT_METERS = TARGET_HEIGHT_METERS;
        this.CAMERA_PITCH_RADIANS = Units.degreesToRadians(CAMERA_PITCH_DEGREES);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Has target", camera.getLatestResult().hasTargets());
        SmartDashboard.putNumber("rangeToTarget", Units.metersToInches(getTargetRange()));
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
}