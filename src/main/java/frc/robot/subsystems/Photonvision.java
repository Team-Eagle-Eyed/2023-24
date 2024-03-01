package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photonvision extends SubsystemBase {

    PhotonCamera camera;

    private double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    private double TARGET_HEIGHT_METERS = 1.45;

    // Angle between horizontal and the camera.
    private double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    public Photonvision(String cameraID, double CAMERA_HEIGHT_METERS, double TARGET_HEIGHT_METERS, double CAMERA_PITCH_RADIANS) {
        this.camera = new PhotonCamera(cameraID);
        this.CAMERA_HEIGHT_METERS = CAMERA_HEIGHT_METERS;
        this.TARGET_HEIGHT_METERS = TARGET_HEIGHT_METERS;
        this.CAMERA_PITCH_RADIANS = CAMERA_PITCH_RADIANS;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Has target", camera.getLatestResult().hasTargets());
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
}