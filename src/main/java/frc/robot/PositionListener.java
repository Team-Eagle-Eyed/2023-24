package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public interface PositionListener {
    void onPositionUpdate(Optional<EstimatedRobotPose> position);
}