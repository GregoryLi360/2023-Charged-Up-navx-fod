package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;;

public class Vision extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    public Vision() throws IOException {
        camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
        poseEstimator = new PhotonPoseEstimator(
            VisionConstants.APRIL_TAG_FIELD_LAYOUT,
            PoseStrategy.LOWEST_AMBIGUITY, 
            camera, 
            VisionConstants.ROBOT_TO_CAMERA
        );
    }

    public Optional<Transform3d> get() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            System.out.println("No targets\n");
            return Optional.empty();
        }

        PhotonTrackedTarget target = result.getBestTarget();

        /* Traditional distance using formula */
        double dist = PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.CAMERA_HEIGHT_METERS, 
            VisionConstants.APRILTAG_HEIGHT_METERS, 
            VisionConstants.CAMERA_PITCH_RADIANS, 
            Units.degreesToRadians(target.getPitch())
        );

        System.out.println("yaw: " + target.getYaw());
        System.out.println("range: " + dist);
        System.out.println("camera to target translation/transform: " + target.getBestCameraToTarget() + "\n");

        return Optional.of(target.getBestCameraToTarget());
    

    }
}