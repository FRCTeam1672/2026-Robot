package frc.robot.subsystems.swervedrive;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import swervelib.SwerveDrive;

public class VisionSubsystem extends SubsystemBase {
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private final PhotonCamera leftCam = new PhotonCamera("1672_Camera1");
    private final PhotonCamera rightCam = new PhotonCamera("1672_Camera2");

    private final Transform3d leftCamPos = new Transform3d(new Translation3d(Units.inchesToMeters(-11),
            Units.inchesToMeters(-12),
            Units.inchesToMeters(8.5)),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(225)));

    private final Transform3d rightCamPos = new Transform3d(new Translation3d(Units.inchesToMeters(11),
            Units.inchesToMeters(-12),
            Units.inchesToMeters(8.5)),
            new Rotation3d(0, Math.toRadians(0), Math.toRadians(135)));



    private Trigger rightCamTrigger = new Trigger(rightCam::isConnected);
    private Trigger leftCamTrigger = new Trigger(leftCam::isConnected);


    // Construct PhotonPoseEstimator
    private final PhotonPoseEstimator leftPoseEst = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamPos);
    private final PhotonPoseEstimator rightPoseEst = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamPos);

    /** Creates a new VisionSubsystem. */
    public VisionSubsystem() {
        rightCamTrigger.onFalse(Commands.runOnce(() -> {
            Elastic.sendNotification(
                new Notification(Notification.NotificationLevel.ERROR, "right Camera Disconnect", "right camera disconnect").withDisplaySeconds(5)
            );
        }));
        leftCamTrigger.onFalse(Commands.runOnce(() -> {
            Elastic.sendNotification(
                new Notification(Notification.NotificationLevel.ERROR, "left Camera Disconnect", "left camera disconnect").withDisplaySeconds(5)
            );
        }));

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("vision/right Camera Connected", rightCamTrigger.getAsBoolean());
        SmartDashboard.putBoolean("vision/left Camera Connected", leftCamTrigger.getAsBoolean());
    }

    public void updatePoseEstimation(SwerveDrive swerve) {
        List<PhotonPipelineResult> leftResults = leftCam.getAllUnreadResults();
        List<PhotonPipelineResult> rightResults = rightCam.getAllUnreadResults();
        if (!leftResults.isEmpty()) {
            PhotonPipelineResult leftPhotonPipelineResult = leftResults.get(0);
            Optional<EstimatedRobotPose> leftUpdate = leftPoseEst.update(leftPhotonPipelineResult);
            if (leftUpdate.isPresent()) {
                if(leftPhotonPipelineResult.getBestTarget().poseAmbiguity < 0.4) {
                    EstimatedRobotPose estimatedRobotPose = leftUpdate.get();
                    swerve.field.getObject("leftPose").setPose(estimatedRobotPose.estimatedPose.toPose2d());
                    swerve.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
                            leftPhotonPipelineResult.getTimestampSeconds());
                }
            }
        }
        if(!rightResults.isEmpty()) {
            PhotonPipelineResult rightPhotonPipelineResult = rightResults.get(0);
            Optional<EstimatedRobotPose> rightUpdate = rightPoseEst.update(rightPhotonPipelineResult);
            if (rightUpdate.isPresent()) {
                if(rightPhotonPipelineResult.getBestTarget().poseAmbiguity < 0.4) {
                EstimatedRobotPose estimatedRobotPose = rightUpdate.get();
                swerve.field.getObject("rightPose").setPose(estimatedRobotPose.estimatedPose.toPose2d());
                swerve.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
                        rightPhotonPipelineResult.getTimestampSeconds());
                }
            }
        }
        
    }
}