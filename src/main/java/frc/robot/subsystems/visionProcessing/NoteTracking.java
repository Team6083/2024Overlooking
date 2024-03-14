// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.visionProcessing;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.NoteTrackingConstants;

public class NoteTracking {
    /** Creates a new VisionTrackingPhotovision. */

    private final boolean driveMode = false;
    private final PhotonCamera noteCamera;

    public NoteTracking() {
        noteCamera = new PhotonCamera(NoteTrackingConstants.kCameraName);
        noteCamera.setPipelineIndex(NoteTrackingConstants.noteTrakingPipeline);
        noteCamera.setDriverMode(driveMode);
    }

    public boolean hasTargets() {
        return noteCamera.getLatestResult().hasTargets();
    }

    public PhotonPipelineResult getPipelineResult() {
        return noteCamera.getLatestResult();
    }

    public PhotonTrackedTarget getBestTarget() {
        return getPipelineResult().getBestTarget();
    }

    public double getYaw() {
        return getBestTarget().getYaw();
    }

    public double getPitch() {
        return getBestTarget().getPitch();
    }

    public Transform2d getCamToTarget() {
        double yaw = getYaw();
        double pitch = getPitch();
        double camPitch = NoteTrackingConstants.pitchDegree; // need to fix, positvie up
        // double camYaw = 30; // need to fix, positive right
        double cam_Y_Offset = NoteTrackingConstants.kCamYOffset; // need to fix, sideway
        double realPitch = pitch + camPitch;

        double camHeight = 0.2; // need to fix
        double x = camHeight / Math.tan(Math.toRadians(realPitch));
        double y = x*Math.tan(Math.toRadians(yaw));
        double realY = y-cam_Y_Offset;
        double angle = Math.atan(Math.atan(realY/x));
        Rotation2d rot = new Rotation2d(angle);
        Translation2d translation = new Translation2d(x, y);
        Transform2d transform = new Transform2d(translation, rot);
        return transform;
    }

    public double getToNoteDistance(){
        double x = getCamToTarget().getX();
        double y = getCamToTarget().getY();
        // double radian = getCamToTarget().getRotation().getRadians();
        double distance = Math.sqrt(x*x+y*y);
        return distance;
    }

    /**
     * Return a list of 2 dimensional note pose. X axis by bot's front, Y axis by
     * bot's sideways. Pitch is up and down angle. Yaw is right and left.
     * 
     * @return {@link Pose2d} note pose
     */
    public List<Pose2d> getNotePose() {
        List<Pose2d> poses = new ArrayList<Pose2d>();

        List<PhotonTrackedTarget> targets = getTargetList();

        for (PhotonTrackedTarget trackedTarget : targets) {
            double pitch = trackedTarget.getPitch();
            double yaw = trackedTarget.getYaw();
            double y = NoteTrackingConstants.cameraHeight
                    * (1 / Math.tan(Math.toRadians(pitch - NoteTrackingConstants.pitchDegree)));
            double x = y * Math.tan(Math.toRadians(yaw - NoteTrackingConstants.yawDegree))
                    - NoteTrackingConstants.cameraWeight;
            poses.add(new Pose2d(x, y, new Rotation2d(0)));
            SmartDashboard.putNumber("noteYaw", trackedTarget.getYaw());
            SmartDashboard.putNumber("notePitch", trackedTarget.getPitch());
            // SmartDashboard.putNumber("noteSkew", trackedTarget.getSkew());
        }
        SmartDashboard.putNumber("noteFound", poses.size());
        if (poses.size() > 0) {
            Pose2d notePose = poses.get(0);
            SmartDashboard.putNumber("noteVision/x", notePose.getX());
            SmartDashboard.putNumber("noteVision/y", notePose.getY());
        }
        return poses;
    }

    public List<PhotonTrackedTarget> getTargetList() {
        if (!noteCamera.isConnected()) {
            System.out.println("Camera not connected");
            return null;
        }

        var results = noteCamera.getLatestResult();
        if (!results.hasTargets()) {
            System.out.println("No target");
            return null;
        }
        return results.getTargets();
    }

    public List<Double> getTx() {
        List<PhotonTrackedTarget> targets = getTargetList();
        List<Double> notesYaw = new ArrayList<Double>();
        double yaw;
        for (PhotonTrackedTarget trackedTarget : targets) {
            yaw = trackedTarget.getYaw();
            notesYaw.add(yaw);
            SmartDashboard.putNumber("noteYaw", trackedTarget.getYaw());
            SmartDashboard.putNumber("notePitch", trackedTarget.getPitch());
            // SmartDashboard.putNumber("noteSkew", trackedTarget.getSkew());
        }
        return notesYaw;
    }

    public Pose2d getLastPose() {
        return getNotePose().get(0);
    }

    public void clearTagSolutions(Field2d field) {
        if (field == null)
            return;
        field.getObject("tagSolutions").setPoses();
        field.getObject("visionPose").setPoses();
        field.getObject("visionAltPose").setPoses();
        field.getObject("visibleTagPoses").setPoses();
    }

    public void plotPose(Field2d field, String label, Pose2d pose) {
        if (field == null)
            return;
        if (pose == null)
            field.getObject(label).setPoses();
        else
            field.getObject(label).setPose(pose);
    }

}
