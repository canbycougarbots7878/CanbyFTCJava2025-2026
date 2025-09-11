package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

@Autonomous(name = "AprilTag Movement", group = "Movement")
public class AprilTagMovement extends LinearOpMode {

    private SparkFunOTOS myOtos;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Camera offset relative to OTOS center
    private static final double CAMERA_OFFSET_X = 0.18; // forward, meters
    private static final double CAMERA_OFFSET_Y = 0.03; // left, meters

    // AprilTag field position (known)
    private static final double APRILTAG_FIELD_X = 1.8288; // 6 ft on +X axis
    private static final double APRILTAG_FIELD_Y = 0.0;

    // Tag front faces the center of the field (0° in this setup)
    private static final double TAG_FRONT_HEADING = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Setup OTOS ---
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        // --- Setup AprilTag detection ---
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Init complete. Press start to begin tracking...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            double robotFieldX = Double.NaN;
            double robotFieldY = Double.NaN;
            double headingRelativeTagFront = Double.NaN;
            double robotForwardFieldAngle = Double.NaN;
            boolean tagSeen = false;

            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    if (tag.ftcPose != null) {
                        tagSeen = true;

                        // Convert tag pose from inches to meters
                        double tagX_m = tag.ftcPose.x * 0.0254;
                        double tagZ_m = tag.ftcPose.z * 0.0254;

                        // Apply camera offset
                        double correctedX_m = tagZ_m + CAMERA_OFFSET_X;
                        double correctedY_m = tagX_m + CAMERA_OFFSET_Y;

                        // Rotate into field coordinates
                        double angleRad = Math.toRadians(pos.h);
                        double fieldOffsetX = correctedX_m * Math.cos(angleRad) - correctedY_m * Math.sin(angleRad);
                        double fieldOffsetY = correctedX_m * Math.sin(angleRad) + correctedY_m * Math.cos(angleRad);

                        // Compute robot field position
                        robotFieldX = APRILTAG_FIELD_X - fieldOffsetX;
                        robotFieldY = APRILTAG_FIELD_Y - fieldOffsetY;

                        // --- Heading relative to tag front ---
                        headingRelativeTagFront = pos.h - TAG_FRONT_HEADING;
                        headingRelativeTagFront = ((headingRelativeTagFront + 180) % 360) - 180;

                        // --- Robot forward vector in field coordinates ---
                        double dx = robotFieldX - APRILTAG_FIELD_X;
                        double dy = robotFieldY - APRILTAG_FIELD_Y;
                        double angleTagToRobot = Math.toDegrees(Math.atan2(dy, dx));

                        robotForwardFieldAngle = angleTagToRobot + pos.h;
                        robotForwardFieldAngle = ((robotForwardFieldAngle + 180) % 360) - 180;

                        break; // use first valid tag
                    }
                }
            }

            // --- Telemetry ---
            telemetry.addData("Robot Field Pos (m)", "X=%.2f, Y=%.2f", robotFieldX, robotFieldY);
            telemetry.addData("Heading (° from +X)", pos.h);
            if (tagSeen) {
                telemetry.addData("Heading Relative to Tag Front (°)", "%.1f", headingRelativeTagFront);
                telemetry.addData("Robot Forward Vector (° Field)", "%.1f", robotForwardFieldAngle);
            } else {
                telemetry.addLine("AprilTag not detected.");
            }
            telemetry.update();

            sleep(50);
        }

        visionPortal.close();
    }
}
