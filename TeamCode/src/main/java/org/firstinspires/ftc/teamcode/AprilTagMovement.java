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

    // Camera offset relative to OTOS origin, in robot-centric meters
    private static final double CAMERA_OFFSET_X = 0.18; // forward
    private static final double CAMERA_OFFSET_Y = 0.03; // left

    // AprilTag known field position
    private static final double APRILTAG_FIELD_X = 6;   // ft
    private static final double APRILTAG_FIELD_Y = 0.0; // ft

    // Tag orientation (front faces field center)
    private static final double TAG_FRONT_HEADING = 0;

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

        boolean tagFixed = false;

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            double robotFieldX = pos.x;
            double robotFieldY = pos.y;
            double robotFieldH = pos.h;

            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagSeen = false;

            if (!tagFixed && !detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    if (tag.ftcPose != null) {
                        tagSeen = true;

                        // --- Compute corrected pose as before ---
                        double robotHrad = Math.toRadians(pos.h);
                        double camX = pos.x + (CAMERA_OFFSET_X * Math.cos(robotHrad)
                                - CAMERA_OFFSET_Y * Math.sin(robotHrad));
                        double camY = pos.y + (CAMERA_OFFSET_X * Math.sin(robotHrad)
                                + CAMERA_OFFSET_Y * Math.cos(robotHrad));

                        double tagX = tag.ftcPose.x * 0.0254;
                        double tagY = tag.ftcPose.y * 0.0254;
                        double relX = camX - tagX;
                        double relY = camY - tagY;
                        double relH = pos.h - tag.ftcPose.yaw;

                        double fieldTagX = APRILTAG_FIELD_X * 0.3048;
                        double fieldTagY = APRILTAG_FIELD_Y * 0.3048;

                        robotFieldX = fieldTagX + relX;
                        robotFieldY = fieldTagY + relY;
                        robotFieldH = TAG_FRONT_HEADING + relH;

                        // --- Apply fix ONCE ---
                        myOtos.setPosition(new SparkFunOTOS.Pose2D(robotFieldX, robotFieldY, robotFieldH));
                        tagFixed = true;

                        break;
                    }
                }
            }

            // --- Telemetry ---
            telemetry.addData("Robot Field Pos (m)", "X=%.2f, Y=%.2f", robotFieldX, robotFieldY);
            telemetry.addData("Heading (° from +X)", robotFieldH);
            if (tagSeen && !tagFixed) telemetry.addLine("AprilTag detected → Pose corrected.");
            else telemetry.addLine("Odometry running...");
            telemetry.update();

            sleep(50);
        }

        visionPortal.close();
    }
}
