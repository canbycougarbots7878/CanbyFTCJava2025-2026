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
//this is a test

@Autonomous(name = "AprilTag Movement", group = "Movement")
public class AprilTagMovement extends LinearOpMode {

    // Camera offset relative to OTOS origin, in robot-centric meters
    private static final double CAMERA_OFFSET_X = 0.18; // forward
    private static final double CAMERA_OFFSET_Y = 0.03; // left

    // AprilTag known field position
    private static final double APRILTAG_FIELD_X = (3*0.3048)-((35*Math.sin(Math.acos((Math.pow(58.7,2)-Math.pow(70,2)-Math.pow(59,2))/(-2*70*59))))/100);
    private static final double APRILTAG_FIELD_Y = (3*0.3048)-((35*Math.cos(Math.acos((Math.pow(58.7,2)-Math.pow(70,2)-Math.pow(59,2))/(-2*70*59))))/100);

    // Tag orientation (front faces field center)
    private static final double TAG_FRONT_HEADING = 0;

    @Override
    public void runOpMode() {

        // --- Setup OTOS ---
        SparkFunOTOS myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        // --- Setup AprilTag detection ---
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
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

                        robotFieldX = APRILTAG_FIELD_X + relX;
                        robotFieldY = APRILTAG_FIELD_Y + relY;
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
