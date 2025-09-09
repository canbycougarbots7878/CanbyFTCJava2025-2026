package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@Autonomous(name = "AprilTag Auto (OTOS Fusion)", group = "Movement")
public class AprilTagMovement extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private SparkFunOTOS myOtos;
    private MovementLib.DriveWheels wheels;

    @Override
    public void runOpMode() {
        // --- Setup drive wheels ---
        wheels = new MovementLib.DriveWheels(hardwareMap);

        // --- Setup OTOS ---
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        myOtos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        // --- Setup AprilTag detection ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Init complete. Press start to begin.");
        telemetry.update();
        waitForStart();

        // --- Step 1: Look for a tag and set target ---
        double targetX = 0, targetY = 0;
        boolean targetSet = false;

        while (opModeIsActive() && !targetSet) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {
                // Loop through all detected tags to find one with a valid pose
                for (AprilTagDetection tag : detections) {
                    if (tag.ftcPose != null) {
                        // Get current robot pose from OTOS
                        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
                        double angleRad = Math.toRadians(pos.h);

                        // Transform tag pose to field coordinates
                        double rotatedX = tag.ftcPose.z * Math.cos(angleRad) - tag.ftcPose.x * Math.sin(angleRad);
                        double rotatedY = tag.ftcPose.z * Math.sin(angleRad) + tag.ftcPose.x * Math.cos(angleRad);

                        targetX = pos.x + rotatedX;
                        targetY = pos.y + rotatedY;

                        telemetry.addData("Target set from tag %d", tag.id);
                        telemetry.addData("Target Field Pos", "X=%.2f, Y=%.2f", targetX, targetY);
                        telemetry.update();

                        targetSet = true; // we have a valid target
                        break; // exit for loop once a valid pose is found
                    } else {
                        telemetry.addData("Tag %d detected but pose is null", tag.id);
                        telemetry.update();
                    }
                }
            } else {
                telemetry.addLine("Looking for AprilTag...");
                telemetry.update();
            }
        }

// --- Step 2: Drive to the target using OTOS ---
        double kP = 0.6;        // proportional gain
        double tolerance = 0.15; // meters
        double maxSpeed = 0.4;

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            double errorX = targetX - pos.x;
            double errorY = targetY - pos.y;

            double distance = Math.hypot(errorX, errorY);

            telemetry.addData("Current Pos", "X=%.2f, Y=%.2f", pos.x, pos.y);
            telemetry.addData("Target Pos", "X=%.2f, Y=%.2f", targetX, targetY);
            telemetry.addData("Errors", "dX=%.2f, dY=%.2f, Dist=%.2f", errorX, errorY, distance);

            if (distance > tolerance) {
                // proportional speeds
                double forward = Math.max(-maxSpeed, Math.min(maxSpeed, errorX * kP));
                double right   = Math.max(-maxSpeed, Math.min(maxSpeed, errorY * kP));
                double rotate  = 0; // optional: add heading correction here

                wheels.Omni_Move(forward, right, rotate, 1.0);
            } else {
                wheels.Stop_Wheels();
                telemetry.addLine("Reached AprilTag target!");
                telemetry.update();
                break;
            }

            telemetry.update();
        }

        wheels.Stop_Wheels();
        visionPortal.close();
    }
}
