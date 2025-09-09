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
                AprilTagDetection tag = detections.get(0);


                if (tag.ftcPose != null) {
                    SparkFunOTOS.Pose2D pos = myOtos.getPosition();
                    double angleRad = Math.toRadians(pos.h);
                    double rotatedX = tag.ftcPose.z * Math.cos(angleRad) - tag.ftcPose.x * Math.sin(angleRad);
                    double rotatedY = tag.ftcPose.z * Math.sin(angleRad) + tag.ftcPose.x * Math.cos(angleRad);

                    targetX = pos.x + rotatedX;
                    targetY = pos.y + rotatedY;

                    telemetry.addData("Target set from tag %d", tag.id);
                    telemetry.addData("Target Field Pos", "X=%.2f, Y=%.2f", targetX, targetY);
                    telemetry.update();
                    targetSet = true;
                } else {
                    telemetry.addData("Tag %d detected, but pose is null", tag.id);
                    telemetry.update();
                }

                // Current OTOS pose
                SparkFunOTOS.Pose2D pos = myOtos.getPosition();

                // Convert AprilTag relative offset into field coordinates
                double angleRad = Math.toRadians(pos.h);
                double rotatedX = tag.ftcPose.z * Math.cos(angleRad) - tag.ftcPose.x * Math.sin(angleRad);
                double rotatedY = tag.ftcPose.z * Math.sin(angleRad) + tag.ftcPose.x * Math.cos(angleRad);

                targetX = pos.x + rotatedX;
                targetY = pos.y + rotatedY;

                telemetry.addData("Target set from tag %d", tag.id);
                telemetry.addData("Target Field Pos", "X=%.2f, Y=%.2f", targetX, targetY);
                telemetry.update();

                targetSet = true;
            } else {
                telemetry.addLine("Looking for AprilTag...");
                telemetry.update();
            }
        }

        // --- Step 2: Drive to the target using OTOS ---
        double kP = 0.6;   // proportional gain (tune this)
        double tolerance = 0.15; // stop within 15 cm

        while (opModeIsActive()) {
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

            double errorX = targetX - pos.x;
            double errorY = targetY - pos.y;

            telemetry.addData("Current Pos", "X=%.2f, Y=%.2f", pos.x, pos.y);
            telemetry.addData("Target Pos", "X=%.2f, Y=%.2f", targetX, targetY);
            telemetry.addData("Errors", "dX=%.2f, dY=%.2f", errorX, errorY);

            double forward = errorX * kP; // forward error = difference in X (OTOS forward axis)
            double right   = errorY * kP; // strafe error = difference in Y
            double rotate  = 0;           // could add heading correction later

            // Clamp speeds
            forward = Math.max(-0.4, Math.min(0.4, forward));
            right   = Math.max(-0.4, Math.min(0.4, right));

            if (Math.abs(errorX) > tolerance || Math.abs(errorY) > tolerance) {
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
