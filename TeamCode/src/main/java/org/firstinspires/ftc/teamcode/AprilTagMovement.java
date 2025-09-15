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
    private static final double APRILTAG_FIELD_X = 6; // +X axis
    private static final double APRILTAG_FIELD_Y = 0.0;

    // Tag front faces the center of the field (180° in this setup)
    private static final double TAG_FRONT_HEADING = 180;

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
            double robotFieldH = Double.NaN;
            boolean tagSeen = false;

            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    if (tag.ftcPose != null) {
                        tagSeen = true;

                        double RobotX = pos.x;
                        double RobotY = pos.y;
                        double RobotH = pos.h;

                        double RobotCameraPositionY = RobotX + CAMERA_OFFSET_X;
                        double RobotCameraPositionX = RobotY + CAMERA_OFFSET_Y;
                        double RobotCameraPositionH = RobotH;

                        double TagPositionX = tag.ftcPose.x;
                        double TagPositionY = tag.ftcPose.y;
                        double TagPositionH = tag.ftcPose.yaw;

                        double TagPositionXMeter = TagPositionX * 0.0254;
                        double TagPositionYMeter = TagPositionY * 0.0254;

                        double Robot_TagPositionX = RobotCameraPositionX - TagPositionXMeter;
                        double Robot_TagPositionY = RobotCameraPositionY - TagPositionYMeter;
                        double Robot_TagPositionH = RobotCameraPositionH - TagPositionH;

                        double AprilTagFieldXMeter = APRILTAG_FIELD_X * 0.3048;
                        double AprilTagFieldYMeter = APRILTAG_FIELD_Y * 0.3048;

                        robotFieldX = Robot_TagPositionX + AprilTagFieldXMeter;
                        robotFieldY = Robot_TagPositionY + AprilTagFieldYMeter;
                        robotFieldH = Robot_TagPositionH + TAG_FRONT_HEADING;

                        break; // use first valid tag
                    }
                }
            }

            // --- Telemetry ---

            if (tagSeen) {
                telemetry.addData("Robot Field Pos (m)", "X=%.2f, Y=%.2f", robotFieldX, robotFieldY);
                telemetry.addData("Heading (° from +X)", robotFieldH);
            } else {
                telemetry.addLine("AprilTag not detected.");
            }
            telemetry.update();

            sleep(50);
        }

        visionPortal.close();
    }
}
