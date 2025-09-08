package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name = "April Tag Movement", group = "Movement")
public class AprilTagMovement extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Build AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // 2. Attach to VisionPortal
        visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag).build();

        telemetry.addLine("Camera ready. Press start to scan for AprilTags.");
        telemetry.update();

        // 3. Wait for start
        waitForStart();

        // 4. Loop while active
        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            telemetry.addData("# of tags detected", currentDetections.size());
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Position", "X=%.2f, Y=%.2f, Z=%.2f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
            }

            telemetry.update();
            sleep(20);
        }
    }
}
