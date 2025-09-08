package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import java.lang.Math;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Player Relative Drive", group = "Movement")
public class PRM_Drive extends LinearOpMode {
    DcMotor Front_Right = null;
    DcMotor Front_Left = null;
    DcMotor Back_Right = null;
    DcMotor Back_Left = null;

    MovementLib.DriveWheels Wheels = null;

    IMU imu;
    public void runOpMode() {
        Front_Right = hardwareMap.get(DcMotor.class, "frontright");
        Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
        Back_Right = hardwareMap.get(DcMotor.class, "backright");
        Back_Left = hardwareMap.get(DcMotor.class, "backleft");

        Wheels = new MovementLib.DriveWheels(Front_Right, Front_Left, Back_Right, Back_Left); // Initialize Wheels handler
        Wheels.Reverse_Left(); // Make all motors spin forward
        imu = hardwareMap.get(IMU.class, "imu");

        waitForStart();
        while(opModeIsActive()) {
            // Get local directions
            double localForward = gamepad1.left_stick_y;
            double localRight = - gamepad1.left_stick_x;

            // Read IMU data
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw() * 0.01745329251; // Get robot yaw converted to radians

            // Calculate global directions using 2D rotation matrix
            double forward = localForward * Math.cos(yaw) - localRight * Math.sin(yaw);
            double right = localForward * Math.sin(yaw) + localRight * Math.cos(yaw);

            Wheels.Omni_Move(forward, right, gamepad1.right_stick_x, (gamepad1.right_bumper ? 1.0 : 0.5));

            if(gamepad1.start) imu.resetYaw();
        }
    }
}
