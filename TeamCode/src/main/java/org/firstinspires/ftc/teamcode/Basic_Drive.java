package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Basic Drive", group = "Movement")
public class Basic_Drive extends LinearOpMode {
    DcMotor Front_Right = null;
    DcMotor Front_Left = null;
    DcMotor Back_Right = null;
    DcMotor Back_Left = null;
    MovementLib.DriveWheels Wheels = null;
    public void runOpMode() {
        Front_Right = hardwareMap.get(DcMotor.class, "frontright");
        Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
        Back_Right = hardwareMap.get(DcMotor.class, "backright");
        Back_Left = hardwareMap.get(DcMotor.class, "backleft");

        Wheels = new MovementLib.DriveWheels(Front_Right, Front_Left, Back_Right, Back_Left); // Initialize Wheels handler
        Wheels.Reverse_Left(); // Make all motors spin forward

        waitForStart();
        while(opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
            double strafe = - gamepad1.left_stick_x;
            Wheels.Omni_Move(forward, strafe, gamepad1.right_stick_x, 0.5);
        }
    }
}
