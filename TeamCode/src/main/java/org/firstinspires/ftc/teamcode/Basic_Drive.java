package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Basic Drive", group = "Movement")
public class Basic_Drive extends LinearOpMode {
    DcMotor Front_Right = null;
    DcMotor Front_Left = null;
    DcMotor Back_Right = null;
    DcMotor Back_Left = null;
    MovementLib.Robot robot = null;
    public void runOpMode() {
        Front_Right = hardwareMap.get(DcMotor.class, "frontright");
        Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
        Back_Right = hardwareMap.get(DcMotor.class, "backright");
        Back_Left = hardwareMap.get(DcMotor.class, "backleft");

        robot = new MovementLib.Robot(Front_Right, Front_Left, Back_Right, Back_Left); // Initialize Wheels handler
        robot.Reverse_Left(); // Make all motors spin forward

        waitForStart();
        while(opModeIsActive()) {
            double forward = gamepad1.left_stick_y;
            double strafe = - gamepad1.left_stick_x;
            double speed = (gamepad1.right_bumper ? 1.0 : 0.5);
            robot.Omni_Move(forward, strafe, gamepad1.right_stick_x, speed);
        }
    }
}
