package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MovementLib {
    public static class DriveWheels {
        public DcMotor Front_Right;
        public DcMotor Front_Left;
        public DcMotor Back_Right;
        public DcMotor Back_Left;

        public DriveWheels(DcMotor Front_Right, DcMotor Front_Left, DcMotor Back_Right, DcMotor Back_Left) {
            this.Front_Right = Front_Right;
            this.Front_Left = Front_Left;
            this.Back_Right = Back_Right;
            this.Back_Left = Back_Left;
        }

        public void Set_Wheels(double Front_Right_Power, double Front_Left_Power, double Back_Right_Power, double Back_Left_Power) {
            this.Front_Right.setPower(Front_Right_Power);
            this.Front_Left.setPower(Front_Left_Power);
            this.Back_Right.setPower(Back_Right_Power);
            this.Back_Left.setPower(Back_Left_Power);
        }

        public void Omni_Move(double Forward, double Right, double RotateCC, double speed) {
            double fl = Forward + Right - RotateCC;
            double fr = -Forward - Right + RotateCC;
            double bl = Forward - Right - RotateCC;
            double br = -Forward + Right + RotateCC;

            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(br), Math.abs(bl)))));

            fl /= max;
            fr /= max;
            br /= max;
            bl /= max;

            this.Set_Wheels(fr * speed, fl * speed, br * speed, bl * speed);
        }

        public void Stop_Wheels() {
            this.Set_Wheels(0, 0, 0, 0);
        }
    }



}
