package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MovementLib {
    public static class DriveWheels {
        public DcMotor Front_Right;
        public DcMotor Front_Left;
        public DcMotor Back_Right;
        public DcMotor Back_Left;

        public DriveWheels(HardwareMap hardwareMap) {
            this.Front_Right = hardwareMap.get(DcMotor.class, "frontright");
            this.Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
            this.Back_Right = hardwareMap.get(DcMotor.class, "backright");
            this.Back_Left = hardwareMap.get(DcMotor.class, "backleft");
        }
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
            double fl = Forward + RotateCC + Right;
            double fr = Forward - RotateCC - Right;
            double bl = Forward + RotateCC - Right;
            double br = Forward - RotateCC + Right;

            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(br), Math.abs(bl)))));

            fl /= max;
            fr /= max;
            br /= max;
            bl /= max;

            this.Set_Wheels(fr * speed, fl * speed, br * speed, bl * speed);
        }

        public void Reverse_These(boolean frontright, boolean frontleft, boolean backright, boolean backleft) {
            if(frontright) {
                this.Front_Right.setDirection(this.Front_Right.getDirection().inverted());
            }
            if(frontleft) {
                this.Front_Left.setDirection(this.Front_Left.getDirection().inverted());
            }
            if(backright) {
                this.Back_Right.setDirection(this.Back_Right.getDirection().inverted());
            }
            if(backleft) {
                this.Back_Left.setDirection(this.Back_Left.getDirection().inverted());
            }
        }
        public void Reverse_Left() {
            this.Reverse_These(false,true,false,true);
        }
        public void Reverse_Right() {
            this.Reverse_These(true,false,true,false);
        }

        public void Stop_Wheels() {
            this.Set_Wheels(0, 0, 0, 0);
        }
    }



}
