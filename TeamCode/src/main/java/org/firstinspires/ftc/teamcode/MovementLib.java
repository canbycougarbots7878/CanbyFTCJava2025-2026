package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("unused")
public class MovementLib {
    public static class Vector2 {
        double x;
        double y;
        public Vector2(double x, double y) {
            this.x = x;
            this.y = y;
        }
        public double magnitude() {
            return Math.sqrt(x*x+y*y);
        }
    }
    public static class Robot {
        public DcMotor Front_Right;
        public DcMotor Front_Left;
        public DcMotor Back_Right;
        public DcMotor Back_Left;
        public SparkFunOTOS otos;

        private Boolean OTOS_ENABLED = false;

        public Robot(HardwareMap hardwareMap) {
            this.Front_Right = hardwareMap.get(DcMotor.class, "frontright");
            this.Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
            this.Back_Right = hardwareMap.get(DcMotor.class, "backright");
            this.Back_Left = hardwareMap.get(DcMotor.class, "backleft");
        }
        public Robot(HardwareMap hardwareMap, boolean findOtos) {
            this.Front_Right = hardwareMap.get(DcMotor.class, "frontright");
            this.Front_Left = hardwareMap.get(DcMotor.class, "frontleft");
            this.Back_Right = hardwareMap.get(DcMotor.class, "backright");
            this.Back_Left = hardwareMap.get(DcMotor.class, "backleft");

            if(findOtos) {
                this.otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
                this.otos.setLinearUnit(DistanceUnit.METER);
                this.otos.setAngularUnit(AngleUnit.DEGREES);
                this.otos.calibrateImu();
                this.otos.resetTracking();
            }

            this.OTOS_ENABLED = findOtos;
        }
        public Robot(DcMotor Front_Right, DcMotor Front_Left, DcMotor Back_Right, DcMotor Back_Left) {
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
            double fl = Forward + Right + RotateCC;
            double fr = Forward - Right - RotateCC;
            double bl = Forward - Right + RotateCC;
            double br = Forward + Right - RotateCC;

            // normalize so no value exceeds 1
            double max = Math.max(1.0, Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;

            this.Set_Wheels(fr * speed, fl * speed, br * speed, bl * speed);
        }
        public void Omni_Move(Vector2 vec2, double speed) {
            Omni_Move(vec2.y,vec2.x,0.0,speed);

        } public void Omni_Move(Vector2 vec2) { Omni_Move(vec2, 1.0); }

        public void Omni_Move(double Forward, double Right, double RotateCC) {
            Omni_Move(Forward, Right, RotateCC, 1.0); /* If no speed provided, assume full speed (1.0) */
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

        public void Omni_Move_To_Target(SparkFunOTOS.Pose2D target) {
            if(!OTOS_ENABLED) return;
            SparkFunOTOS.Pose2D pose = otos.getPosition();
            double dx = 2 * (target.x - pose.x);
            double dy = 2 * (target.y - pose.y);
            double dh = (target.h - pose.h) / 180.0;
            Omni_Move(dx, dy, dh);
        }
        public double Distance_To(SparkFunOTOS.Pose2D target) {
            SparkFunOTOS.Pose2D pos = otos.getPosition();
            double dx = 2 * (target.x - pos.x);
            double dy = 2 * (target.y - pos.y);
            double dh = (target.h - pos.h) / 180.0;
            return Math.sqrt(dx*dx+dy*dy+dh*dh);
        }
    }
}
