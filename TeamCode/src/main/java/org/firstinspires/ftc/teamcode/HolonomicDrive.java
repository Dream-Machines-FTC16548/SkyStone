/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@TeleOp(name="HolonomicDrive", group="Pushbot@Chen")
//@Autonomous(name="Prototype: Auto Mecanum Park Left", group="Prototype")
@Disabled
public class HolonomicDrive extends LinearOpMode {
        float rotate_angle = 0;
        double reset_angle = 0;

        private DcMotor front_left_wheel = null;
        private DcMotor back_left_wheel = null;
        private DcMotor back_right_wheel = null;
        private DcMotor front_right_wheel = null;

        BNO055IMU imu;
        @Override
        public void runOpMode() {
            front_left_wheel = hardwareMap.dcMotor.get("front_left");
            back_left_wheel = hardwareMap.dcMotor.get("back_left");
            back_right_wheel = hardwareMap.dcMotor.get("back_right");
            front_right_wheel = hardwareMap.dcMotor.get("front_right");

            front_left_wheel.setDirection(DcMotor.Direction.REVERSE);
            back_left_wheel.setDirection(DcMotor.Direction.REVERSE);
            front_right_wheel.setDirection(DcMotor.Direction.FORWARD);
            back_right_wheel.setDirection(DcMotor.Direction.FORWARD);

            front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            imu = hardwareMap.get(BNO055IMU.class, "imu");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode                = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = false;
            imu.initialize(parameters);

            while(!opModeIsActive()){}

            while(opModeIsActive()){
                drive();
                resetAngle();
                //driveSimple();
                telemetry.update();
            }
        }
        public void driveSimple(){
            double power = .5;
            if(gamepad1.dpad_up){ //Forward
                front_left_wheel.setPower(-power);
                back_left_wheel.setPower(-power);
                back_right_wheel.setPower(-power);
                front_right_wheel.setPower(-power);
            }
            else if(gamepad1.dpad_left){ //Left
                front_left_wheel.setPower(power);
                back_left_wheel.setPower(-power);
                back_right_wheel.setPower(power);
                front_right_wheel.setPower(-power);
            }
            else if(gamepad1.dpad_down){ //Back
                front_left_wheel.setPower(power);
                back_left_wheel.setPower(power);
                back_right_wheel.setPower(power);
                front_right_wheel.setPower(power);
            }
            else if(gamepad1.dpad_right){ //Right
                front_left_wheel.setPower(-power);
                back_left_wheel.setPower(power);
                back_right_wheel.setPower(-power);
                front_right_wheel.setPower(power);
            }
            else if(Math.abs(gamepad1.right_stick_x) > 0){ //Rotation
                front_left_wheel.setPower(-gamepad1.right_stick_x);
                back_left_wheel.setPower(-gamepad1.right_stick_x);
                back_right_wheel.setPower(gamepad1.right_stick_x);
                front_right_wheel.setPower(gamepad1.right_stick_x);
            }
            else{
                front_left_wheel.setPower(0);
                back_left_wheel.setPower(0);
                back_right_wheel.setPower(0);
                front_right_wheel.setPower(0);
            }
        }
        public void drive() {
            double Protate = gamepad1.right_stick_x/4;
            double stick_x = gamepad1.left_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2); //Accounts for Protate when limiting magnitude to be less than 1
            double stick_y = gamepad1.left_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2);
            double theta = 0;
            double Px = 0;
            double Py = 0;

            double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians
            if (gyroAngle <= 0) {
                gyroAngle = gyroAngle + (Math.PI / 2);
            } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
                gyroAngle = gyroAngle + (Math.PI / 2);
            } else if (Math.PI / 2 <= gyroAngle) {
                gyroAngle = gyroAngle - (3 * Math.PI / 2);
            }
            gyroAngle = -1 * gyroAngle;

            if(gamepad1.right_bumper){ //Disables gyro, sets to -Math.PI/2 so front is defined correctly.
                gyroAngle = -Math.PI/2;
            }

            //Linear directions in case you want to do straight lines.
            if(gamepad1.dpad_right){
                stick_x = 0.5;
            }
            else if(gamepad1.dpad_left){
                stick_x = -0.5;
            }
            if(gamepad1.dpad_up){
                stick_y = -0.5;
            }
            else if(gamepad1.dpad_down){
                stick_y = 0.5;
            }


            //MOVEMENT
            theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
            Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
            Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));

            telemetry.addData("Stick_X", stick_x);
            telemetry.addData("Stick_Y", stick_y);
            telemetry.addData("Magnitude",  Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
            telemetry.addData("Front Left", Py - Protate);
            telemetry.addData("Back Left", Px - Protate);
            telemetry.addData("Back Right", Py + Protate);
            telemetry.addData("Front Right", Px + Protate);

            front_left_wheel.setPower(Py - Protate);
            back_left_wheel.setPower(Px - Protate);
            back_right_wheel.setPower(Py + Protate);
            front_right_wheel.setPower(Px + Protate);
        }
        public void resetAngle(){
            if(gamepad1.a){
                reset_angle = getHeading() + reset_angle;
            }
        }
        public double getHeading(){
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle;
            if(heading < -180) {
                heading = heading + 360;
            }
            else if(heading > 180){
                heading = heading - 360;
            }
            heading = heading - reset_angle;
            return heading;
        }
    }

