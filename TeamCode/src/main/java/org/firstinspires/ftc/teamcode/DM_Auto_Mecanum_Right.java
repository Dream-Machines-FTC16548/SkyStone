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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

@Autonomous(name="DM: Auto Mecanum Park Right", group="DM#16548")
//@Disabled
public class DM_Auto_Mecanum_Right extends DM_Auto_Mecanum_Base {

    @Override
    public void runOpMode() {
        // Vuforia and Tensorflow related initialization
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
//        robot.init(hardwareMap);

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Front grabber servos
        front_left_grab = hardwareMap.get(Servo.class, "fl_grab");
        front_right_grab = hardwareMap.get(Servo.class, "fr_grab");
        front_left_grab.setPosition(0.6);
        front_right_grab.setPosition(0.1);

        // Distance Sensor
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        // Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Color Sensor related initialization
        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        colorSensor.enableLed(true);
        colorSensor2 = hardwareMap.get(ColorSensor.class, "color_sensor2");
        colorSensor2.enableLed(true);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status",  "Encoder Reset Done");
        telemetry.update();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "IMU Calibrating");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "IMU Calibration Done");
        telemetry.update();

        resetAngle();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step 1: Move forward
        int target_leftPos = -1000;
        int target_rightPos = -1000;
        //moveFwdAndBack( DRIVE_SPEED, target_leftPos, target_rightPos, 800 );
        moveFwdAndBackForMilliseconds(0.25, 3000);

        sleep(500 );

        // Step 2: Move Sideway to Right
        target_leftPos += 2500;     // 1500
        target_rightPos -= 2500;    // -3500
        //moveSideway( DRIVE_SPEED, target_leftPos, target_rightPos );
        moveSidewayForMilliseconds(-0.25, 2000);

        sleep(500 );

        // Step 2.5: Move forward until certain range
        moveFwdUntilRange( 0.25, 3 );    // 1500 - x
        sleep(500 );                           // -3500 - x

        // Step 3: Put down front grabbers
        front_left_grab.setPosition(0.0);
        front_right_grab.setPosition(0.65);
        sleep(2000);

        // Step 4: Move backward
        target_leftPos += 2400;     // 3000 - x
        target_rightPos += 2400;    // - 2000 - x
        moveFwdAndBackForMilliseconds(-0.25, 4000);
//        moveFwdAndBack( -0.8, (int)(target_leftPos*0.9), (int)(target_rightPos*0.9), 100 );
//        moveFwdAndBack( -0.4, (int)(target_leftPos*0.2), (int)(target_rightPos*0.2), 100 );
        sleep(500 );

        // Step 5: Move front grabbers up
        front_left_grab.setPosition(0.6);
        front_right_grab.setPosition(0.1);
        sleep(500 );

        // Step 6: Move Sideway to Left
//        moveSideway( -DRIVE_SPEED, -400, 1100 );
        moveSidewayForMilliseconds(0.25, 2000);
        sleep(100 );
        moveSidewayUntilColorFound( 0.3, COLOR_BLUE, 15);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // S1: Forward 6 Inches with 4 Sec timeout
//        telemetry.addData("Status",  ">> S1 Started");
//        telemetry.update();
//        encoderDrive(DRIVE_SPEED_SLOW,  6,  6, 4.0);

        // S2: Turn right 90 degrees
//        telemetry.addData("Status",  ">> S2 Started");
//        telemetry.update();
//        encoderDrive(TURN_SPEED,   -12, -12, 4.0);
//        rotate(-90, TURN_SPEED);

        // S3: Reverse 6 Inches with 4 Sec timeout
//        telemetry.addData("Status",  ">> S3 Started");
//        telemetry.update();
//        encoderDrive(DRIVE_SPEED_SLOW, 6, 6, 4.0);

        // S4: Turn left 90 degrees
//        telemetry.addData("Status",  ">> S4 Started");
//        telemetry.update();
//        rotate(90, TURN_SPEED);

        // S5: Look for Skystone
/*
        telemetry.addData("Status",  ">> S5 Started");

        telemetry.update();

        boolean foundSkyStone = false;
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equalsIgnoreCase("SkyStone")) {
                            telemetry.addData("Label: ", "SkyStone Found!");
                            foundSkyStone = true;
                        }

 */
/*
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
*/
/*
                    }
                    telemetry.update();
                }
            }

            if (foundSkyStone)
                break;
        }
*/

        // S6: Play sound
        // create a sound parameter that holds the desired player parameters.

        // S7: Turn left 90 degrees
//        telemetry.addData("Status",  ">> S7 Started");
//        telemetry.update();
//        rotate(90, TURN_SPEED);

        // S8: Move forward 108 inches with 10 Sec timeout
//        telemetry.addData("Status",  ">> S8 Started");
//        telemetry.update();
//        encoderDrive(DRIVE_SPEED, 72, 72, 4.5);

        // S9: Turn right 90 degrees
//        telemetry.addData("Status",  ">> S9 Started");
//        telemetry.update();
//        rotate(-90, TURN_SPEED);

        // S10: Move forward 24 inches with 6 Sec timeout
//        telemetry.addData("Status",  ">> S10 Started");
//        telemetry.update();
//        encoderDrive(DRIVE_SPEED, 16, 16, 2.0);

//        sleep(1000);     // pause for servos to move
        // Set the panel back to the default color

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

//        telemetry.addData("Path", "Complete");
//        telemetry.update();

        // Vuforia and Tensorflow related clean-up
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
