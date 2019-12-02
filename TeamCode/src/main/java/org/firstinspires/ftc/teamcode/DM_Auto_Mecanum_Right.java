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

        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step 1: Move forward
        moveFwdAndBackForMilliseconds(0.5, 900);

        // Step 2: Move Sideway to Left
        moveSidewayForMilliseconds(0.3, 2500);

        // Step 2.5: Move forward until certain range
        moveFwdUntilRange( 0.25, 2.3, 2 );    // 1500 - x
                                                           // -3500 - x

        // Step 3: Put down front grabbers
        front_left_grab.setPosition(0.0);
        front_right_grab.setPosition(0.65);
        sleep(2000);

        // Step 4: Move backward
        moveFwdAndBackForMilliseconds(-0.5, 1500);
        moveFwdAndBackForMilliseconds(-0.25, 1300);
        sleep(500 );

        // Step 5: Move front grabbers up
        front_left_grab.setPosition(0.6);
        front_right_grab.setPosition(0.1);
        sleep(2000 );

        // Step 6: Move Sideway to Left
        moveSidewayForMilliseconds(-0.3, 3000);
        sleep(100 );
        moveSidewayUntilColorFound( -0.25, COLOR_BLUE, 100);

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
