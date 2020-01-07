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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="DM: Auto Skystone Park Right", group="DM#16548")
//@Disabled
public class DM_Auto_Skystone_Right extends DM_Auto_Skystone_Base {

    @Override
    public void runOpMode() {

        initHardware();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step 1: Move forward
        telemetry.addData("Status: ", 1);
        telemetry.update();
        moveFwdAndBackForMilliseconds(0.4, 860);

            // Step 2: Move Sideway to Left until Skystone is found
        telemetry.addData("Status: ", 2);
        telemetry.update();
        moveSidewayUntilSkystoneFoundV2( 0.16, 8000 );
//        moveSidewayForMilliseconds(0.15, 200);

        // Step 3: Move forward until certain range
        telemetry.addData("Status: ", 3);
        telemetry.update();
        moveFwdUntilRange( 0.2, 1.2, 6 );

        // Step 4: Put down front grabbers
        telemetry.addData("Status: ", 4);
        telemetry.update();
//        front_left_grab.setPosition(0.0);
        front_right_grab.setPosition(0.65);
        sleep(500);

        // Step 5: Move backward a bit
        telemetry.addData("Status: ", 5);
        telemetry.update();
        moveFwdAndBackForMilliseconds(-0.25, 1800);
        sleep(500 );

        // raise up the grabber a little to reduce friction of skystone
        front_right_grab.setPosition(0.52);
        sleep(500);

        // Step 6: Move Sideway to Right to line
        telemetry.addData("Status: ", 6);
        telemetry.update();
        moveSidewayUntilColorFound(-0.35, COLOR_RED, 20 );
        sleep(100 );

        // Step 7 Move Sideway to Right
        telemetry.addData("Status: ", 7);
        telemetry.update();
        moveSidewayForMilliseconds( -0.4, 500 );

        // Step 8: Move front grabbers up
        telemetry.addData("Status: ", 8);
        telemetry.update();
//        front_left_grab.setPosition(0.6);
        front_right_grab.setPosition(0.1);
        sleep(500 );

        // Step 9: Move Sideway to Left to park
        telemetry.addData("Status: ", 9);
        telemetry.update();
//        moveSidewayForMilliseconds(-0.3, 3000);
//        sleep(100 );
        moveSidewayUntilColorFound( 0.35, COLOR_RED, 5);

        // Step 10: Move forward a bit
        telemetry.addData("Status: ", 5);
        telemetry.update();
        moveFwdAndBackForMilliseconds(0.25, 900);
        sleep(500 );


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
