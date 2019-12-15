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
        moveFwdAndBackForMilliseconds(0.5, 900);

        // Step 2: Move forward until certain range
        moveFwdUntilRange( 0.25, 2.3, 2 );    // 1500 - x

        // Step 3: Move Sideway to Left until Skystone is found
        moveSidewayUntilSkystoneFound( -0.25, 5000 );

        // Step 4: Put down front grabbers
        front_left_grab.setPosition(0.0);
        front_right_grab.setPosition(0.65);
        sleep(2000);

        // Step 5: Move backward a bit
        moveFwdAndBackForMilliseconds(-0.25, 1300);
        sleep(500 );

        // Step 6: Move Sideway to Right
        moveSidewayForMilliseconds(0.5, 3000);
        sleep(100 );

        // Step 7: Move front grabbers up
        front_left_grab.setPosition(0.6);
        front_right_grab.setPosition(0.1);
        sleep(2000 );

        // Step 8: Move Sideway to Left to park
//        moveSidewayForMilliseconds(-0.3, 3000);
//        sleep(100 );
        moveSidewayUntilColorFound( -0.25, COLOR_RED, 100);

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
