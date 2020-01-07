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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="DM: Auto Skystone Base", group="DM#16548")
@Disabled
public abstract class DM_Auto_Skystone_Base extends LinearOpMode {

    /* Declare OpMode members. */
//    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    protected DcMotor frontLeft, frontRight, backLeft, backRight;
    protected Servo front_left_grab, front_right_grab;
    protected DistanceSensor sensorRange;
    ColorSensor colorSensor;    // Hardware Device Object
    ColorSensor colorSensor2;
    float hsvValues[] = {0F,0F,0F};
    float hsvValues2[] = {0F,0F,0F};
    final float values[] = hsvValues;
    final float values2[] = hsvValues2;
    int relativeLayoutId;
    View relativeLayout;

    protected ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 / 2 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 / 3 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;   // For figuring circumference - 100mm
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED_SLOW        = 0.4;
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.15;
    static final double     TARGET_SKYSTONE_LEFT_POS    = 370.0;
    static final double     TARGET_SKYSTONE_RIGHT_POS   = 900.0;
    static final boolean    fGyroAssisted           = true;
    boolean                 soundPlaying            = false;

    public final static int     COLOR_RED = 1;
    public final static int     COLOR_BLUE = 2;
    public final static int     COLOR_MIN = 50;
    public final static int     COLOR_MIN2 = 500;

    // Gyro related initialization
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    // Vuforia and Tensorflow related initialization
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";

    private static final String VUFORIA_KEY =
            "ATg5zNT/////AAABmQhNspqC40DCvUSavEYlElMBRSLlK9rhCxs+Jd4rqsraiBijuWYBeupEoJrKGOgaTaP2AuF8RIBHvXtJLaf6jEffF6Jfi0wxmwKfxCegMt4YezZ22wpiK2WfnvvjolMxcVFpKLo38wrA8n88Dy8G2Rg2HAu2HILsg+Sq6dfKpynpbQs8ycs46zHvZUWVp+BVdifSxoKC4RT9zwPtyykIUhiw2Nr1ueaHQKMYTda2EbhgZ/1LP4/fqSNHqZhcqbFTSL3Fcsup+a449TPBlERNWgJDoInJ4lT9iyopclF5tVKqS01xpbmEwAaDp/v5e/aV4HPupgGdRbQCVdIvHQv8XtS7VNT6+Y1wy9QX1MlonqGk";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    protected VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    protected TFObjectDetector tfod;

    protected  void initHardware()
    {
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
        if ( fGyroAssisted ) {
            while (!imu.isGyroCalibrated()) {
                sleep(50);
                idle();
            }

            resetAngle();
        }

        telemetry.addData("Status", "IMU Calibration Done");
        telemetry.update();

        telemetry.addData("Status", "Initialization Done");
        telemetry.update();
    }

    protected void moveSideway( double speed, int leftPos, int rightPos ) {

        // Right = +ve speed; Left = -ve speed
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setTargetPosition( leftPos );
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setTargetPosition( rightPos );
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy())) {

            // Use gyro to drive in a straight line.
            if ( fGyroAssisted )
                correction = checkDirection() * Math.abs(speed)*0.2;
            else
                correction = 0;

            frontLeft.setPower(speed + correction);
            frontRight.setPower(-speed - correction);
            backLeft.setPower(-speed + correction);
            backRight.setPower(speed - correction);

            // Display it for the driver.
            telemetry.addData("LF", frontLeft.getCurrentPosition());
            telemetry.addData("RF", frontRight.getCurrentPosition());
            telemetry.addData("LB", backLeft.getCurrentPosition());
            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }


    protected void moveSidewayUntilColorFound( double speed, int color, int timeouts ) {
        speed = - speed;
        // Right = +ve speed; Left = -ve speed
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime     runtime = new ElapsedTime();
        boolean colorFound = false;
        while (opModeIsActive() && !colorFound  && runtime.seconds()<timeouts) {
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            Color.RGBToHSV(colorSensor2.red() * 8, colorSensor2.green() * 8, colorSensor2.blue() * 8, hsvValues2);

            if ( color == COLOR_BLUE && ( hsvValues[0] > 150 || hsvValues2[0] > 150 ))
                colorFound = true;
            else if ( color == COLOR_RED && ( hsvValues[0] < 50 || hsvValues2[0] < 50 ))
                colorFound = true;

            // Use gyro to drive in a straight line.
            if (fGyroAssisted )
                correction = checkDirection() * Math.abs(speed) * 0.4;
            else
                correction = 0;

            frontLeft.setPower(speed + correction);
            frontRight.setPower(-speed - correction);
            backLeft.setPower(-speed + correction);
            backRight.setPower(speed - correction);

            // Display it for the driver.
            telemetry.addData("Correction", correction);
            telemetry.addData("LF", frontLeft.getPower());
            telemetry.addData("RF", frontRight.getPower());
            telemetry.addData("LB", backLeft.getPower());
            telemetry.addData("RB", backRight.getPower());
            telemetry.update();
        }

        sleep(200);
        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    private boolean objectWithinTargetHorizontalRange( double leftPos, double rightPos ) {
        if ( leftPos >= TARGET_SKYSTONE_LEFT_POS /* && rightPos <= TARGET_SKYSTONE_RIGHT_POS */ )
            return true;
        return false;
    }

    protected boolean SkyStoneFound(){
        boolean skyStoneFound = false;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if ( recognition.getLabel().equalsIgnoreCase( LABEL_SKYSTONE )) {
                        if ( objectWithinTargetHorizontalRange( recognition.getLeft(), recognition.getRight()))
                            skyStoneFound = true;

                        telemetry.addData( "Staus: ", String.format("Skystone Found"));
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f",
                                recognition.getLeft());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f",
                                recognition.getRight());
                        telemetry.update();
//                            sleep(1000);
                    }

                    if( skyStoneFound ){
                        break;
                    }
                }
            }
        }

        return skyStoneFound;
    }


    protected void moveSidewayUntilSkystoneFoundV2( double speed, int timeouts ) {
        speed = - speed;
        // Right = +ve speed; Left = -ve speed
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime     runtime = new ElapsedTime();
        boolean skyStoneFound = false;

        while (opModeIsActive() && !skyStoneFound  && runtime.seconds()<timeouts) {

            skyStoneFound = SkyStoneFound();

            if(skyStoneFound) {
                break;
            }
            // Use gyro to drive in a straight line.
            if (fGyroAssisted )
                correction = checkDirection() * Math.abs(speed) * 0.4;
            else
                correction = 0;

            frontLeft.setPower(speed + correction);
            frontRight.setPower(-speed - correction);
            backLeft.setPower(-speed + correction);
            backRight.setPower(speed - correction);

            // Display it for the driver.
            telemetry.addData("Correction", correction);
            telemetry.addData("LF", frontLeft.getPower());
            telemetry.addData("RF", frontRight.getPower());
            telemetry.addData("LB", backLeft.getPower());
            telemetry.addData("RB", backRight.getPower());
            telemetry.update();
        }

        //sleep(200);
        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }



    protected void moveSidewayUntilSkystoneFound( double speed, int timeouts ) {
        speed = - speed;
        // Right = +ve speed; Left = -ve speed
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime     runtime = new ElapsedTime();
        boolean skyStoneFound = false;
        while (opModeIsActive() && !skyStoneFound  && runtime.seconds()<timeouts) {

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if ( recognition.getLabel().equalsIgnoreCase( LABEL_SKYSTONE )) {
                            if ( objectWithinTargetHorizontalRange( recognition.getLeft(), recognition.getRight()))
                                skyStoneFound = true;
                            telemetry.addData( "Staus: ", String.format("Skystone Found"));
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f",
                                    recognition.getLeft());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f",
                                    recognition.getRight());
                            telemetry.update();
//                            sleep(1000);
                        }
                    }
                }
            }

            // Use gyro to drive in a straight line.
            if ( fGyroAssisted )
                correction = checkDirection() * Math.abs(speed)/2;
            else
                correction = 0;

            frontLeft.setPower(speed + correction);
            frontRight.setPower(-speed - correction);
            backLeft.setPower(-speed + correction);
            backRight.setPower(speed - correction);

            // Display it for the driver.
            telemetry.addData("LF", frontLeft.getCurrentPosition());
            telemetry.addData("RF", frontRight.getCurrentPosition());
            telemetry.addData("LB", backLeft.getCurrentPosition());
            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.update();
        }

        sleep(200);
        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    protected void moveFwdAndBack( double speed, int leftPos, int rightPos, int timeouts) {

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setTargetPosition( leftPos );
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setTargetPosition( rightPos );
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

        ElapsedTime     runtime = new ElapsedTime();

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy()) && runtime.seconds()< timeouts) {

            // Use gyro to drive in a straight line.
            if ( fGyroAssisted )
                correction = checkDirection() * Math.abs(speed);
            else
                correction = 0;

            telemetry.addData("In move: ", 6);
            telemetry.update();

            frontLeft.setPower(speed + correction);
            frontRight.setPower(speed - correction);
            backLeft.setPower(-speed - correction);
            backRight.setPower(-speed + correction);

            // Display it for the driver.
//            telemetry.addData("LF", frontLeft.getCurrentPosition());
//            telemetry.addData("RF", frontRight.getCurrentPosition());
//            telemetry.addData("LB", backLeft.getCurrentPosition());
//            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.addData("LF", frontLeft.getPower());
            telemetry.addData("RF", frontRight.getPower());
            telemetry.addData("LB", backLeft.getPower());
            telemetry.addData("RB", backRight.getPower());
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    protected void moveFwdUntilRange( double speed, double distanceInch, int timeouts ) {
        speed = -speed;

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        double distance = sensorRange.getDistance(DistanceUnit.INCH);
        ElapsedTime     runtime = new ElapsedTime();

        while (opModeIsActive() && distance > distanceInch && runtime.seconds()< timeouts) {

            // Use gyro to drive in a straight line.
            if (fGyroAssisted )
                correction = checkDirection() * Math.abs(speed);
            else
                correction = 0;

            frontLeft.setPower(speed + correction);
            frontRight.setPower(speed - correction);
            backLeft.setPower(speed + correction);
            backRight.setPower(speed - correction);

            distance = sensorRange.getDistance(DistanceUnit.INCH);

            // Display it for the driver.
//            telemetry.addData("LF", frontLeft.getCurrentPosition());
//            telemetry.addData("RF", frontRight.getCurrentPosition());
//            telemetry.addData("LB", backLeft.getCurrentPosition());
//            telemetry.addData("RB", backRight.getCurrentPosition());
//            telemetry.addData("Correction", correction);
            telemetry.addData("Distance=", distance);
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    protected void moveFwdAndBackForMilliseconds( double speed, double milliseconds ) {
        speed = -speed;

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        ElapsedTime     runtime = new ElapsedTime();

        while (opModeIsActive() && runtime.milliseconds() < milliseconds ) {

            // Use gyro to drive in a straight line.
            if ( fGyroAssisted )
                correction = checkDirection() * Math.abs(speed);
            else
                correction = 0;

            frontLeft.setPower(speed + correction);
            frontRight.setPower(speed - correction);
            backLeft.setPower(speed + correction);
            backRight.setPower(speed - correction);

            // Display it for the driver.
            telemetry.addData("LF", frontLeft.getCurrentPosition());
            telemetry.addData("RF", frontRight.getCurrentPosition());
            telemetry.addData("LB", backLeft.getCurrentPosition());
            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.addData("Speed", speed);
            telemetry.addData("Second", milliseconds);
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    protected void moveSidewayForMilliseconds( double speed, double milliseconds ) {
        speed = -speed;

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        ElapsedTime     runtime = new ElapsedTime();

        while (opModeIsActive() && runtime.milliseconds() < milliseconds ) {

            // Use gyro to drive in a straight line.
            if ( fGyroAssisted )
                correction = checkDirection() * Math.abs(speed) * 0.4;
            else
                correction = 0;

            frontLeft.setPower(speed + correction);
            frontRight.setPower(-speed - correction);
            backLeft.setPower(-speed + correction);
            backRight.setPower(speed - correction);

            // Display it for the driver.
            telemetry.addData("LF", frontLeft.getCurrentPosition());
            telemetry.addData("RF", frontRight.getCurrentPosition());
            telemetry.addData("LB", backLeft.getCurrentPosition());
            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    protected void moveFwdAndBackForDistance( double speed, double inches, double timeoutInMilliseconds ) {
        speed = -speed;

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        ElapsedTime     runtime = new ElapsedTime();
        double startPostion = frontLeft.getCurrentPosition();
        //COUNTS_PER_INCH
        while (opModeIsActive() && runtime.milliseconds() < timeoutInMilliseconds ) {

            double leftDiff = startPostion + inches*COUNTS_PER_INCH + frontLeft.getCurrentPosition();
            double slowDownFactor = 1.0;
            if (leftDiff < 2 * COUNTS_PER_INCH) {
                slowDownFactor = (double) leftDiff / (2 * COUNTS_PER_INCH);
            }

            if( Math.abs(frontLeft.getCurrentPosition()-startPostion)<50)
                break;

            // Use gyro to drive in a straight line.
            if ( fGyroAssisted )
                correction = checkDirection() * Math.abs(speed);
            else
                correction = 0;

            frontLeft.setPower((speed + correction)* slowDownFactor);
            frontRight.setPower((speed - correction)* slowDownFactor);
            backLeft.setPower((speed + correction)* slowDownFactor);
            backRight.setPower((speed - correction)* slowDownFactor);

            // Display it for the driver.
            telemetry.addData("LF", frontLeft.getCurrentPosition());
            telemetry.addData("RF", frontRight.getCurrentPosition());
            telemetry.addData("LB", backLeft.getCurrentPosition());
            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    protected void moveSidewayForDistance( double speed, double inches, double timeoutInMilliseconds ) {
        speed = -speed;

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        ElapsedTime     runtime = new ElapsedTime();
        double startPostion = backLeft.getCurrentPosition();

        while (opModeIsActive() && runtime.milliseconds() < timeoutInMilliseconds ) {

            double leftDiff = startPostion + inches*COUNTS_PER_INCH + backLeft.getCurrentPosition();
            double slowDownFactor = 1.0;
            if (leftDiff < 2 * COUNTS_PER_INCH) {
                slowDownFactor = (double) leftDiff / (2 * COUNTS_PER_INCH);
            }

            if( Math.abs(frontLeft.getCurrentPosition()-startPostion)<50)
                break;

            // Use gyro to drive in a straight line.
            if ( fGyroAssisted )
                correction = checkDirection() * Math.abs(speed)/2;
            else
                correction = 0;
            frontLeft.setPower(speed + correction);
            frontRight.setPower(-speed - correction);
            backLeft.setPower(-speed + correction);
            backRight.setPower(speed - correction);

            // Display it for the driver.
            telemetry.addData("LF", frontLeft.getCurrentPosition());
            telemetry.addData("RF", frontRight.getCurrentPosition());
            telemetry.addData("LB", backLeft.getCurrentPosition());
            telemetry.addData("RB", backRight.getCurrentPosition());
            telemetry.addData("Correction", correction);
            telemetry.update();
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }



    // Gyro related routines
    /**
     * Resets the cumulative angle tracking to zero.
     */
    protected void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    protected double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    protected void rotate(int degrees, double power)
    {
        double  turn, frontleftpower, frontrightpower, backleftpower, backrightpower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {
            // turn right.
            turn = -0.5;
        } else if (degrees > 0) {
            // turn left.
            turn = 0.5;
        } else
            return;

        // set power to rotate.
        frontleftpower = Range.clip(power  - turn, -1, 1);
        frontrightpower = Range.clip(power  + turn, -1, 1);
        backleftpower = Range.clip(power  - turn, -1, 1);
        backrightpower = Range.clip(power + turn, -1, 1);
        frontLeft.setPower(frontleftpower);
        frontRight.setPower(frontrightpower);
        backLeft.setPower(backleftpower);
        backRight.setPower(backrightpower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            double diff = getAngle() - (double) degrees;
            while (opModeIsActive() && diff > 0) {
                if ( diff < 20.0 ) {
                    frontLeft.setPower(frontleftpower* diff / 20);
                    frontRight.setPower(frontrightpower* diff / 20);
                    backLeft.setPower(backleftpower* diff / 20);
                    backRight.setPower(backrightpower* diff / 20);
                }
                diff = getAngle() - degrees;
            }
        } else {
            // left turn.
            double diff = (double) degrees - getAngle();
            while (opModeIsActive() && diff > 0) {
                if ( diff < 20.0 ) {
                    frontLeft.setPower(frontleftpower* diff / 20);
                    frontRight.setPower(frontrightpower* diff / 20);
                    backLeft.setPower(backleftpower* diff / 20);
                    backRight.setPower(backrightpower* diff / 20);
                }
                diff = degrees - getAngle();
            }
        }

        // turn the motors off.
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    // Vuforia and Tensorflow related functions
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "WebcamSkystone");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
    }

}
