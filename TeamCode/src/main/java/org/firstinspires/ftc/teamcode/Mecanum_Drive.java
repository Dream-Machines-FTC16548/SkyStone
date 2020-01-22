package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class Mecanum_Drive extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor linear_motor;
    private DcMotor lowerArm;
    private Servo leftClaw, rightClaw, linear, grabber, rotate, front_left_grab, front_right_grab, side_grab, capstonePitcher;
    private DigitalChannel lower_sensor, arm_limit;
    private float fwd, side, turn, power, lower_arm_stick, grabber_stick;
    private double clawOffset = 0.5;
    private double rotation_offset = 0.0;
    private double linearOffset = 0.0;
    private double grabberOffset = 0.0;
    private boolean btn_y, btn_a, left_bumper_1, right_bumper_1, left_bumper_2, right_bumper_2, dpad_left, dpad_right, btn_x1, btn_b1, btn_x2, btn_b2;
    private boolean clawExpanded = false;
    private boolean clawOpen = true;
    private boolean turning = false;
    private boolean gyro_assist = false;
    private boolean front_grabber_up = true;
    private boolean side_grabber_up = true;
    private double target_grabber = 0.0;
    private double constant_F = -1.0;


    // Gyro related initialization
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, correction;

    // Variables
    public final static double deadzone = 0.2;              // Deadzone for bot movement
    public final static double deadzone_hi = 0.9;           // Deadzone for straight movement
    public final static double turn_speed_max = 0.15;       // Max turn speed
    public final static double rotation_speed = 0.01;       // Claw rotation movement rate
    public final static double rotation_offset_max = 0.1;   // Max offset allowed to move rotation servo
    public final static double rotation_initial = 0.5;       // Claw rotation movement rate
    public final static double claw_speed = 0.01;           // Claw movement rate
    public final static double claw_left_initial = 0.0;    // Left claw initial position
    public final static double claw_right_initial = 0.0;   // Right claw initial position
    public final static double claw_mid_position = 0.5;    // Claw mid point position
    public final static double claw_move_span = 0.8;       // Claw movement span
    public final static double claw_grip_offset = 0.01;     // Grip offset for claws
    public final static double claw_release_offset = -0.01; // Release offset for claws
    public final static double linear_speed = 0.001;        // Linear servo rotation rate
    public final static double linear_lower = 0.28;         // Linear servo lower limit
    public final static double linear_upper = 0.615;        // Linear servo upper limit
    public final static double linear_initial = linear_lower;
    public final static double linear_offset_max = 0.002;    // Max offset allowed to move linear servo
    public final static double linear_motor_up_power = 0.8;     // Linear Motor up power
    public final static double linear_motor_down_power = -0.8;  // Linear Motor down power
    public final static int linear_motor_up_step = 50;      // Linear Motor position up
    public final static int linear_motor_down_step = 50;    // Linear Motor position down
    public final static int linear_motor_initial = 0;        // Linear Motor initial position
    public final static int linear_motor_end = 3760;        // Linear Motor end position
    public final static int linear_motor_low_protection = 300;
    public final static double grabber_speed = 0.005;       // Grabber servo rotation rate
    public final static double arm_up_power = 1.0;          // Arm up movement rate
    public final static double arm_down_power = -1.0;       // Arm down movement rate
    public final static int arm_up_step = 30;               // Arm up movement position step
    public final static int arm_down_step = 30;             // Arm up movement position step
    public final static int arm_low_protection = 2000;
    public final static double constantA = 8;
    public final static double constantB = 25;

    @Override
    public void runOpMode() {
        // Gyro initialization and calibration
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Status", "1");
        telemetry.update();

        // Reset angle
        resetAngle();

        telemetry.addData("Status", "2");
        telemetry.update();

        // Mecanum wheels
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "3");
        telemetry.update();

        // Arm motor
        lowerArm = hardwareMap.get(DcMotor.class, "lower_arm");
        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Linear motor
        linear_motor = hardwareMap.get(DcMotor.class, "linear");
        linear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_motor.setDirection(DcMotor.Direction.REVERSE);

        // Claw servos
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
//        linear = hardwareMap.get(Servo.class, "linear");
        grabber = hardwareMap.get(Servo.class,"grabber");
//        rotate = hardwareMap.get(Servo.class, "rotate");

        telemetry.addData("Status", "4");
        telemetry.update();

        // Front and side grabber servos
        front_left_grab = hardwareMap.get(Servo.class, "fl_grab");
        front_right_grab = hardwareMap.get(Servo.class, "fr_grab");
//        side_grab = hardwareMap.get(Servo.class, "side_grab");

        telemetry.addData("Status", "5");
        telemetry.update();

        capstonePitcher = hardwareMap.get(Servo.class, "capstone");

        // Sensor for lower arm
        lower_sensor = hardwareMap.get(DigitalChannel.class, "arm_lower_sensor");
        lower_sensor.setMode(DigitalChannel.Mode.INPUT);

        arm_limit = hardwareMap.get(DigitalChannel.class, "arm_limit");
        arm_limit.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "6");
        telemetry.update();

        // Setting initial positions for claw, grabber, linear servo and arm
        // Skipping initiation at the beginning
        initializePositions();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getGamePadValues();

            // Bot movement with Mecanum wheels
            power = fwd; //this can be tweaked for exponential power increase

            if ( turn != 0 ) {
                // We are turing
                turning = true;
                frontLeft.setPower(Range.clip(power  - turn, -1, 1));
                frontRight.setPower(Range.clip(power  + turn, -1, 1));
                backLeft.setPower(Range.clip(power  - turn, -1, 1));
                backRight.setPower(Range.clip(power + turn, -1, 1));
            } else {
                // Just finished turning and let it settled down
                if ( turning ) {
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    sleep(500);

                    resetAngle();
                    turning = false;
                }

                // Moving in straight line (forward, backward or strafe) without turning; Add gyro assistance
                if (gyro_assist)
                    correction = checkDirection();
                else
                    correction = 0;

                frontLeft.setPower(Range.clip(power + side + correction, -1, 1));
                frontRight.setPower(Range.clip(power - side - correction, -1, 1));
                backLeft.setPower(Range.clip(power - side + correction, -1, 1));
                backRight.setPower(Range.clip(power + side - correction, -1, 1));
            }


            // Arm movements
            if (opModeIsActive() && lower_arm_stick < 0 && lowerArm.getCurrentPosition() < 3000 ) {
                if ( lowerArm.getCurrentPosition() > arm_low_protection && linear_motor.getCurrentPosition() < linear_motor_low_protection ) {
                    // do nothing
                } else {
                    lowerArm.setTargetPosition(lowerArm.getCurrentPosition() + arm_up_step);
                    lowerArm.setPower(arm_up_power);
                    constant_F = -1.0;
                }
            } else if (opModeIsActive() && lower_arm_stick > 0 && lower_sensor.getState()) {
                lowerArm.setTargetPosition(lowerArm.getCurrentPosition() - arm_down_step);
                lowerArm.setPower(arm_down_power);
                constant_F = -1.0;
            } else
                lowerArm.setPower(0.0);


            // Linear motor movements
            if (opModeIsActive() && btn_y && linear_motor.getCurrentPosition() < linear_motor_end ) {
                int linear_original_pos = linear_motor.getCurrentPosition();
                int linear_target_pos = linear_motor.getCurrentPosition() + linear_motor_up_step;
                linear_motor.setTargetPosition(linear_target_pos);
                linear_motor.setPower(linear_motor_up_power);
                if ( constant_F < 0 ) {
                    // Calculate constant F to keep
                    double c = 19 + ( linear_original_pos * 7 / linear_motor_end );
                    double L = 40 + ( lowerArm.getCurrentPosition() / 100 );
                    constant_F = L * ( Math.pow(constantA,2.0) + Math.pow(constantB,2.0) - Math.pow(c,2.0) ) / ( 2 * constantA * constantB );
                } else {
                    // Calculate target position of lower arm position (L)
//                    telemetry.addData("OLD:", lowerArm.getCurrentPosition());
                    double c = 19 + ( linear_target_pos * 7 / linear_motor_end );
                    double L = constant_F * 2 * constantA * constantB / ( Math.pow(constantA,2.0) + Math.pow(constantB,2.0) - Math.pow(c,2.0) );
                    double targetPos = (L - 40) * 100;
                    Double dbTargetPost = new Double( targetPos );
                    lowerArm.setTargetPosition(dbTargetPost.intValue());
                    lowerArm.setPower(arm_up_power);

/*
                    telemetry.addData("F:", constant_F );
                    telemetry.addData("L:", L );
                    telemetry.addData("NEW:", dbTargetPost );
                    telemetry.update();

 */
                }
            } else if (opModeIsActive() && btn_a && arm_limit.getState()) {
                int linear_original_pos = linear_motor.getCurrentPosition();
                int linear_target_pos = linear_motor.getCurrentPosition() - linear_motor_down_step;
                if ( lowerArm.getCurrentPosition() > arm_low_protection && linear_original_pos < linear_motor_low_protection ) {
                    // Do nothing
                } else {
                    linear_motor.setTargetPosition(linear_target_pos);
                    linear_motor.setPower(linear_motor_down_power);
                    if (constant_F < 0) {
                        // Calculate constant F to keep
                        double c = 19 + (linear_original_pos * 7 / linear_motor_end);
                        double L = 40 + (lowerArm.getCurrentPosition() / 100);
                        constant_F = L * (Math.pow(constantA, 2.0) + Math.pow(constantB, 2.0) - Math.pow(c, 2.0)) / (2 * constantA * constantB);
                    } else {
//                        telemetry.addData("OLD:", lowerArm.getCurrentPosition());
                        // Calculate target position of lower arm position (L)
                        double c = 19 + (linear_target_pos * 7 / linear_motor_end);
                        double L = constant_F * 2 * constantA * constantB / (Math.pow(constantA, 2.0) + Math.pow(constantB, 2.0) - Math.pow(c, 2.0));
                        double targetPos = (L - 40) * 100;
                        Double dbTargetPost = new Double(targetPos);
                        lowerArm.setTargetPosition(dbTargetPost.intValue());
                        lowerArm.setPower(arm_down_power);
                    }
                }
            } else
                linear_motor.setPower(0.0);

            // Use gamepad left & right Bumpers to open and close the claw
            if (left_bumper_2) {
                clawOffset += claw_speed;
//                clawOffset = claw_grip_offset;
                clawOpen = true;
                clawExpanded = true;
            } else if (right_bumper_2) {
                clawOffset -= claw_speed;
//                clawOffset = claw_release_offset;
                clawOpen = false;
                clawExpanded = true;
            }

            // Move both claw servos to new position.  Assume servos are mirror image of each other.
            if ( clawExpanded ) {
//                clawOffset = Range.clip(clawOffset, -claw_move_span, claw_move_span);
//                leftClaw.setPosition(claw_mid_position + clawOffset);
//                rightClaw.setPosition(claw_mid_position - clawOffset);
                if (clawOpen) {
                    leftClaw.setPosition(0.39);
                    rightClaw.setPosition(0.59);
                } else {
                    leftClaw.setPosition(0.31);
                    rightClaw.setPosition(0.75);
                }
            }


            // User gamepad left bumper to move up and down the front grabber
            if (left_bumper_1) {
                front_grabber_up = !front_grabber_up;
                sleep(300);
            }

            if ( front_grabber_up ) {
                front_left_grab.setPosition(0.6);
                front_right_grab.setPosition(0.1);
            } else {
                front_left_grab.setPosition(0.0);
                front_right_grab.setPosition(0.65);
            }

            if( gamepad1.dpad_down )
            {
                capstonePitcher.setPosition(0);
            }else if(gamepad1.dpad_up )
            {
                capstonePitcher.setPosition(0.35);
            }

            if( gamepad1.y && gamepad1.a)
            {
                //moveToPickPosition();
                //commented due to problems
            }

            // User gamepad right bumper to move up and down the side grabber
//            if (right_bumper_1) {
//                side_grabber_up = !side_grabber_up;
//                sleep(300);
//            }

//            if ( side_grabber_up )
//                side_grab.setPosition(0.0);
//            else
//                side_grab.setPosition(1.0);


            // User DPad left and right to rotate claw
//            if (dpad_left)
//                rotation_offset -= rotation_speed;
//            else if (dpad_right)
//                rotation_offset += rotation_speed;

//            rotation_offset = Range.clip( rotation_offset, -0.5, 0.5 );
//            rotate.setPosition( rotation_initial + rotation_offset );


            // Use gamepad Y & right A buttons to rotate the linear servo
/*
            if (btn_y)
                linearOffset += linear_speed;
            else if (btn_a)
                linearOffset -= linear_speed;

            // Move linear servo to the desired position.
            linearOffset = Range.clip(linearOffset, -linear_offset_max, linear_offset_max);       // At most move certain limit amount for linear servo

            // Only set position when linear action buttons are not pressed OR linearOffset is at max allowed value
            if (( !btn_y && !btn_a ) || Math.abs(linearOffset) == linear_offset_max ) {
                double target = linear.getPosition() + linearOffset;
                if (target < linear_lower)
                    target = linear_lower;
                if (target > linear_upper)
                    target = linear_upper;
                linear.setPosition(target);

                if ( grabber.getPosition() > 0.5 ) {
                    target_grabber = 0.765 + (target - linear_lower) / (linear_upper - linear_lower) * (0.905 - 0.765);
                    grabberOffset = target_grabber;
                }

                linearOffset = 0.0;
            }
*/


            if ( grabber.getPosition() > 0.5 ) {
                target_grabber = 0.769 + (double) ((linear_motor.getCurrentPosition() - linear_motor_low_protection)) / (double) ( linear_motor_end - linear_motor_low_protection ) * (0.975 - 0.769);
                grabberOffset = target_grabber;
            }

            // Grabber movement
            if ( grabber_stick < 0 )
                grabberOffset += grabber_speed;
            else if ( grabber_stick > 0 )
                grabberOffset -= grabber_speed;

            // Move grabber servo to the desired position.
            grabberOffset = Range.clip(grabberOffset, 0.0, 1.0);
            grabber.setPosition(grabberOffset);


            // Check gyro support toggle
            if ( btn_x1 && btn_b1 ) {
                gyro_assist = !gyro_assist;
                resetAngle();
            }

            // Check request for initialize positions
            if ( btn_x2 && btn_b2 )
                initializePositions();

            // Output Telemetry information
//            telemetry.addData("FWD=", fwd);
//            telemetry.addData("SIDE=", side);
//            telemetry.addData("TURN=", turn);
            telemetry.addData("RCLAW=", rightClaw.getPosition());
            telemetry.addData("LCLAW", leftClaw.getPosition());
            telemetry.addData("GRAB=", grabber.getPosition());
            telemetry.addData("TGRAB=", target_grabber );

//            telemetry.addData("Linear= ", linear.getPosition());
            telemetry.addData("Linear= ", linear_motor.getCurrentPosition());
//            telemetry.addData("LFront= ", front_left_grab.getPosition());
//            telemetry.addData("LRront= ", front_right_grab.getPosition());
//            telemetry.addData("ARMSTK=", lower_arm_stick);
            telemetry.addData("Arm= ", lowerArm.getCurrentPosition());
            telemetry.addData("Gyro= ", gyro_assist);
//            telemetry.addData("LF", frontLeft.getCurrentPosition());
//            telemetry.addData("RF", frontRight.getCurrentPosition());
//            telemetry.addData("LB", backLeft.getCurrentPosition());
//            telemetry.addData("RB", backRight.getCurrentPosition());
//            telemetry.addData("Power:", power);
//            telemetry.addData("Side:", side);
//            telemetry.addData("Turn:", turn);
            telemetry.update();
        }
    }

    void getGamePadValues() {
        fwd = gamepad1.left_stick_y;                // For mecanum drive forward and backward
        side = gamepad1.left_stick_x;               // For mecanum drive sideway
        turn = gamepad1.right_stick_x;              // For bot turns

        btn_x1 = gamepad1.x;                        // For toggle of gyro assistance
        btn_b1 = gamepad1.b;                        // For toggle of gyro assistance

        lower_arm_stick = gamepad2.left_stick_y;    // For lower arm movement
        grabber_stick = gamepad2.right_stick_y;     // For grabber

        btn_y = gamepad2.y;                         // For linear servo - up
        btn_a = gamepad2.a;                         // For linear servo - down
        btn_x2 = gamepad2.x;                        // For initialize positions
        btn_b2 = gamepad2.b;                        // For initialize positions

        left_bumper_1 = gamepad1.left_bumper;       // For front grabber
        right_bumper_1 = gamepad1.right_bumper;     // For right grabber
        left_bumper_2 = gamepad2.left_bumper;         // For claw release
        right_bumper_2 = gamepad2.right_bumper;       // For claw grip

        dpad_left = gamepad2.dpad_left;             // For rotation servo
        dpad_right = gamepad2.dpad_right;           // For rotation servo

        //updates joystick values
        if( Math.abs(fwd) < deadzone_hi ) fwd = 0;
        if( Math.abs(side) < deadzone_hi ) side = 0;
        if( Math.abs(turn) < deadzone ) turn = 0;
        if( Math.abs(lower_arm_stick) < deadzone ) lower_arm_stick = 0;
        if( Math.abs(grabber_stick) < deadzone ) grabber_stick = 0;
        //checks deadzones
    }

    void initializePositions() {
        // Init claw and grabber positions
        leftClaw.setPosition(claw_left_initial);
        rightClaw.setPosition(claw_right_initial);
        clawExpanded = false;
        grabber.setPosition(0.0);
        grabberOffset = 0.0;
        clawOffset = 0.0;
        front_left_grab.setPosition(0.6);
        front_right_grab.setPosition(0.1);
        sleep(2000);

        // Init linear servo position
//        linear.setPosition(linear_initial);

        // Initialize lower arm position
        telemetry.addData("Status", "7");
        telemetry.update();
        while ( lower_sensor.getState()) {
//            telemetry.addData("Status", "7.5");
//            telemetry.addData("Status", lower_sensor.getState());
//            telemetry.update();
            lowerArm.setTargetPosition(lowerArm.getCurrentPosition() - arm_down_step);
            lowerArm.setPower(arm_down_power);
        }
        lowerArm.setPower(0);
        lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "8");
        telemetry.addData("LINEAR=", linear_motor.getCurrentPosition());
        telemetry.update();
        // Initialize arm motor position
        linear_motor.setTargetPosition(linear_motor.getCurrentPosition() + 200);
        linear_motor.setPower(linear_motor_up_power);

        telemetry.addData("Status", "9");
        telemetry.addData("LINEAR=", linear_motor.getCurrentPosition());
        telemetry.update();

        while ( arm_limit.getState()) {
            linear_motor.setTargetPosition(linear_motor.getCurrentPosition() - (int) (linear_motor_down_step * 0.1 ));
            linear_motor.setPower(linear_motor_down_power * 0.1 );
        }
        linear_motor.setPower(0);
        linear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "10");
        telemetry.update();

    }

    void moveToPickPosition() {
        // Init claw and grabber positions

        // Init linear servo position
//        linear.setPosition(linear_initial);

        // Initialize lower arm position
        lowerArm.setPower(arm_up_power/3);
        while (opModeIsActive() && lowerArm.getCurrentPosition()<815) {
            lowerArm.setTargetPosition(lowerArm.getCurrentPosition() + arm_down_step);
        }
        lowerArm.setPower(0);
        lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize arm motor position
        linear_motor.setPower(linear_motor_up_power/3);
        while (opModeIsActive() && linear_motor.getCurrentPosition()<2697) {
            linear_motor.setTargetPosition(linear_motor.getCurrentPosition() + linear_motor_down_step);
        }

        linear_motor.setPower(0);
        linear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftClaw.setPosition(0.39);
        rightClaw.setPosition(0.59);
        clawExpanded = false;
        grabber.setPosition(0.8);
        grabberOffset = 0.0;
        clawOffset = 0.0;
        front_left_grab.setPosition(0.6);
        front_right_grab.setPosition(0.1);
        sleep(2000);

    }



    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
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
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = 0.02;

        angle = getAngle();

        if (angle < 5 && angle > -5)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
}

