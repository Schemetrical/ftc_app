/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.banana;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import for_camera_opmodes.LinearOpModeCamera;

import static java.lang.Math.abs;

/**
 * Created by Yichen Cao on 2017-02-25.
 */

class BananaAuto extends LinearOpModeCamera {

    /* Declare OpMode members. */
    private BananaHardware robot   = new BananaHardware();   // Use a Pushbot's hardware

    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_CM       = 10.16 ;     // For figuring circumference
    private static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    private static final double     DRIVE_SPEED             = 0.4;     // Nominal speed for better accuracy.
    private static final double     TURN_SPEED              = 0.2;     // Nominal half speed for better accuracy.

    private ElapsedTime runtime = new ElapsedTime();

    boolean red = false;
    boolean shooting = true;
    boolean pushingBall = false;
    boolean ramp = false;

    private static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    private static final int ds2 = 2;
    private static final double MOVE_SPEED = 0.5;
    private static final double ENCODER_THRESHOLD = 8;
    private static final double COLOR_DIFF_THRESHOLD = 5000000;

    private static final double TURN_CM_PER_DEG = 0.62;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        setup();
        waitForStart();

    // =====================
    // AUTONOMOUS DRIVE CODE
    // =====================



        telemetry.addData("Path", "Complete");
        telemetry.update();
        if (ramp) {
            runRamp();
        } else if (pushingBall) {
            runBall();
        } else {
            runBeacon();
        }
    }

    private void runRamp() {
        driveStraight(DRIVE_SPEED * 2, 10);
        performActionWithDuration(() -> robot.motorFlicker.setPower(-1.0), 1, "Shoot 1");
        robot.servoBallStopper.setPosition(0.0);
        sleep(500);

        performActionWithDuration(() -> robot.motorFlicker.setPower(-1.0), 1.3, "Shoot 2");

        driveStraight(DRIVE_SPEED * 2, 30);
        turn(DRIVE_SPEED * 2, 50);
        driveStraight(DRIVE_SPEED * 2, 40);
        turn(DRIVE_SPEED * 2, 70);
        driveStraight(DRIVE_SPEED * 2, 60);
    }

    private void runBall() {
        driveStraight(DRIVE_SPEED, 25);

        performActionWithDuration(() -> robot.motorFlicker.setPower(-1.0), 1, "Shoot 1");
        robot.servoBallStopper.setPosition(0.0);
        sleep(500);

        performActionWithDuration(() -> robot.motorFlicker.setPower(-1.0), 1.3, "Shoot 2");

        driveStraight(DRIVE_SPEED * 2, 120);
    }


    private void runBeacon() {

        if (!red) {
            driveStraight(DRIVE_SPEED * 0.75, 15);
        }

        if (shooting) {
            performActionWithDuration(() -> robot.motorFlicker.setPower(-1.0), 1, "Shoot 1");
            robot.servoBallStopper.setPosition(0.0);
            sleep(500);

            performActionWithDuration(() -> robot.motorFlicker.setPower(-1.0), 1.3, "Shoot 2");
        }
//        sleep(500);

        if (red) {
            driveStraight(DRIVE_SPEED * 0.75, 41.4);
        }

        // FIRST TURN =======================
//        turn(TURN_SPEED, red ? -117 : 37); // 38.5 at 13.32V
        if (red) {
//            turn(TURN_SPEED, 100);
        } else {
            drive(DRIVE_SPEED, 70 * TURN_CM_PER_DEG, 0); // 65 at 13.82 V
        }

        driveStraight(DRIVE_SPEED, red ? -149 : 106); // 120 at 13.82V

        if (red) {
            turn(TURN_SPEED, 54);
        } else {
//            turn(TURN_SPEED, -54);
            drive(DRIVE_SPEED, 0, 74 * TURN_CM_PER_DEG); // 65 at 13.82 V
        }

//        sleep(500);

        if (!opModeIsActive())
            return;

        findWhiteLine(4);
//        if (red) {
//            drive(DRIVE_SPEED, 0, -66 * TURN_CM_PER_DEG);
//        } else {
////            turn(TURN_SPEED, -54);
//            drive(DRIVE_SPEED, -22 * TURN_CM_PER_DEG, -5 * TURN_CM_PER_DEG); // 65 at 13.82 V
//        }
        pushButton(.7);

        if (!opModeIsActive())
            return;

        driveStraight(MOVE_SPEED * 1.25, red ? -100 : 100);

        findWhiteLine(4);
        pushButton(0);

        if (!opModeIsActive())
            return;

        turn(TURN_SPEED, red ? -45 : 45);
        driveStraight(DRIVE_SPEED, red ? 48.0 : -140);
    }

    // ========================
    // FUNCTIONS FOR AUTONOMOUS
    // ========================

    private void setup() {
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.servoButtonRotate.setPosition(0.5);
        robot.servoBallStopper.setPosition(0.4);
        robot.servoButtonLinearSlide.setPower(BananaHardware.STOPPING_SERVO);
        robot.lightSensor.enableLed(true);
        startCamera();

        telemetry.addData("Status", "Robot Ready");
        telemetry.update();
    }

    private void drive(double speed, double left, double right) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLeft.getCurrentPosition() + (int)(left * COUNTS_PER_CM);
            newRightTarget = robot.motorRight.getCurrentPosition() + (int)(right * COUNTS_PER_CM);

            // Set Target and Turn On RUN_TO_POSITION
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);

            // start motion.
            robot.move(speed, speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() && (
                            abs(robot.motorLeft.getCurrentPosition() - newLeftTarget) > ENCODER_THRESHOLD ||
                            abs(robot.motorRight.getCurrentPosition() - newRightTarget) > ENCODER_THRESHOLD)) {
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.motorLeft.getCurrentPosition(),
                        robot.motorRight.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  speed, speed);
                telemetry.update();
            }

            // Stop all motion;
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);
        }
    }

    private void driveStraight(double speed, double distance) {
        drive(speed, distance, distance);
    }

    private void turn(double speed, double angle) {
        drive(speed, angle * TURN_CM_PER_DEG, -angle * TURN_CM_PER_DEG);
    }

    private boolean isRedOnLeft() {
        boolean finished = false;
        boolean redOnLeft = false;

        while (opModeIsActive() && !finished) {
            if (imageReady()) { // only do this if an image has been returned from the camera
                int redValue = 0;
                int blueValue = 0;

                // get image, rotated so (0,0) is in the bottom left of the preview window
                Bitmap rgbImage;
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

                // 480 x 640
                for (int x = 0; x < rgbImage.getWidth() - 222; x++) {
                    for (int y = 0; y < rgbImage.getHeight() - 246; y++) {
                        int pixel = rgbImage.getPixel(x + 80, y + 127);
                        redValue += red(pixel);
                        blueValue += blue(pixel);
                    }
                }

                redOnLeft = blueValue - redValue < COLOR_DIFF_THRESHOLD;
                finished = true;
                telemetry.addData("B/R", blueValue + " " + redValue);
                telemetry.addData("Color:", redOnLeft ? "RED BLUE" : "BLUE RED");
                telemetry.update();
            } else {
                telemetry.addData("Color:", "Getting");
                telemetry.update();
            }
        }
        return redOnLeft;
    }

    private void pushButton(double offsetDistance) {
        boolean redOnLeft = isRedOnLeft();
        performActionWithDuration(() -> {
            robot.servoButtonLinearSlide.setPower(-1);
            // 0 right 1 left
            robot.servoButtonRotate.setPosition(redOnLeft ^ red ? 0.09 : 0.94);
        }, .9 + offsetDistance, "Push Button 2");
        robot.servoButtonLinearSlide.setPower(BananaHardware.STOPPING_SERVO);
//        sleep(500);
        performActionWithDuration(() -> {
            robot.servoButtonLinearSlide.setPower(1);
            robot.servoButtonRotate.setPosition(0.5);
        }, 1.2, "Retract Button");
        robot.servoButtonLinearSlide.setPower(BananaHardware.STOPPING_SERVO);
    }

    private void findWhiteLine(double timeout) {
        runtime.reset();
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);
        robot.move(red ? -MOVE_SPEED * 0.75 : MOVE_SPEED * 0.75, red ? -MOVE_SPEED * 0.75 : MOVE_SPEED * 0.75);
        while (opModeIsActive() && (robot.lightSensor.getLightDetected() < WHITE_THRESHOLD) && (runtime.seconds() < timeout)) {
            // Display the light level while we are looking for the line
            telemetry.addData("Light Level: ",  robot.lightSensor.getLightDetected());
            telemetry.update();
        }
        for (DcMotorSimple motor: robot.allMotors) {
            motor.setPower(0);
        }
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        driveStraight(MOVE_SPEED / 3, red ? -1.5 : 1.5);
    }

    interface RobotAction {
        void performAction();
    }

    private void performActionWithDuration(RobotAction action, double duration, String description) {
        action.performAction();
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < duration)) {
            telemetry.addData(description, "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        for (DcMotorSimple motor: robot.allMotors) {
            motor.setPower(0);
        }
    }
}
