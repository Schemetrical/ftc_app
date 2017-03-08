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
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by Yichen Cao on 2017-02-25.
 */

public class BananaAuto extends LinearOpModeCamera {

    /* Declare OpMode members. */
    private BananaHardware robot   = new BananaHardware();   // Use a Pushbot's hardware

    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_CM       = 10.16 ;     // For figuring circumference
    private static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.8;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    private ElapsedTime runtime = new ElapsedTime();

    boolean red = false;

    private static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    private static final int ds2 = 2;
    private static final double MOVE_SPEED = 0.5;
    private static final double ENCODER_THRESHOLD = 10;

    private static final double TURN_CM_PER_DEG = 0.62;

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        robot.servoButtonRotate.setPosition(0.5);
        robot.servoBallStopper.setPosition(0.4);
        robot.servoButtonLinearSlide.setPower(BananaHardware.STOPPING_SERVO);
        robot.lightSensor.enableLed(true);
        startCamera();

        driveStraight(DRIVE_SPEED, 15);

        //TODO: Uncomment
//        performActionWithDuration(() -> {
//            robot.motorFlicker.setPower(1.0);
//        }, 1, "Shoot 1");
//
//        performActionWithDuration(() -> {
//            robot.servoBallStopper.setPosition(0.0);
//            robot.motorFlicker.setPower(1.0);
//        }, 1, "Shoot 2");

        turn(TURN_SPEED, red ? -135 : 54); //TODO:  figure out how many ticks is 45 deg
        sleep(500);

        driveStraight(DRIVE_SPEED, red ? -100 : 100);

        if (red) {
            turn(TURN_SPEED, 45);
        } else {
//            turn(TURN_SPEED, -54);
            drive(DRIVE_SPEED, 0, -54 * TURN_CM_PER_DEG * 2);
        }

        sleep(500);

        findWhiteLine(4);
        pushButton();

        findWhiteLine(6);
        pushButton();

        turn(TURN_SPEED, red ? -45 : 45);
        driveStraight(DRIVE_SPEED, red ? 48.0 : -48.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    void drive(double speed, double left, double right) {
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
                            robot.motorLeft.getCurrentPosition() - newLeftTarget > ENCODER_THRESHOLD ||
                            robot.motorRight.getCurrentPosition() - newRightTarget > ENCODER_THRESHOLD)) {
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
                int greenValue = 0;

                // get image, rotated so (0,0) is in the bottom left of the preview window
                Bitmap rgbImage;
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

                for (int x = 0; x < rgbImage.getWidth(); x++) {
                    for (int y = 0; y < rgbImage.getHeight(); y++) {
                        int pixel = rgbImage.getPixel(x, y);
                        redValue += red(pixel);
                        blueValue += blue(pixel);
                        greenValue += green(pixel);
                    }
                }
                int color = highestColor(redValue, greenValue, blueValue);

                redOnLeft = color == 0;
                finished = true;
                telemetry.addData("Color:", redOnLeft ? "RED BLUE" : "BLUE RED");
                telemetry.update();
            } else {
                telemetry.addData("Color:", "Getting");
                telemetry.update();
            }
        }
        return redOnLeft;
    }

    private void pushButton() {
        boolean redOnLeft = isRedOnLeft();
        if (red) {
            driveStraight(DRIVE_SPEED, -5);
        } else {
            driveStraight(DRIVE_SPEED, 5);
        }
        performActionWithDuration(() -> {
            robot.servoButtonLinearSlide.setPower(-1);
        }, .7, "Push Button 1");
        performActionWithDuration(() -> {
            robot.servoButtonLinearSlide.setPower(-1);
            // 0 left 1 right
            robot.servoButtonRotate.setPosition(redOnLeft && red ? 0 : 1);
        }, 1, "Push Button 2");
        performActionWithDuration(() -> {
            robot.servoButtonLinearSlide.setPower(1);
            robot.servoButtonRotate.setPosition(0.5);
        }, 1, "Retract Button");
        robot.servoButtonLinearSlide.setPower(BananaHardware.STOPPING_SERVO);
    }

    private void findWhiteLine(double timeout) {
        runtime.reset();
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.move(MOVE_SPEED, MOVE_SPEED);
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
