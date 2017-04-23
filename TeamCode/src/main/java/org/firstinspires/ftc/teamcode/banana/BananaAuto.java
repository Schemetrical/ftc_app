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

    public enum Color {
        RED, BLUE, UNSURE
    }

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    private static final double     DRIVE_SPEED             = 0.4;     // Nominal speed for better accuracy.
    private static final double     TURN_SPEED              = 0.2;     // Nominal half speed for better accuracy.

    private ElapsedTime runtime = new ElapsedTime();

    boolean red = false;
    boolean shooting = true;
    boolean pushingBall = false;
    boolean ramp = false;
    boolean testingColor = false;

    private static final double     WHITE_THRESHOLD = 0.1;  // spans between 0.1 - 0.5 from dark to light
    private static final int ds2 = 2;
    private static final double COLOR_DIFF_THRESHOLD = 2000000;

    double initialLightIntensity = 0.2;

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
//            runRamp();
        } else if (pushingBall) {
            runBall();
        } else if (testingColor) {
            while (opModeIsActive()) {
                Color color = getColor();
                sleep(1000);
            }
        } else {
            runBeacon();
        }

        cleanup();
    }
/*
    private void runRamp() {
        if (red) {
            driveStraight(DRIVE_SPEED * 2, 10);
        } else {
            driveStraight(DRIVE_SPEED * 2, 10);
            shoot();

            driveStraight(DRIVE_SPEED * 2, 30);
            turn(DRIVE_SPEED * 2, 50);
            driveStraight(DRIVE_SPEED * 2, 40);
            turn(DRIVE_SPEED * 2, 70);
            driveStraight(DRIVE_SPEED * 2, 60);
        }
    }*/

    private void runBall() {
        if (red) {
            performActionWithDuration(() -> robot.move(DRIVE_SPEED, DRIVE_SPEED), 1.1, "Move 1");
        } else {
            performActionWithDuration(() -> robot.move(DRIVE_SPEED, DRIVE_SPEED), 1.3, "Move 1");
        }
        shoot();
        sleep(17000);
        performActionWithDuration(() -> robot.move(DRIVE_SPEED, DRIVE_SPEED), 2, "Move 2");
        cleanup();
    }


    private void runBeacon() {

        if (red) {
// RED  =======================
/*
            performActionWithDuration(() -> robot.move(DRIVE_SPEED, DRIVE_SPEED), .7, "Move 1");
            shoot();

            drive(DRIVE_SPEED, 0, 32);

            driveStraight(DRIVE_SPEED, red ? -149 : 130); // 106
            findWhiteLine(8);

            turn(TURN_SPEED, red ? -37 : 48);*/
        } else {
// BLUE =======================
            performActionWithDuration(() -> robot.move(DRIVE_SPEED, DRIVE_SPEED), 1, "Move 1");
            shoot();

//            performActionWithDuration(() -> robot.strafe(DRIVE_SPEED), 1, "Strafe");
            performActionWithDuration(() -> robot.move(DRIVE_SPEED, DRIVE_SPEED), 1, "Move 2");

//            performActionWithDuration(() -> robot.strafe(DRIVE_SPEED), 1.6, "Strafe 3");
        }


// BEACON 1 =======================
        performActionWithDuration(() -> robot.move(0, DRIVE_SPEED), 0.5, "Turn 1");
        findWhiteLine(8);
        pushButton();

// BEACON 2 =======================
        performActionWithDuration(() -> robot.move(DRIVE_SPEED, DRIVE_SPEED), 1, "Move 4");
        findWhiteLine(8);

        pushButton();

/*
        turn(TURN_SPEED, red ? -45 : 30);
        driveStraight(DRIVE_SPEED, red ? 48.0 : -140);*/

        cleanup();
    }

    // ========================
    // FUNCTIONS FOR AUTONOMOUS
    // ========================

    private void setup() {

        robot.servoBallStopper.setPosition(0.4);
        robot.lightSensor.enableLed(true);
        sleep(500);
        initialLightIntensity = robot.lightSensor.getLightDetected();
        startCamera();

        telemetry.addData("Status", "Robot Ready, Initial Light: " + initialLightIntensity);
        telemetry.update();
    }

    private void shoot() {
        if (shooting) {
            performActionWithDuration(() -> robot.motorFlicker.setPower(-1.0), .7, "Shoot 1");
            robot.servoBallStopper.setPosition(0.0);
            sleep(500);

            performActionWithDuration(() -> robot.motorFlicker.setPower(-1.0), 1.1, "Shoot 2");
        }
    }
/*
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
    }*/

    private Color getColor() {
        boolean finished = false;
        Color detectedColor = Color.UNSURE;

        while (opModeIsActive() && !finished) {
            if (imageReady()) { // only do this if an image has been returned from the camera
                int redValue = 0;
                int blueValue = 0;

                // get image, rotated so (0,0) is in the bottom left of the preview window
                Bitmap rgbImage;
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);

                // 480 x 640
                for (int x = rgbImage.getWidth() * 2 / 3; x < rgbImage.getWidth(); x++) {
                    for (int y = rgbImage.getHeight() / 3; y < rgbImage.getHeight() / 3 * 2; y++) {
                        int pixel = rgbImage.getPixel(x, y);
                        redValue += red(pixel);
                        blueValue += blue(pixel);
                    }
                }

                if (abs(blueValue - redValue) > COLOR_DIFF_THRESHOLD) {
                    detectedColor = blueValue > redValue ? Color.BLUE : Color.RED;
                }
                finished = true;
                telemetry.addData("B/R", blueValue + " " + redValue);
                telemetry.addData("Color:", detectedColor);
                telemetry.update();
            } else {
                telemetry.addData("Color:", "Getting");
                telemetry.update();
            }
        }
        return detectedColor;
    }

    private void pushButton() {
//        ramSequence();
        Color color = getColor();
        while (color == Color.UNSURE) {
            color = getColor();
        }
        boolean driveForward = (color == Color.RED) == red; // RIGHT RED AND RED, RIGHT BLUE AND BLUE

        performActionWithDuration(() -> robot.move((driveForward ? 0 : -DRIVE_SPEED) / 2 , (driveForward ? 0 : -DRIVE_SPEED) / 2), 0.5, "Move 1");

        ramSequence();
    }

    private void ramSequence() {
        performActionWithDuration(() -> {
//            robot.strafe(DRIVE_SPEED);
        }, 1.5, "Ram");

        performActionWithDuration(() -> {
//            robot.strafe(-DRIVE_SPEED);
        }, 0.7, "Unram");
    }

    private void findWhiteLine(double timeout) {

        runtime.reset();

        sleep(500);
        robot.move(red ? -DRIVE_SPEED * 0.25 : DRIVE_SPEED * 0.25, red ? -DRIVE_SPEED * 0.25 : DRIVE_SPEED * 0.25);
        while (opModeIsActive() && (robot.lightSensor.getLightDetected() < initialLightIntensity + WHITE_THRESHOLD) && (runtime.seconds() < timeout)) {
            // Display the light level while we are looking for the line
            telemetry.addData("Light Level: ",  robot.lightSensor.getLightDetected());
            telemetry.update();
        }
        for (DcMotorSimple motor: robot.allMotors) {
            motor.setPower(0);
        }
    }

    private void cleanup() {
        for (DcMotorSimple motor: robot.allMotors) {
            motor.setPower(0);
        }
        stopCamera();
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
