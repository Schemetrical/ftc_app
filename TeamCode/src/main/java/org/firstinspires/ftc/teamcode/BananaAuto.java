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
//    HiTechnicNxtGyroSensor gyro    = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM   = 10.16 ;     // For figuring circumference
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    private ElapsedTime runtime = new ElapsedTime();

    boolean red = false;

    private static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    private static final int ds2 = 2;
    private static final double MOVE_SPEED = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);
//        gyro = (HiTechnicNxtGyroSensor)hardwareMap.gyroSensor.get("gs");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
//        telemetry.addData(">", "Calibrating Gyro");    //
//        telemetry.update();
//
//
//        gyro.calibrate();
//
//        // make sure the gyro is calibrated before continuing
//        while (!isStopRequested() && gyro.isCalibrating())  {
//            sleep(50);
//            idle();
//        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        robot.servoButtonRotate.setPosition(0.5);
        robot.servoBallStopper.setPosition(0.4);
        robot.servoButtonLinearSlide.setPower(robot.STOPPING_SERVO);
        robot.lightSensor.enableLed(true);
        startCamera();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
//            telemetry.addData(">", "Robot Heading = %d", /*gyro.getIntegratedZValue()*/);
            telemetry.update();
            idle();
        }
//        gyro.resetZAxisIntegrator();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        drive(DRIVE_SPEED, 5);

        //TODO: Uncomment
//        performActionWithDuration(() -> {
//            robot.motorFlicker.setPower(1.0);
//        }, 1, "Shoot 1");
//
//        performActionWithDuration(() -> {
//            robot.servoBallStopper.setPosition(0.0);
//            robot.motorFlicker.setPower(1.0);
//        }, 1, "Shoot 2");

        turn(TURN_SPEED, red ? 30 : 10); //TODO:  figure out how many ticks is 45 deg
        sleep(500);

        drive(DRIVE_SPEED, red ? -40 : 40);
        turn(TURN_SPEED, red ? 10 : -10);
        sleep(500);

        findWhiteLine(4);
        pushButton();

        findWhiteLine(6);
        pushButton();

        turn(TURN_SPEED, red ? -10 : 10);
        drive(DRIVE_SPEED, red ? 48.0 : -48.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void drive (double speed, double distance) {
        int newLeftTarget;
        int newRightTarget;
        int moveCounts;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_CM);
            newLeftTarget = robot.motorLeft.getCurrentPosition() + moveCounts;
            newRightTarget = robot.motorRight.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.motorLeft.setPower(speed);
            robot.motorRight.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.motorLeft.isBusy() && robot.motorRight.isBusy())) {
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

    public void turn (double speed, double angle) {
        int newLeftTarget;
        int newRightTarget;
        int moveCounts;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(angle * COUNTS_PER_CM);
            newLeftTarget = robot.motorLeft.getCurrentPosition() + moveCounts;
            newRightTarget = robot.motorRight.getCurrentPosition() - moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.motorLeft.setPower(speed);
            robot.motorRight.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.motorLeft.isBusy() && robot.motorRight.isBusy())) {
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
            drive(DRIVE_SPEED, -4);
        } else {
            drive(DRIVE_SPEED, 4);
        }
        performActionWithDuration(() -> {
            robot.servoButtonLinearSlide.setPower(1);
        }, 1, "Push Button 1");
        performActionWithDuration(() -> {
            robot.servoButtonLinearSlide.setPower(1);
            // 0 left 1 right
            robot.servoButtonRotate.setPosition(redOnLeft && red ? 0 : 1);
        }, 1, "Push Button 2");
        robot.servoButtonLinearSlide.setPower(0);
    }

    private void findWhiteLine(double timeout) {
        robot.move(MOVE_SPEED, MOVE_SPEED);
        while (opModeIsActive() && (robot.lightSensor.getLightDetected() < WHITE_THRESHOLD) && (runtime.seconds() < 6.0)) {
            // Display the light level while we are looking for the line
            telemetry.addData("Light Level: ",  robot.lightSensor.getLightDetected());
            telemetry.update();
        }
        for (DcMotorSimple motor: robot.allMotors) {
            motor.setPower(0);
        }
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

//    @Override
//    public void runOpMode() {
//
//        /*
//         * Initialize the drive system variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.init(hardwareMap);
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Ready to run");
//        telemetry.update();
//        startCamera();
//
//        robot.servoButtonRotate.setPosition(0.5);
//        robot.lightSensor.enableLed(true);
//
////        double voltage = hardwareMap.voltageSensor.get("motor_controller").getVoltage();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
//        startCamera();
//
//        // Step 1:  Drive forward for 1 second
//        performActionWithDuration(() -> {
//            robot.move(MOVE_SPEED, MOVE_SPEED);
//        }, .5, "Drive Forward");
//
//        // Step 2:  Shoot
//        performActionWithDuration(() -> {
//            robot.motorFlicker.setPower(1.0);
//        }, 2, "Shoot");
//
//        // Step 3: Turn
//        if (red) {
//            performActionWithDuration(() -> {
//                robot.move(MOVE_SPEED, -MOVE_SPEED);
//            }, .5, "Turn");
//        } else {
//            performActionWithDuration(() -> {
//                robot.move(-MOVE_SPEED, MOVE_SPEED);
//            }, .5, "Turn");
//        }
//
//        // Step 4: Move until bumping wall
//        performActionWithDuration(() -> {
//            robot.move(MOVE_SPEED, MOVE_SPEED);
//        }, 3, "Move until Wall");
//
//        // Step 5: Turn
//        if (red) {
//            performActionWithDuration(() -> {
//                robot.move(-MOVE_SPEED, MOVE_SPEED);
//            }, .5, "Turn");
//        } else {
//            performActionWithDuration(() -> {
//                robot.move(MOVE_SPEED, -MOVE_SPEED);
//            }, .5, "Turn");
//        }
//
//        // Step 6: Go to white line
//        findWhiteLine(4); // timeout is 4s
//
//        // Step 7: Push Button
//        pushButton();
//
//        // Step 8: Go to white line
//        findWhiteLine(6);
//
//        // Step 9: Push Button
//        pushButton();
//
//        // Step 10: Turn
//        performActionWithDuration(() -> {
//            robot.servoButtonLinearSlide.setPower(-1);
//            robot.move(red ? -MOVE_SPEED : MOVE_SPEED, red ? MOVE_SPEED : -MOVE_SPEED);
//        }, .5, "Turn");
//        robot.servoButtonLinearSlide.setPower(0);
//
//        // Step 11: Move until bumping center pole
//        performActionWithDuration(() -> {
//            robot.move(red ? MOVE_SPEED : -MOVE_SPEED, red ? MOVE_SPEED : -MOVE_SPEED);
//        }, 6, "Move until Wall");
//
//        // Stop all driveMotors
//        for (DcMotorSimple motor: robot.allMotors) {
//            motor.setPower(0);
//            robot.lightSensor.enableLed(false);
//        }
//    }

}
