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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by Yichen Cao on 2017-02-25.
 */

@Autonomous(name="Banana Autonomous", group="Banana")
public class BananaAuto extends LinearOpModeCamera {

    /* Declare OpMode members. */
    private BananaHardware robot   = new BananaHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    private static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    private static final int ds2 = 2;
    private static final double MOVE_SPEED = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        OrientationManager orientationManager = new OrientationManager();
        orientationManager.start(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        robot.servoButtonPusher.setPosition(0.5);
        robot.lightSensor.enableLed(true);

//        double voltage = hardwareMap.voltageSensor.get("motor_controller").getVoltage();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        startCamera();

        // Step 1:  Drive forward for 1 second
        performActionWithDuration(() -> {
            robot.move(MOVE_SPEED, MOVE_SPEED);
        }, .5, "Drive Forward");

        // Step 2:  Shoot
        performActionWithDuration(() -> {
            robot.motorFlicker.setPower(1.0);
        }, 2, "Shoot");

        // Step 3: Turn
        performActionWithDuration(() -> {
            robot.move(MOVE_SPEED, -MOVE_SPEED);
        }, .5, "Turn");

        // Step 4: Move until bumping wall
        performActionWithDuration(() -> {
            robot.move(MOVE_SPEED, MOVE_SPEED);
        }, 3, "Move until Wall");

        // Step 5: Turn
        performActionWithDuration(() -> {
            robot.move(-MOVE_SPEED, MOVE_SPEED);
        }, .5, "Turn");

        // Step 6: Go to white line
        findWhiteLine(4); // timeout is 4s

        // Step 7: Push Button
        performActionWithDuration(() -> {
            robot.servoButtonPusher.setPosition(isRedOnLeft() ? 0 : 1);
        }, 2, "Push Button");

        // Step 8: Go to white line
        findWhiteLine(6);

        // Step 9: Push Button
        performActionWithDuration(() -> {
            robot.servoButtonPusher.setPosition(isRedOnLeft() ? 0 : 1);
        }, 2, "Push Button");

        // Step 10: Turn
        performActionWithDuration(() -> {
            robot.move(MOVE_SPEED, -MOVE_SPEED);
        }, .5, "Turn");

        // Step 11: Move until bumping center pole
        performActionWithDuration(() -> {
            robot.move(-MOVE_SPEED, -MOVE_SPEED);
        }, 6, "Move until Wall");

        // Stop all driveMotors
        for (DcMotorSimple motor: robot.allMotors) {
            motor.setPower(0);
            robot.lightSensor.enableLed(false);
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
}
