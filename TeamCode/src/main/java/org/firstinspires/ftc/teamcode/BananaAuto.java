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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
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
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

//        double voltage = hardwareMap.voltageSensor.get("motor_controller").getVoltage();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        startCamera();

        // Step 1:  Drive forward for 1 second
        robot.motorFrontLeft.setPower(-1);
        robot.motorBackRight.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Shoot
        robot.motorFrontLeft.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorFlicker.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Move again until white line
        robot.motorFlicker.setPower(0);
        robot.motorFrontLeft.setPower(-1);
        robot.motorBackRight.setPower(1);
        runtime.reset();

        // get a reference to our Light Sensor object.
        LightSensor lightSensor = hardwareMap.lightSensor.get("sensor_light");
        //  lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        // seek for white line cap at 6 seconds
        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD) && (runtime.seconds() < 6.0)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Leg 3 Light: ",  lightSensor.getLightDetected());
            telemetry.update();
        }

        // Step 4:  Stop and hit light
        robot.motorFrontLeft.setPower(0);
        robot.motorBackRight.setPower(0);

        if (opModeIsActive()) {
            hitLightSequence();
        }

        // Step 5: Find line again, using diagonal but can drive along easy axis
        robot.motorFrontLeft.setPower(-1);
        robot.motorFrontRight.setPower(1);
        robot.motorBackLeft.setPower(-1);
        robot.motorBackRight.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        while (opModeIsActive() && (lightSensor.getLightDetected() < WHITE_THRESHOLD) && (runtime.seconds() < 6.0)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Leg 5 Light: ",  lightSensor.getLightDetected());
            telemetry.update();
        }

        // Step 6:  Stop and hit light
        for (DcMotor motor: robot.driveMotors) {
            motor.setPower(0);
        }

        if (opModeIsActive()) {
            hitLightSequence();
        }

        stopCamera();

        // Step 7: Park Robot?

        telemetry.addData("FINISH",  lightSensor.getLightDetected());
        telemetry.update();

        // Stop all driveMotors
        for (DcMotor motor: robot.driveMotors) {
            motor.setPower(0);
        }
        orientationManager.stop();
    }

    private void hitLightSequence() {

        boolean redOnLeft = isRedOnLeft();

        robot.motorFrontLeft.setPower(redOnLeft ? -1 : 1);
        robot.motorFrontRight.setPower(redOnLeft ? 1 : -1);
        robot.motorBackLeft.setPower(redOnLeft ? -1 : 1);
        robot.motorBackRight.setPower(redOnLeft ? 1 : -1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Offset: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        for (DcMotor motor: robot.driveMotors) {
            motor.setPower(0);
        }
        sleep(500);

        robot.motorFrontLeft.setPower(-1);
        robot.motorFrontRight.setPower(-1);
        robot.motorBackLeft.setPower(1);
        robot.motorBackRight.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Approach: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.motorFrontLeft.setPower(1);
        robot.motorFrontRight.setPower(1);
        robot.motorBackLeft.setPower(-1);
        robot.motorBackRight.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Back: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        for (DcMotor motor: robot.driveMotors) {
            motor.setPower(0);
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
}
