package org.firstinspires.ftc.teamcode.g7tech;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import for_camera_opmodes.LinearOpModeCamera;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

/**
 * Created by micro on 26-Feb-17.
 */

public class G7Auto extends LinearOpModeCamera {

    /* Declare OpMode members. */
    private G7Hardware robot = new G7Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private double initialLightIntensity = 0.0;
    private static final int ds2 = 2;
    private static final double     WHITE_THRESHOLD = 0.1;
    private static final double COLOR_DIFF_THRESHOLD = 10000000;

    public boolean red = false;

    public enum Color {
        RED, BLUE, UNSURE
    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        robot.lightSensor.enableLed(true);
        sleep(500);
        initialLightIntensity = robot.lightSensor.getLightDetected();


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for game start (driver press PLAY)
        waitForStart();

        // Step 1:  Move
        performActionWithDuration(() -> {
            robot.move(PI/2, 1, 0);
        }, 1, "Move");

        // Step 2:  Shoot
        performActionWithDuration(() -> {
            robot.motorCatapultLeft.setPower(1);
        }, .5, "Shoot");

        // Step 3:  Move
        performActionWithDuration(() -> {
            robot.move(PI/2, 1, 0);
        }, .5, "Move");

        // Step 4:  Shoot
        performActionWithDuration(() -> {
            robot.motorCatapultRight.setPower(1);
        }, .5, "Shoot");

        // Step 5:  Move
        performActionWithDuration(() -> {
            robot.move(red ? PI : 0, 1, 0);
        }, 2, "Move");

        // Step 6:  Find White line
        findWhiteLine(3);
        ramSequence();

        // Step 7:  Find White line
        findWhiteLine(7);
        ramSequence();
    }

    private void findWhiteLine(double timeout) {
        runtime.reset();
        sleep(500);
        robot.move(PI/2, 0.25, 0);
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
    }

    private void pushButton() {

        ramSequence();
        Color color = getColor();

        while ((color == Color.UNSURE || ((color == Color.RED) ^ red)) && opModeIsActive()) {
            sleep(500);
            ramSequence();
            sleep(1000);
            color = getColor();
        }
    }

    private void ramSequence() {
        performActionWithDuration(() -> {
            robot.move(red ? PI : 0, 0.5, 0);
        }, 1.5, "Ram");

        performActionWithDuration(() -> {
            robot.move(red ? 0 : PI, 0.5, 0);
        }, .5, "Unram");
    }
    interface RobotAction {
        void performAction();
    }

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
                for (int x = 0; x < rgbImage.getWidth(); x++) {
                    for (int y = 0; y < rgbImage.getHeight(); y++) {
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

    private void performActionWithDuration(G7Auto.RobotAction action, double duration, String description) {
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
