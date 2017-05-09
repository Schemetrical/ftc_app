package org.firstinspires.ftc.teamcode.repeatersv2;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.repeatersv2.RepeatersHardwareV2;

import for_camera_opmodes.LinearOpModeCamera;

import static java.lang.Math.abs;

/**
 * Created by MichaelL on 5/3/17.
 */

class RepeatersAutoV2 extends LinearOpModeCamera {

    /* Declare OpMode members. */
    private RepeatersHardwareV2 robot   = new RepeatersHardwareV2();   // Use a Pushbot's hardware

    public enum Color {
        RED, BLUE, UNSURE
    }

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    private static final double     DRIVE_SPEED             = 0.6;     // Nominal speed for better accuracy.

    private ElapsedTime runtime = new ElapsedTime();

    boolean red = false;
    boolean shooting = true;
    boolean shooting2 = true;
    boolean autoBall = false;
    boolean autoBall2 = false;
    boolean autoNoCenter = false;
    boolean autoramp = false;
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
        if (autoramp) {
            runRamp();
        } else if (autoBall) {
            runBall();
        } else if (autoBall2) {
            runBall2();
        } else if (autoNoCenter) {
            runBallNoCenter();
        }
        else {
            runBeacon();
        }

        cleanup();
    }

    private void runRamp() {
        sleep(15000);
        if (red) {
            performActionWithDuration(() -> robot.move(DRIVE_SPEED,DRIVE_SPEED), 0.7, "Move 1");
            shoot2();
            performActionWithDuration(() -> robot.move(-DRIVE_SPEED,DRIVE_SPEED), 0.5, "Rotate 1");
            performActionWithDuration(() -> robot.move(DRIVE_SPEED,DRIVE_SPEED), 1.7, "Move2");
        } else {
            performActionWithDuration(() -> robot.move(DRIVE_SPEED, DRIVE_SPEED), 0.7, "Move 1");
            shoot2();
            performActionWithDuration(() -> robot.move(DRIVE_SPEED,-DRIVE_SPEED), 0.5, "Rotate 1");
            performActionWithDuration(() -> robot.move(DRIVE_SPEED,DRIVE_SPEED), 1.7, "Move2");
        }
        cleanup();
    }
    private void runBall() {
        sleep(15000);
        if (red) {
            performActionWithDuration(() -> robot.move(-DRIVE_SPEED, -DRIVE_SPEED), 0.7, "Move 1");
        } else {
            performActionWithDuration(() -> robot.move(-DRIVE_SPEED, -DRIVE_SPEED), 0.7, "Move 1");
        }
        shoot();
        performActionWithDuration(() -> robot.move(-DRIVE_SPEED, -DRIVE_SPEED), 1.7, "Move 2");
        cleanup();
    }

    private void runBall2() {
        sleep(15000);
        if (red) {
            performActionWithDuration(() -> robot.move(-DRIVE_SPEED, -DRIVE_SPEED), 0.7, "Move 1");
        } else {
            performActionWithDuration(() -> robot.move(-DRIVE_SPEED, -DRIVE_SPEED), 0.7, "Move 1");
        }
        shoot2();
        performActionWithDuration(() -> robot.move(-DRIVE_SPEED, -DRIVE_SPEED), 1.7, "Move 2");
        cleanup();
    }

    private void runBallNoCenter() {
        sleep(15000);
        if (red) {
            performActionWithDuration(() -> robot.move(-DRIVE_SPEED, -DRIVE_SPEED), 0.7, "Move 1");
        } else {
            performActionWithDuration(() -> robot.move(-DRIVE_SPEED, -DRIVE_SPEED), 0.7, "Move 1");
        }
        shoot2();
        sleep(500);
        cleanup();
    }

    private void runBeacon() {

        if (red) {
// RED  =======================
            performActionWithDuration(() -> robot.move(-DRIVE_SPEED, -DRIVE_SPEED), 1.3, "Move 1");
            shoot();
            performActionWithDuration(() -> robot.move(-DRIVE_SPEED, 0), 0.25, "Rotate 1");
            performActionWithDuration(() -> robot.move(-DRIVE_SPEED, -DRIVE_SPEED),1.5, "Move 2");
            performActionWithDuration(() -> robot.move(0, -DRIVE_SPEED), 0.25, "Rotate 2");
            sleep(500);

            //Beacon1
            findWhiteLine(8);
            ramSequence();
            //Beacon2
            findWhiteLine(8);
            ramSequence();

        } else {
// BLUE =======================
            performActionWithDuration(() -> robot.move(DRIVE_SPEED, DRIVE_SPEED), 1.5, "Move 1");
            performActionWithDuration(() -> robot.move(0, DRIVE_SPEED),0.25, "Rotate 1");
            sleep(500);

            //Beacon1
            findWhiteLine(8);
            ramSequence();
            //Beacon2
            findWhiteLine(8);
            ramSequence();

            performActionWithDuration(() -> robot.move(DRIVE_SPEED, -DRIVE_SPEED), 0.4, "Prep 1");
            performActionWithDuration(() -> robot.move(-DRIVE_SPEED, -DRIVE_SPEED), 1.0, "Prep 2");
            shoot2();
        }

        cleanup();
    }

    // ========================
    // FUNCTIONS FOR AUTONOMOUS
    // ========================

    private void setup() {

        robot.rightlightSensor.enableLed(true);
        sleep(500);
        initialLightIntensity = robot.rightlightSensor.getLightDetected();
        startCamera();

        telemetry.addData("Status", "Robot Ready, Initial Light: " + initialLightIntensity);
        telemetry.update();
    }

    private void shoot() {
        if (shooting) {
            performActionWithDuration(() -> robot.flickerMotor.setPower(-1), 1.5, "Shot 1");
        }
    }

    private void shoot2() {
        if (shooting2) {
            performActionWithDuration(() -> robot.flickerMotor.setPower(-1), 1.5, "Shot 1");
            performActionWithDuration(() -> robot.elevatorMotor.setPower(1), 2.2, "Prep 1");
            performActionWithDuration(() -> robot.flickerMotor.setPower(-1), 1.5, "Shot 2");

        }
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

        ramSequence();
    }

    private void ramSequence() {
        performActionWithDuration(() -> robot.autobeaconServo.setPower(1), 1.5, "Ram");
        sleep(500);
        performActionWithDuration(() -> robot.autobeaconServo.setPower(-1), 1.5, "Unram");
    }

    private void findWhiteLine(double timeout) {

        runtime.reset();

        sleep(500);
        robot.move(red ? -DRIVE_SPEED * 0.25 : DRIVE_SPEED * 0.25, red ? -DRIVE_SPEED * 0.25 : DRIVE_SPEED * 0.25);
        while (opModeIsActive() && (robot.rightlightSensor.getLightDetected() < initialLightIntensity + WHITE_THRESHOLD) && (runtime.seconds() < timeout)) {
            // Display the light level while we are looking for the line
            telemetry.addData("Light Level: ",  robot.rightlightSensor.getLightDetected());
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

    private void performActionWithDuration (RobotAction action, double duration, String description) {
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
