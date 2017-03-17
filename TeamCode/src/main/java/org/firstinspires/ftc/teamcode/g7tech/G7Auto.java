package org.firstinspires.ftc.teamcode.g7tech;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.repeaters.RepeatersHardware;

import for_camera_opmodes.LinearOpModeCamera;

import static java.lang.Math.PI;

/**
 * Created by micro on 26-Feb-17.
 */

@Autonomous(name="G7 Autonomous", group="G7Tech")
public class G7Auto extends LinearOpModeCamera {

    /* Declare OpMode members. */
    private G7Hardware robot = new G7Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        /*
        robot.compassSensor.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
        while (!robot.compassSensor.calibrationFailed() && (runtime.seconds() < 2)) {
            telemetry.addData("Calibrating", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        if (robot.compassSensor.calibrationFailed()) {
            telemetry.addData("Calibration", "Failed");
            telemetry.update();
        }
        robot.compassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);*/


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for game start (driver press PLAY)
        waitForStart();

        /* Step 1:  Drive forward to aim
            robot.rightMotor.setPower(-1);
            robot.leftMotor.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        */
        // Step 1:  Move
            robot.move(PI/2, 1, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Stop
        robot.move(0, 0, 0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.5)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
}
