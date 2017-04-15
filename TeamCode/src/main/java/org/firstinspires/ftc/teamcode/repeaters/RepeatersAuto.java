package org.firstinspires.ftc.teamcode.repeaters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by micro on 26-Feb-17.
 */

@Autonomous(name="Repeaters Autonomous", group="Repeater")
public class RepeatersAuto extends LinearOpModeCamera {

    /* Declare OpMode members. */
    private RepeatersHardware robot = new RepeatersHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for game start (driver press PLAY)
        waitForStart();

        // Step 0:  time delay
            sleep(15000);
        // Step 1:  Drive forward to aim
            robot.rightMotor.setPower(-1);
            robot.leftMotor.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 1:  Shoot
            robot.rightMotor.setPower(0);
            robot.leftMotor.setPower(0);
            robot.flickerMotor.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive forward to park on center vortex
            robot.flickerMotor.setPower(0);
            robot.rightMotor.setPower(-1);
            robot.leftMotor.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
}
