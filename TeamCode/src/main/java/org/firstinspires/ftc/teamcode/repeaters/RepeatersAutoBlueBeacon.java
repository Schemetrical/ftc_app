package org.firstinspires.ftc.teamcode.repeaters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by MichaelL on 4/18/17.
 */

@Autonomous(name="Repeaters Teleop", group="Repeaters")
public class RepeatersAutoBlueBeacon extends LinearOpModeCamera{
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

        // Step 1: use leftmotor rotate robot right (30deg?) align with cornervortex
        robot.rightMotor.setPower(0);
        robot.leftMotor.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() <0.25)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 2:  Drive forward
        robot.rightMotor.setPower(1);
        robot.leftMotor.setPower(1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 3:  use rightmotor rotate robot left (30deg?) align with playingfieldwall
        robot.rightMotor.setPower(1);
        robot.leftMotor.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.25)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Drive forward to search for white line
        robot.flickerMotor.setPower(0);
        robot.rightMotor.setPower(-1);
        robot.leftMotor.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 5: Press beacon
        // Step 6: Move onto the next one
    }
}

