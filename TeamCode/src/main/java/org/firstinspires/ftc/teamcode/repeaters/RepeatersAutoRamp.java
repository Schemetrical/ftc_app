package org.firstinspires.ftc.teamcode.repeaters;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import for_camera_opmodes.LinearOpModeCamera;

/**
 * Created by MichaelL on 4/24/17.
 */


@Autonomous(name="Repeaters Autonomous", group="Repeaters")
public class RepeatersAutoRamp extends LinearOpModeCamera {

    private RepeatersHardware robot = new RepeatersHardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initiate driver system
        robot.init(hardwareMap);
        // Wait for driver to press "PLAY"
        waitForStart();
        // Step 0: time delay
        sleep(20000);
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

        // Step 2:  Start elevator to load second ball
        robot.flickerMotor.setPower(0);
        robot.elevatorMotor.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Shoot second ball
        robot.elevatorMotor.setPower(0);
        robot.flickerMotor.setPower(-1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4: Position for ramp

    }

}
