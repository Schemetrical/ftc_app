
package org.firstinspires.ftc.teamcode.repeaters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Repeaters Teleop", group="Repeaters")  // @Autonomous(...) is the other common choice
public class RepeatersOp extends OpMode
{
    /* Declare OpMode members. */
    private RepeatersHardware robot = new RepeatersHardware();
    private ElapsedTime runtime = new ElapsedTime();

     //Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);

        runtime.reset();
        // telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
            // function to turn robot 180 degree
            robot.leftMotor.setPower(-gamepad1.left_trigger + gamepad1.right_trigger);
            robot.rightMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        }   else if (gamepad1.dpad_up) {
            robot.leftMotor.setPower(1);
            robot.rightMotor.setPower(1);
        }   else if (gamepad1.dpad_down) {
            robot.leftMotor.setPower(-1);
            robot.rightMotor.setPower(-1);
        }   else {
            robot.leftMotor.setPower(-gamepad1.left_stick_y);
            robot.rightMotor.setPower(-gamepad1.right_stick_y);
        }


        robot.collectorMotor.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
        robot.elevatorMotor.setPower(-gamepad2.left_stick_y);

        if (gamepad1.x) {
            robot.flickerMotor.setPower(1);
        } else if (gamepad1.y) {
            robot.flickerMotor.setPower(-1);
        } else {
            robot.flickerMotor.setPower(0);
        }


        //driver1 drives the robot and rotates robot, driver2 collects - elevates - and shoots balls
    }

    /*
     * Code to run ONCE after the driver hits STOP
                                                                     */
    @Override
    public void stop() {
        for (DcMotor motor: robot.motors) {
            motor.setPower(0);
        }
    }

}