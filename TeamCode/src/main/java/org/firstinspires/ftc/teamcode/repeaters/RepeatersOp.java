
package org.firstinspires.ftc.teamcode.repeaters;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.min;

@TeleOp(name="Repeaters Teleop", group="Repeaters")  // @Autonomous(...) is the other common choice
public class RepeatersOp extends OpMode
{
    /* Declare OpMode members. */
    private RepeatersHardware robot = new RepeatersHardware();
    private ElapsedTime runtime = new ElapsedTime();

    // State variables for beacon
    private boolean gamepad2pressed;
    private boolean beaconUp;

     //Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        runtime.reset();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
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

        // Setup beacon servo variables, and move it to raised position
        gamepad2pressed = false;
        beaconUp = true;
        robot.beaconServo.setPosition(0.52);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.update();

        //driver1 drives the robot and rotates robot, driver2 collects - elevates - and shoots balls

        // GAMEPAD 1
        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) { // Trigger turn robot
            robot.move(-gamepad1.left_trigger + gamepad1.right_trigger
                      , gamepad1.left_trigger - gamepad1.right_trigger);
        }   else if (gamepad1.dpad_up) { // All forward
            robot.move(1, 1);
        }   else if (gamepad1.dpad_down) { // All backward
            robot.move(-1, -1);
        }   else { // Set to joystick control
            robot.move(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        }

        // Ball Flicker; gamepad1 x and y
        if (gamepad1.x) {
            robot.flickerMotor.setPower(1);
        } else if (gamepad1.y) {
            robot.flickerMotor.setPower(-1);
        } else {
            robot.flickerMotor.setPower(0);
        }


        //GAMEPAD 2
        robot.collectorMotor.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
        robot.elevatorMotor.setPower(-gamepad2.left_stick_y);

        if (gamepad2.x) { // Switch like behavior for gampad2.x for beacon pusher
            if (!gamepad2pressed) {
                robot.beacon(beaconUp); // If is beacon up, move it down
                beaconUp = !beaconUp;
                gamepad2pressed = true;
            }
        } else { // Once is depressed, reset variable
            gamepad2pressed = false;
        }
        //backup for linearslidep

        robot.autobeaconServo.setPower(min(gamepad2.right_stick_y + 0.05, 1));
        /*/
        if (gamepad2.b) {
            robot.autobeaconServo.setPower(1);
        }   else if (gamepad2.a) {
            robot.autobeaconServo.setPower(-1);
        }
        /*/

    }

    /*
     * Code to run ONCE after the driver hits STOP
                                                                     */
    @Override
    public void stop() {
        for (DcMotor motor: robot.motors) {
            motor.setPower(0);
        }
        robot.beacon(false); // Sets beacon to up position
    }

}
