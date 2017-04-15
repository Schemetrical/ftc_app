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
package org.firstinspires.ftc.teamcode.banana;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Yichen Cao on 2017-02-25.
 */

@TeleOp(name="Banana Teleop", group="Banana")  // @Autonomous(...) is the other common choice
public class BananaOp extends OpMode {
    /* Declare OpMode members. */
    BananaHardware robot = new BananaHardware();
    private ElapsedTime runtime = new ElapsedTime();
    private boolean flipped = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        robot.init(hardwareMap);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.motorWinchRight.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.servoSlideReleaseLeft.setPosition(0.5);
        robot.servoSlideReleaseRight.setPosition(0.5);

        runtime.reset();
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
        robot.servoBallStopper.setPosition(0.4);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        // leftMotor.setPower(-gamepad1.left_stick_y);
        // rightMotor.setPower(-gamepad1.right_stick_y);

        // GAMEPAD 1

        if (gamepad1.b) {
            flipped = true;
        }

        if (gamepad1.a) {
            flipped = false;
        }

        double multiplier = 1;
        if (gamepad1.right_bumper) {
            multiplier = 0.5;
        } else if (gamepad1.left_bumper) {
            multiplier = 0.25;
        }

        if (gamepad1.dpad_up) {
            robot.move(flipped ? -1 : 1, flipped ? -1 : 1);
        } else if (gamepad1.dpad_down) {
            robot.move(flipped ? 1 : -1, flipped ? 1 : -1);
        } else if (gamepad1.left_trigger > 0 || gamepad1.left_trigger > 0) {
            robot.rotate(gamepad1.right_trigger - gamepad1.left_trigger);
        } else {
            robot.move((flipped ? gamepad1.right_stick_y : -gamepad1.left_stick_y) * multiplier,
                    (flipped ? gamepad1.left_stick_y : -gamepad1.right_stick_y) * multiplier);
        }

        // GAMEPAD 2
        robot.motorBallSpinner.setPower(-gamepad2.left_stick_y);

        if (gamepad2.b) {
            robot.motorFlicker.setPower(1);
        } else if (gamepad2.a) {
            robot.motorFlicker.setPower(-1);
        } else {
            robot.motorFlicker.setPower(0);
        }

        if (gamepad2.x) {
            robot.servoSlideReleaseLeft.setPosition(0.05);
            robot.servoSlideReleaseRight.setPosition(0.95);
        } else if (gamepad2.y) {
            robot.servoSlideReleaseLeft.setPosition(0.5);
            robot.servoSlideReleaseRight.setPosition(0.5);
        }

        if (gamepad2.right_bumper) {
            robot.servoBallStopper.setPosition(0.0);
        } else {
            robot.servoBallStopper.setPosition(0.4);
        }

        robot.winch(gamepad2.right_stick_y);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        for (DcMotorSimple motor: robot.allMotors) {
            motor.setPower(0);
        }
    }

}
