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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.PI;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

/**
 * Created by Yichen Cao on 2017-02-25.
 */

@TeleOp(name="Banana Teleop", group="Banana")  // @Autonomous(...) is the other common choice
public class BananaOp extends OpMode {
    /* Declare OpMode members. */
    BananaHardware robot = new BananaHardware();
    private ElapsedTime runtime = new ElapsedTime();
    private OrientationManager orientationManager;
    boolean relative = true;

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
        orientationManager = new OrientationManager(hardwareMap, telemetry);
        orientationManager.start();
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
        // leftMotor.setPower(-gamepad1.left_stick_y);
        // rightMotor.setPower(-gamepad1.right_stick_y);

        double direction = atan(-gamepad1.left_stick_y/gamepad1.left_stick_x);
        // pythagorean
        double power = sqrt(pow(gamepad1.left_stick_y, 2) + pow(gamepad1.left_stick_x, 2));

        if (gamepad1.a) {
            relative = true;
        }
        if (gamepad1.b) {
            relative = false;
        }

        double orientation = relative ? orientationManager.azimuth : 0;

        moveRobot(orientation + PI/2 - direction, power, gamepad1.right_stick_x);
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

    void moveRobot(double direction, double power, double rotation) {

        // case 1: only rotation, rotate at full power
        if (power == 0) {
            for (DcMotor motor: robot.motors) {
                motor.setPower(rotation);
            }
            return;
        }

        // case 2/3: there is lateral movement, so calculate that. If there is rotation, average the two.
        double scale = rotation == 0 ? 1 : 0.5;

        double x = power * cos(direction);
        double y = power * sin(direction);

        for (int i = 0; i < robot.motors.length; i++) {
            DcMotor motor = robot.motors[i];
            double motorX = BananaHardware.MOTOR_XY[2*i-1];
            double motorY = BananaHardware.MOTOR_XY[2*i];
            // some dot product magic
            double dotProduct = x * motorX + y * motorY;
            motor.setPower(dotProduct * scale + rotation * 0.5);
        }
    }

}
