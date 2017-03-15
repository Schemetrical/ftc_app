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
package org.firstinspires.ftc.teamcode.g7tech;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.PI;
import static java.lang.Math.atan;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

/**
 * Created by Yichen Cao on 2017-02-25.
 */

@TeleOp(name="G7 Teleop", group="G7Tech")  // @Autonomous(...) is the other common choice
public class G7Op extends OpMode {
    /* Declare OpMode members. */
    private G7Hardware robot = new G7Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    private double offset = 0;
    private boolean relative = true;

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

        robot.compassSensor.setMode(CompassSensor.CompassMode.CALIBRATION_MODE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Calibrating", "%2.5f S, Failed: %b", runtime.seconds(), robot.compassSensor.calibrationFailed());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.compassSensor.setMode(CompassSensor.CompassMode.MEASUREMENT_MODE);
        offset = getDirection();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        // GAMEPAD 1

        if (gamepad1.b) {
            relative = false;
        } else if (gamepad1.a) {
            relative = true;
            offset = getDirection();
        }

        double direction;
        if (gamepad1.left_stick_x == 0) {
            direction = PI/2 * (-gamepad1.left_stick_y > 0 ? 1 : -1);
        } else {
            direction = atan(-gamepad1.left_stick_y/gamepad1.left_stick_x);
            if (gamepad1.left_stick_x < 0) direction += PI;
        }

        if (relative) {
            direction += (getDirection() - offset);
        }

        // pythagorean
        double power = sqrt(pow(gamepad1.left_stick_y, 2) + pow(gamepad1.left_stick_x, 2));

        robot.move(direction, power, -gamepad1.right_stick_x);

        // GAMEPAD 2
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

    private double getDirection() {
        return robot.compassSensor.getDirection() * PI / 180;
    }

}
