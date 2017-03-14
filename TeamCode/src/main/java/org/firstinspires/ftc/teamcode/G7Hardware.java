package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Yichen Cao on 2017-02-25.
 */

class G7Hardware {
    /* Public OpMode members. */
    /* Declare OpMode members. */
    DcMotor motorFront, motorLeft, motorRight, motorBack;

    DcMotorSimple[] driveMotors;
    DcMotorSimple[] allMotors;

    static final double[] MOTOR_XY = {1, 0, 0, 1, 0, -1, -1, 0};

    /* Constructor */
    G7Hardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Define and Initialize Motors
        motorFront = ahwMap.dcMotor.get("mf");
        motorLeft = ahwMap.dcMotor.get("ml");
        motorRight = ahwMap.dcMotor.get("mr");
        motorBack = ahwMap.dcMotor.get("mb");

        driveMotors = new DcMotorSimple[]{
                motorFront,
                motorLeft,
                motorRight,
                motorBack
        };

        allMotors = new DcMotorSimple[]{
                motorFront,
                motorLeft,
                motorRight,
                motorBack
        };

        // Set all motors to zero power
        for (DcMotorSimple motor: allMotors) {
            motor.setPower(0);
        }
    }

    void move(double left, double right) {
        motorLeft.setPower(left);
        motorRight.setPower(right);
    }

    // clockwise rotation = positive power
    void rotate(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(-power);
    }
}

