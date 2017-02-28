package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.sqrt;

/**
 * Created by Yichen Cao on 2017-02-25.
 */

class BananaHardware {
    /* Public OpMode members. */
    /* Declare OpMode members. */
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;

    DcMotor motorFlicker;
    DcMotor motorLinearSlideWinch;
    DcMotor motorBallSpinner;

    DcMotor[] driveMotors;

    private static final double SQRT22 = sqrt(2)/2;

    static final double[] MOTOR_XY = {-SQRT22, -SQRT22, -SQRT22, SQRT22, SQRT22, -SQRT22, SQRT22, SQRT22};

    /* Constructor */
    BananaHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Define and Initialize Motors
        motorFrontLeft = ahwMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = ahwMap.dcMotor.get("motorFrontRight");
        motorBackLeft = ahwMap.dcMotor.get("motorBackLeft");
        motorBackRight = ahwMap.dcMotor.get("motorBackRight");

        motorFlicker = ahwMap.dcMotor.get("motorFlicker");
        motorLinearSlideWinch = ahwMap.dcMotor.get("motorLinearSlideWinch");
        motorBallSpinner = ahwMap.dcMotor.get("motorBallSpinner");

        motorLinearSlideWinch.setDirection(DcMotor.Direction.REVERSE);

        driveMotors = new DcMotor[]{motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight};

        // Set all driveMotors to zero power
        for (DcMotor motor: driveMotors) {
            motor.setPower(0);
        }
    }
}
