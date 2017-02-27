package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    DcMotor[] motors;

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

        motors = new DcMotor[]{motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight};

        // Set all motors to zero power
        for (DcMotor motor: motors) {
            motor.setPower(0);
        }
    }
}
