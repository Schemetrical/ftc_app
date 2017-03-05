package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.sqrt;

/**
 * Created by Yichen Cao on 2017-02-25.
 */

class BananaHardware {
    /* Public OpMode members. */
    /* Declare OpMode members. */
    private DcMotorSimple motorLeft;
    private DcMotorSimple motorRight;

    DcMotorSimple motorFlicker;
    DcMotorSimple motorLinearSlideWinch;
    DcMotorSimple motorBallSpinner;

    Servo servoButtonPusher;
    CRServo servoForkliftRelease;

    LightSensor lightSensor;

    DcMotorSimple[] allMotors;

    private static final double SQRT22 = sqrt(2)/2;

    static final double[] MOTOR_XY = {-SQRT22, -SQRT22, -SQRT22, SQRT22, SQRT22, -SQRT22, SQRT22, SQRT22};

    /* Constructor */
    BananaHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Define and Initialize Motors
        motorLeft = ahwMap.dcMotor.get("motorLeft");
        motorRight = ahwMap.dcMotor.get("motorRight");

        motorFlicker = ahwMap.dcMotor.get("motorFlicker");
        motorLinearSlideWinch = ahwMap.dcMotor.get("motorLinearSlideWinch");
        motorBallSpinner = ahwMap.dcMotor.get("motorBallSpinner");

        motorLinearSlideWinch.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFlicker.setDirection(DcMotorSimple.Direction.REVERSE);

        allMotors = new DcMotorSimple[]{motorLeft, motorRight, motorFlicker, motorLinearSlideWinch, motorBallSpinner};

        lightSensor = ahwMap.lightSensor.get("sensor_light");
        servoForkliftRelease = ahwMap.crservo.get("servoForkliftRelease");

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
