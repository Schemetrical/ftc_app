package org.firstinspires.ftc.teamcode.banana;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;

    DcMotorSimple motorFlicker;
    DcMotorSimple motorBallSpinner;
    private DcMotorSimple motorWinchLeft;
    private DcMotorSimple motorWinchRight;

    Servo servoBallStopper;
    Servo servoSlideReleaseLeft, servoSlideReleaseRight;

    LightSensor lightSensor;

    DcMotorSimple[] allMotors;

    static final double STOPPING_SERVO = 0.08;

    /* Constructor */
    BananaHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Define and Initialize Motors
        motorFrontLeft = ahwMap.dcMotor.get("mfl");
        motorFrontRight = ahwMap.dcMotor.get("mfr");
        motorBackLeft = ahwMap.dcMotor.get("mbl");
        motorBackRight = ahwMap.dcMotor.get("mbr");

        motorFlicker = ahwMap.dcMotor.get("mf");
        motorBallSpinner = ahwMap.dcMotor.get("ms");

        motorWinchLeft = ahwMap.dcMotor.get("mwl");
        motorWinchRight = ahwMap.dcMotor.get("mwr");

//        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorFlicker.setDirection(DcMotorSimple.Direction.REVERSE);

        allMotors = new DcMotorSimple[]{
                motorFrontLeft,
                motorFrontRight,
                motorBackLeft,
                motorBackRight,
                motorFlicker,
                motorBallSpinner,
                motorWinchLeft,
                motorWinchRight
        };

//        lightSensor = ahwMap.lightSensor.get("ls");

        servoBallStopper = ahwMap.servo.get("sbs");

        servoSlideReleaseLeft = ahwMap.servo.get("ssrl");
        servoSlideReleaseRight = ahwMap.servo.get("ssrr");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorWinchRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        for (DcMotorSimple motor: allMotors) {
            motor.setPower(0);
        }
    }

    void move(double left, double right) {
        motorFrontLeft.setPower(-left);
        motorBackLeft.setPower(-left);
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right);
    }

    void strafe(double power) { // right is positive, left is negative
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
    }

    // clockwise rotation = positive power
    void rotate(double power) {
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(-power);
    }



    void winch(double power) {
        motorWinchLeft.setPower(power);
        motorWinchRight.setPower(power);
    }
}
