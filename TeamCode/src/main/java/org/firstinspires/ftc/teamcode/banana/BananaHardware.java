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
    DcMotor motorLeft, motorRight;

    DcMotorSimple motorFlicker;
    DcMotorSimple motorBallSpinner;

    CRServo servoButtonLinearSlide;
    Servo servoButtonRotate;
    Servo servoBallStopper;

    LightSensor lightSensorNear, lightSensorFar;

    DcMotorSimple[] allMotors;

    static final double STOPPING_SERVO = 0.08;

    /* Constructor */
    BananaHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Define and Initialize Motors
        motorLeft = ahwMap.dcMotor.get("ml");
        motorRight = ahwMap.dcMotor.get("mr");

        motorFlicker = ahwMap.dcMotor.get("mf");
        motorBallSpinner = ahwMap.dcMotor.get("mbs");

//        motorLinearSlideWinchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFlicker.setDirection(DcMotorSimple.Direction.REVERSE);

        allMotors = new DcMotorSimple[]{
                motorLeft,
                motorRight,
                motorFlicker,
                motorBallSpinner};

        lightSensorNear = ahwMap.lightSensor.get("lsn");
        lightSensorFar = ahwMap.lightSensor.get("lsf");

        servoButtonLinearSlide = ahwMap.crservo.get("sbls");
        servoButtonRotate = ahwMap.servo.get("sbr");
        servoBallStopper = ahwMap.servo.get("sbs");

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
