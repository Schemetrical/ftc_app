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
    private DcMotorSimple motorLinearSlideWinchLeft, motorLinearSlideWinchRight;

    DcMotorSimple motorFlicker;
    DcMotorSimple motorBallSpinner;

    CRServo servoButtonLinearSlide;
    Servo servoButtonRotate;
    CRServo servoForkliftRelease;
    Servo servoBallStopper;

    LightSensor lightSensor;

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
        motorLinearSlideWinchLeft = ahwMap.dcMotor.get("mlswl");
        motorLinearSlideWinchRight = ahwMap.dcMotor.get("mlswr");
        motorBallSpinner = ahwMap.dcMotor.get("mbs");

//        motorLinearSlideWinchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFlicker.setDirection(DcMotorSimple.Direction.REVERSE);

        allMotors = new DcMotorSimple[]{
                motorLeft,
                motorRight,
                motorFlicker,
                motorLinearSlideWinchLeft,
                motorLinearSlideWinchRight,
                motorBallSpinner};

        lightSensor = ahwMap.lightSensor.get("ls");

        servoForkliftRelease = ahwMap.crservo.get("sflr");
        servoButtonLinearSlide = ahwMap.crservo.get("sbls");
        servoButtonRotate = ahwMap.servo.get("sbr");
        servoBallStopper = ahwMap.servo.get("sbs");

        // Set all motors to zero power
        for (DcMotorSimple motor: allMotors) {
            motor.setPower(0);
        }
    }

    void winch(double power) {
        motorLinearSlideWinchLeft.setPower(power);
        motorLinearSlideWinchRight.setPower(power);
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
