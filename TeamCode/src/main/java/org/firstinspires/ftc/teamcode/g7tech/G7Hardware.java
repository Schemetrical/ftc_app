package org.firstinspires.ftc.teamcode.g7tech;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtCompassSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Created by Yichen Cao on 2017-02-25.
 */

class G7Hardware {
    /* Public OpMode members. */
    /* Declare OpMode members. */
    DcMotor motorFront, motorLeft, motorRight, motorBack;

    DcMotorSimple[] driveMotors;
    DcMotorSimple[] allMotors;

    CRServo servoButtonLinearSlide;
    Servo servoButtonRotate;

//    HiTechnicNxtCompassSensor compassSensor;
//    LightSensor lightSensorNear, lightSensorFar;

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

//        compassSensor = (HiTechnicNxtCompassSensor)ahwMap.compassSensor.get("cs");
        servoButtonLinearSlide = ahwMap.crservo.get("sbls");
        servoButtonRotate = ahwMap.servo.get("sbr");
//
//        lightSensorNear = ahwMap.lightSensor.get("lsn");
//        lightSensorFar = ahwMap.lightSensor.get("lsf");

        // Set all motors to zero power
        for (DcMotorSimple motor: allMotors) {
            motor.setPower(0);
        }
    }


    void move(double direction, double power, double rotation) {
        // case 1: only rotation, rotate at full power
        if (power == 0) {
            for (DcMotorSimple motor: driveMotors) {
                motor.setPower(rotation);
            }
            return;
        }

        // case 2/3: there is lateral movement, so calculate that. If there is rotation, average the two.
        double scale = rotation == 0 ? 1 : 0.5;

        double x = power * cos(direction);
        double y = power * sin(direction);

        for (int i = 0; i < driveMotors.length; i++) {
            DcMotorSimple motor = driveMotors[i];
            double motorX = MOTOR_XY[2*i];
            double motorY = MOTOR_XY[2*i+1];
            // some dot product magic
            double dotProduct = x * motorX + y * motorY;
            double finalPower = dotProduct * scale + rotation * 0.5;
            if (Double.isNaN(finalPower)) finalPower = 0;
            motor.setPower(finalPower);
        }
    }
}

