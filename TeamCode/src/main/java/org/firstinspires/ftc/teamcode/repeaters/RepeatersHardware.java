package org.firstinspires.ftc.teamcode.repeaters;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
/**
 * Created by micro on 26-Feb-17.
 */

public class RepeatersHardware {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor collectorMotor;
    DcMotor elevatorMotor;
    DcMotor flickerMotor;

    Servo beaconServo;
    Servo autobeaconServo;

    DcMotor[] motors;
    DcMotorSimple[] allMotors;
    //    HiTechnicNxtCompassSensor compassSensor;
    LightSensor lightSensor;

    public void init(HardwareMap ahwMap) {

        // define and initialize driveMotors
        leftMotor = ahwMap.dcMotor.get("leftMotor");
        rightMotor = ahwMap.dcMotor.get("rightMotor");
        collectorMotor = ahwMap.dcMotor.get("collectorMotor");
        elevatorMotor = ahwMap.dcMotor.get("elevatorMotor");
        flickerMotor = ahwMap.dcMotor.get("flickerMotor");

        // define servos
        beaconServo = ahwMap.servo.get("beaconServo");
        autobeaconServo = ahwMap.servo.get("autobeaconServo");

        motors = new DcMotor[]{leftMotor, rightMotor, collectorMotor, elevatorMotor, flickerMotor};

        //set motor power to 0
        for (DcMotor motor: motors) {
            motor.setPower(0);
        }
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        allMotors = new DcMotorSimple[]{
                leftMotor,
                rightMotor,
                collectorMotor,
                elevatorMotor,
                flickerMotor,
        };
    }

    void move(double left, double right) {
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    void beacon(boolean moveDown) {
        if (moveDown) {
            beaconServo.setPosition(0.94);
        } else {
            beaconServo.setPosition(0.52);
        }
    }
}
