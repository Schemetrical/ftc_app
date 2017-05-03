package org.firstinspires.ftc.teamcode.repeatersv2;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoEx;

/**
 * Created by micro on 26-Feb-17.
 */

public class RepeatersHardwareV2 {
    DcMotor fleftMotor;
    DcMotor frightMotor;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor collectorMotor;
    DcMotor elevatorMotor;
    DcMotor flickerMotor;

    Servo beaconServo;
    CRServo autobeaconServo;

    DcMotor[] motors;
    DcMotorSimple[] allMotors;

    //    HiTechnicNxtCompassSensor compassSensor;
    LightSensor rightlightSensor;

    public void init(HardwareMap ahwMap) {

        // define and initialize driveMotors
        fleftMotor = ahwMap.dcMotor.get("fleftMotor");
        frightMotor = ahwMap.dcMotor.get("frightMotor");
        leftMotor = ahwMap.dcMotor.get("leftMotor");
        rightMotor = ahwMap.dcMotor.get("rightMotor");
        collectorMotor = ahwMap.dcMotor.get("collectorMotor");
        elevatorMotor = ahwMap.dcMotor.get("elevatorMotor");
        flickerMotor = ahwMap.dcMotor.get("flickerMotor");

        // define lightsensors
        rightlightSensor = ahwMap.lightSensor.get("rightlightSensor");

        // define servos
        beaconServo = ahwMap.servo.get("beaconServo");
        autobeaconServo = ahwMap.crservo.get("autobeaconServo");

        motors = new DcMotor[]{fleftMotor, frightMotor, leftMotor, rightMotor, collectorMotor, elevatorMotor, flickerMotor};

        //set motor power to 0
        for (DcMotor motor: motors) {
            motor.setPower(0);
        }
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        allMotors = new DcMotorSimple[]{
                fleftMotor,
                frightMotor,
                leftMotor,
                rightMotor,
                collectorMotor,
                elevatorMotor,
                flickerMotor,
        };

    }

    void move(double left, double right) {
        fleftMotor.setPower(-left);
        frightMotor.setPower(right);
        leftMotor.setPower(-left);
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
