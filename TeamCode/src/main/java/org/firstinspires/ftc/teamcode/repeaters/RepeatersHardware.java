package org.firstinspires.ftc.teamcode.repeaters;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    DcMotor[] motors;

    public void init(HardwareMap ahwMap) {

        //define and initialize driveMotors
        leftMotor = ahwMap.dcMotor.get("leftMotor");
        rightMotor = ahwMap.dcMotor.get("rightMotor");
        collectorMotor = ahwMap.dcMotor.get("collectorMotor");
        elevatorMotor = ahwMap.dcMotor.get("elevatorMotor");
        flickerMotor = ahwMap.dcMotor.get("flickerMotor");

        motors = new DcMotor[]{leftMotor, rightMotor, collectorMotor, elevatorMotor, flickerMotor};

        //set motor power to 0
        for (DcMotor motor: motors) {
            motor.setPower(0);
        }
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

    }
}
