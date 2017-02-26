package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
/**
 * Created by micro on 26-Feb-17.
 */

public class RepeatersHardware {
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor collectorMotor = null;
    DcMotor elevatorMotor = null;
    DcMotor flickerMotor = null;

    DcMotor[] motors = {leftMotor, rightMotor, collectorMotor, elevatorMotor, flickerMotor};

    public void init(HardwareMap ahwMap) {

        //define and initialize motors
        leftMotor = ahwMap.dcMotor.get("leftMotor");
        rightMotor = ahwMap.dcMotor.get("rightMotor");
        collectorMotor = ahwMap.dcMotor.get("collectorMotor");
        elevatorMotor = ahwMap.dcMotor.get("elevatorMotor");
        flickerMotor = ahwMap.dcMotor.get("flickerMotor");

        //set motor power to 0
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        collectorMotor.setPower(0);
        elevatorMotor.setPower(0);
        flickerMotor.setPower(0);

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

    }
}
