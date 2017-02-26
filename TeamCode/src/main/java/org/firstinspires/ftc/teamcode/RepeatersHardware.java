package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;
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

    private HardwareMap hwMap  = null;
    private ElapsedTime period = new ElapsedTime();

    public void init(HardwareMap ahwMap) {
        //reference to hwMap
        hwMap = ahwMap;
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
        //set motors to run without encoders
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flickerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /***
         *  Define and initialize ALL installed servos.
         * arm = hwMap.servo.get("arm");
         * claw = hwMap.servo.get("claw");
         * arm.setPosition(ARM_HOME);
         * claw.setPosition(CLAW_HOME);
         */

    }
}
