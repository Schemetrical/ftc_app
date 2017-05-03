package org.firstinspires.ftc.teamcode.repeatersv2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by MichaelL on 5/3/17.
 */

@Autonomous (name="RepeatersAuto2BallsRampV2",group="RepeatersV2")
public class RepeatersAuto2BallsRampV2 extends RepeatersAutoV2 {
    @Override
    public void runOpMode() {
        autoramp = true;
        super.runOpMode();
    }
}
