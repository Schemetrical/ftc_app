package org.firstinspires.ftc.teamcode.repeatersv2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by MichaelL on 5/3/17.
 */

@Autonomous (name="RepeatersAuto2BallsNoCenterV2",group="RepeatersV2")
public class RepeatersAuto2BallsNoCenterV2 extends RepeatersAuto2BallsV2{
    @Override
    public void runOpMode() {
        autoNoCenter = true;
        super.runOpMode();
    }
}
