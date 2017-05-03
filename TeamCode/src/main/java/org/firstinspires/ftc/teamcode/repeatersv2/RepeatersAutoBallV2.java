package org.firstinspires.ftc.teamcode.repeatersv2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by MichaelL on 5/3/17.
 */

@Autonomous (name="RepeatersAutoBallV2", group="RepeatersV2")
public class RepeatersAutoBallV2 extends RepeatersAutoV2 {
    @Override
    public void runOpMode() {
        autoBall = true;
        super.runOpMode();
    }

}
