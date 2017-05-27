package org.firstinspires.ftc.teamcode.repeatersv2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by MichaelL on 5/27/17.
 */

@Autonomous (name="RepeatersRedBeaconV2", group="Repeaters")
public class RepeatersRedBeaconV2 extends RepeatersAutoV2{
    @Override
    public void runOpMode() {
        red = true;
        super.runOpMode();
    }
}
