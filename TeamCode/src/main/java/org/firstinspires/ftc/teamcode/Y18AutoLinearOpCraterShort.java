package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Y18CraterShort", group="GG")
@Disabled
public class Y18AutoLinearOpCraterShort extends Y18AutoLinearOp
{
    @Override
    public void runOpMode() {
        isCraterTripFlag_= true;
        isShortTripFlag_ = true;

        super.runOpMode();
    }
}
