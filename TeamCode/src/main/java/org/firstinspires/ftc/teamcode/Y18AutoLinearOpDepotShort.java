package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/*
created by Aditi on Dec 12th 2018
 */
@Autonomous(name="Y18AutoDepotShort", group="GG")
//@Disabled
public class Y18AutoLinearOpDepotShort extends Y18AutoLinearOp
{

    @Override
    public void runOpMode() {
        isCraterTripFlag_ = false;
        isShortTripFlag_ = true;

        super.runOpMode();
    }
}
