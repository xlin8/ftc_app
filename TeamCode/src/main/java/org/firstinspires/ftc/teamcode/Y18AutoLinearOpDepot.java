package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
/*
created by Aditi on Dec 12th 2018
 */
@Autonomous(name="Y18AutoDepot", group="GG")
//@Disabled
public class Y18AutoLinearOpDepot extends Y18AutoLinearOp
{

    @Override
    public void runOpMode() {
        isCraterTripFlag_ = false;
        isShortTripFlag_ = false;

        super.runOpMode();
    }
}
