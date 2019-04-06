package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
created by Aditi on Dec 12th 2018
 */
@Autonomous(name="Y18AutoCraterDoubleSample", group="GG")
@Disabled
public class Y18AutoLinearOpCraterDoubleSample extends Y18AutoLinearOp
{

    @Override
    public void runOpMode() {
        isCraterTripFlag_ = true;
        isShortTripFlag_ = false;
        isCraterDoubleSampleTrip = true;

        super.runOpMode();
    }
}
