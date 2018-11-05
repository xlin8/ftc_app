package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by aliri on 10/20/2017.
 */

@Autonomous(name="BluePos1", group="GG")
@Disabled
public class Y17AutoBlue extends Y17AutoTest
{

    ///  Constructor
    public Y17AutoBlue() {
    }

    ///Code to run when the op mode is initialized goes here
    @Override
    public void init() {
        super.init();
        init_wait_time_ = 0.0;
        color_flipped_ = true;
        trip_name_ = "BluePos1"; 
    }

    /// This method will be called once before entering loop()
    @Override
    public void init_loop() {
        super.init_loop();
    }

    /// This method will be called repeatedly in a loop
    @Override
    public void loop() {
        super.loop();
    }

    /// Code to run when the op mode is first disabled goes here
    @Override
    public void stop() {}
}
