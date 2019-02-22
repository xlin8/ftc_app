package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * TeleOp codes for depot side
 */

@TeleOp(name="Y18TeleCraneDepot", group="GG")
//@Disabled
public class Y18TeleCraneDepot extends Y18TeleCrane
{

   ///  Constructor
   public Y18TeleCraneDepot() {
   }

   ///Code to run when the op mode is initialized goes here
   @Override
      public void init() {
         super.init();
         crater_trip_ = false;
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

