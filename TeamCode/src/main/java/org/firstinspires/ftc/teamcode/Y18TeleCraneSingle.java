package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp codes for depot side
 */

@TeleOp(name="Y18TeleCraneFast", group="GG")
//@Disabled
public class Y18TeleCraneFast extends Y18TeleCrane
{

   ///  Constructor
   public Y18TeleCraneFast() {
   }

   ///Code to run when the op mode is initialized goes here
   @Override
      public void init() {
         super.init();
         crater_trip_ = true;
         fast_low_sen_drive_ = true;
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

