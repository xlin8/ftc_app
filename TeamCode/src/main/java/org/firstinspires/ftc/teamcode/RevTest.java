/**
 * This is TeleOp Run for League Meet0
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//import com.qualcomm.robotcore.hardware.ColorRangeSensor;  // NON-EXISTING


/// TeleOp Run for League Meet0 
@TeleOp(name="RevTest", group="GG")
@Disabled
public class RevTest extends Y17LM0Common
{

   //boolean USE_ENCODER_FOR_TELEOP = false;
   boolean USE_ENCODER_FOR_TELEOP = true;

   /// Other variables
   boolean USE_REV_TOUCH = false;
   DigitalChannel rev_touch_;
   boolean USE_REV_RGB = false;
   ColorSensor rev_rgb_;         // REV color sensor
   //boolean USE_REV_RGB2 = true;
   boolean USE_REV_RGB2 = false;
   ColorSensor rev_rgb2_;         // REV color sensor2
   boolean USE_REV_RGB2_RANGE = !USE_REV_RGB2;
   //ColorRangeSensor rev_rgb2_range_;         // REV color/range sensor2
   LynxI2cColorRangeSensor rev_rgb2_range_;         // REV color/range sensor2

   boolean USE_REV_RANGE = true;
   LynxI2cColorRangeSensor rev_range_;         // REV color/range sensor2
   LynxI2cColorRangeSensor rev_range2_;        // REV color/range sensor2

   boolean USE_MR_RANGE = false;
   ModernRoboticsI2cRangeSensor range_;                    // MR range sensor 
   boolean USE_MR_RANGE2 = false;
   ModernRoboticsI2cRangeSensor range2_;                   // MR range sensor2 

   boolean USE_MECANUM_WHEELS = false;
   static final double MAX_SIDE_WALK_POWER = 0.20;

   /// Servos
   boolean TEST_JEWEL_MODULE = true;
   Servo servo_jewel_;                           // servo for rising/lowering long stick
   Servo servo_knock_;                           // servo for knocking the jewel
   double servo_jewel_pos_;                                  
   double servo_knock_pos_;                                 
   //static final double  JEWEL_INIT = 0.7;
   static final double  JEWEL_INIT = 0.9;        // new servo 485HB, 2017/10/13 
   static final double  KNOCK_INIT = 0.3;

   ///  Constructor
   public RevTest() {
   }


   ///  Code to run when the op mode is initialized goes here
   @Override public void init() {
      super.init();

      /// TODO: add tele-op specific hardware here
      if( USE_REV_TOUCH ) {
         //rev_touch_ = hardwareMap.touchSensor.get("rev_touch"); 
         rev_touch_  = hardwareMap.get(DigitalChannel.class, "rev_touch");  // must use DigitalChannel for REV touch sensor
         rev_touch_.setMode(DigitalChannelController.Mode.INPUT);
      }
      if( USE_REV_RGB ) {
         rev_rgb_ = hardwareMap.colorSensor.get("rev_rgb");
      }
      if( USE_REV_RGB2 ) {
         rev_rgb2_ = hardwareMap.colorSensor.get("rev_rgb2");
      }
      if( USE_REV_RGB2_RANGE ) {
         //rev_rgb2_range_ = hardwareMap.colorSensor.get("rev_rgb2");
         rev_rgb2_range_ = hardwareMap.get(LynxI2cColorRangeSensor.class, "rev_rgb2");
      }
      if( USE_REV_RANGE ) {
         rev_range_ = hardwareMap.get(LynxI2cColorRangeSensor.class, "rev_range_right");
         rev_range2_ = hardwareMap.get(LynxI2cColorRangeSensor.class, "rev_range_left");
      }
      if( USE_MR_RANGE ) {
         range_ = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "mr_range");
      }
      if( USE_MR_RANGE2 ) {
         range_ = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "mr_range2");
      }

      if( USE_ENCODER_FOR_TELEOP ) {
         motorLF_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorLB_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorRF_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorRB_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         //motorLF_.setMaxSpeed(4000);  // unavailable for SDK V3.10

         //motorSweeper_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         //motorLift_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }

      if( TEST_JEWEL_MODULE ) {
         servo_jewel_ = hardwareMap.servo.get("servo_jewel");
         servo_knock_ = hardwareMap.servo.get("servo_knock");
         servo_jewel_pos_ = JEWEL_INIT; 
         servo_knock_pos_ = KNOCK_INIT; 
      }

   }

   @Override public void init_loop() {
      super.init_loop(); 

      if( USE_ENCODER_FOR_TELEOP ) {
         motorLF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorLB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorRF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorRB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);

         //motorSweeper_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
         //motorLift_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
      }
   }

   @Override public void loop () {
      curr_time_ = timer_.time(); 

      double power_lf = 0, power_lb = 0, power_rf = 0, power_rb = 0;
      double lsy = 0, lsx = 0, rsy = 0, rsx = 0;

      servo_left_jaw_pos_ = CR_SERVO_STOP;
      servo_right_jaw_pos_ = CR_SERVO_STOP;

      // rsy: right_stick_y ranges from -1 to 1, where -1 is full up, and 1 is full down
      // rsx: right_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
      rsy = -gamepad1.right_stick_y;
      rsx = gamepad1.right_stick_x;

      if (gamepad1.a) {
         servo_left_jaw_pos_ = LEFT_JAW_OPEN;
      } else if (gamepad1.b){
         servo_left_jaw_pos_ = LEFT_JAW_CLOSE;
      } else if (gamepad1.x){
         servo_right_jaw_pos_ = RIGHT_JAW_OPEN;
      } else if (gamepad1.y){
         servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
      }

      if (Math.abs(rsx) > 0.2) {
         /// Rotate the robot

         // Use right_stick_x to rotate the robot; left => turn left(CCW), right => turn right(CW)
         // Ignore it if the value is too small to avoid mis-operation.
         // right_stick_y is not used at this time
         power_rf = rsx;
         power_lf = rsx;

         // clip the power_rf/power_lf values so that the values never exceed +/- 1.0
         power_rf = Range.clip(power_rf, -1, 1);
         power_lf = Range.clip(power_lf, -1, 1);

         // scale the joystick value to make it easier to control the robot more precisely at slower speeds.
         power_rf = (double) scaleRotatePower(power_rf);
         power_lf = (double) scaleRotatePower(power_lf);

         power_rb = power_rf;
         power_lb = power_lf;


      } else {
         ///  Use left stick to move/turn the robot to a specific direction at a given speed

         lsy = -gamepad1.left_stick_y;   // throttle
         lsx = gamepad1.left_stick_x;    // direction; lsx>0, turn right <=> RF<LF, left wheels turn faster

         //power_rf = -(-lsx + lsy);
         //power_lf = -(-lsx - lsy);
         power_lf = lsx + lsy;
         power_rf = lsx - lsy;


         // clip the power_rf/power_lf values so that the values never exceed +/- 1.0
         power_rf = Range.clip(power_rf, -1, 1);
         power_lf = Range.clip(power_lf, -1, 1);

         // scale the joystick value to make it easier to control the robot more precisely at slower speeds.
         power_rf = (double) scaleInput(power_rf);
         power_lf = (double) scaleInput(power_lf);

         /// LR is same as LF, RR is same as RF
         power_lb = power_lf;
         power_rb = power_rf;

      }

      if( USE_MECANUM_WHEELS  && Math.abs(rsx) < 0.2 ) {
         if( Math.abs(rsy)<0.2 ) {
            lsy = -gamepad1.left_stick_y;   // throttle
            lsx = gamepad1.left_stick_x;    // direction; lsx>0, turn right <=> RF<LF, left wheels turn faster

            power_lf = lsy + lsx; 
            power_lb = lsy - lsx; 
            power_rf = -lsy + lsx; 
            power_rb = -lsy - lsx; 

            power_lf = Range.clip(power_lf, -1, 1);
            power_lb = Range.clip(power_lb, -1, 1);
            power_rf = Range.clip(power_rf, -1, 1);
            power_rb = Range.clip(power_rb, -1, 1);

            power_lf = (double) scaleInput(power_lf);
            power_lb = (double) scaleInput(power_lb);
            power_rf = (double) scaleInput(power_rf);
            power_rb = (double) scaleInput(power_rb);
         } else {      
            // just for sidewalk, use rsy to scale it
            double sidewalk_r = Math.abs(rsy)*MAX_SIDE_WALK_POWER;
            power_lf = lsx * sidewalk_r; 
            power_lb = -power_lf; 
            power_rf = power_lf; 
            power_rb = -power_lf; 
         }
      }

      /// Set power values for all motors
      motorRF_.setPower(power_rf);
      motorRB_.setPower(power_rb);
      motorLF_.setPower(power_lf);
      motorLB_.setPower(power_lb);

      servo_left_jaw_.setPosition(servo_left_jaw_pos_);
      servo_right_jaw_.setPosition(servo_right_jaw_pos_);

      if( TEST_JEWEL_MODULE ) {
         if( gamepad1.right_bumper ) {
            if( gamepad1.dpad_up ) {
               servo_jewel_pos_ += 0.1;
            } else if( gamepad1.dpad_down ) {
               servo_jewel_pos_ -= 0.1;
            } else if( gamepad1.dpad_left ) {
               servo_knock_pos_ += 0.1;
            } else if( gamepad1.dpad_right ) {
               servo_knock_pos_ -= 0.1;
            }
         }
         servo_jewel_pos_ = Range.clip(servo_jewel_pos_, 0, 1);
         servo_jewel_.setPosition( servo_jewel_pos_ ); 
         servo_knock_pos_ = Range.clip(servo_knock_pos_, 0, 1);
         servo_knock_.setPosition( servo_knock_pos_ ); 
      } 

      // Send telemetry data back to driver station for debugging
      boolean show_wheel_power = true;
      boolean show_heading = true;
      boolean show_voltage = true;
      boolean show_rev_rgb = true;
      boolean show_rev_touch = true;
      boolean show_mr_range = true;
      boolean show_rev_range = true;

      telemetry.addData("GiftGears(Test)", "Team:"+(isRedTeam()?"RED":"BLUE")+", Time:"+String.format("%.2f",curr_time_));

      if( show_wheel_power )  {
         telemetry.addData("WheelPower", "LF/RF/LB/RB: " + String.format("%.2f", motorLF_.getPower()) + "/" + String.format("%.2f", motorRF_.getPower()) + "/" + String.format("%.2f", motorLB_.getPower()) + "/" + String.format("%.2f", motorRB_.getPower())); 
         if( USE_ENCODER_FOR_TELEOP ) {
            telemetry.addData("EncPos", ": LF="+String.valueOf(motorLF_.getCurrentPosition())+", RF="+String.valueOf(motorRF_.getCurrentPosition())+", LB="+String.valueOf(motorLB_.getCurrentPosition())+",RB="+String.valueOf(motorRB_.getCurrentPosition()));
         }
      }
      if( show_heading && imu_!=null ) {
         telemetry.addData("IMU", "heading: " + String.format("%.2f", getHeading())); 
      } 
      if( show_voltage ) {
         telemetry.addData("Battery", String.format("#sensors: %d, voltage: %.2fV", numVoltageSensors(), getBatteryVoltage(true)));
      }
      if( show_rev_touch && rev_touch_!=null ) {
         //telemetry.addData("REV_Touch", String.format("pressed: %s",rev_touch_.isPressed()?"YES":"NO"));
         telemetry.addData("REV_Touch", String.format("state: %s, pressed: %s",rev_touch_.getState()?"HIGH":"LOW",rev_touch_.getState()?"NO":"YES"));
      }

      if( show_rev_rgb && rev_rgb_!=null ) {
         telemetry.addData("REV_RGB", String.format("R/G/B/Alpha=%d/%d/%d/%d",rev_rgb_.red(),rev_rgb_.green(),rev_rgb_.blue(),rev_rgb_.alpha()));      

      }
      if( show_rev_rgb && rev_rgb2_!=null ) {
         telemetry.addData("REV_RGB2", String.format("R/G/B/Alpha=%d/%d/%d/%d",rev_rgb2_.red(),rev_rgb2_.green(),rev_rgb2_.blue(),rev_rgb2_.alpha()));      
      } 
      if( show_rev_rgb && rev_rgb2_range_!=null ) {
         telemetry.addData("REV_RGB2_RANGE", String.format("R/G/B/Alpha=%d/%d/%d/%d, Distance=%.2fcm",rev_rgb2_range_.red(),rev_rgb2_range_.green(),rev_rgb2_range_.blue(),rev_rgb2_range_.alpha(),rev_rgb2_range_.getDistance(DistanceUnit.CM)));      
      } 
      if( show_rev_range && rev_range_!=null ) {
         telemetry.addData("REV_RANGE", String.format("R/G/B/Alpha=%d/%d/%d/%d, Distance=%.2fcm",rev_range_.red(),rev_range_.green(),rev_range_.blue(),rev_range_.alpha(),rev_range_.getDistance(DistanceUnit.CM)));      
         telemetry.addData("REV_RANGE_LEFT", String.format("R/G/B/Alpha=%d/%d/%d/%d, Distance=%.2fcm",rev_range2_.red(),rev_range2_.green(),rev_range2_.blue(),rev_range2_.alpha(),rev_range2_.getDistance(DistanceUnit.CM)));      
      } 
      if( show_mr_range && range_!=null ) {
         telemetry.addData("RangeSensor", String.format("ultra/opt/dist=%d/%.2f/%.2f",range_.rawUltrasonic(),range_.cmOptical(),range_.getDistance(DistanceUnit.CM)));
      }
      if( show_mr_range && range2_!=null ) {
         telemetry.addData("RangeSensor2", String.format("ultra/opt/dist=%d/%.2f/%.2f",range2_.rawUltrasonic(),range2_.cmOptical(),range2_.getDistance(DistanceUnit.CM)));
      }

      if( TEST_JEWEL_MODULE ) {
         telemetry.addData("Jewel Module", String.format("jewel_pos=%.2f, knock_pos=%.2f",servo_jewel_.getPosition(),servo_knock_.getPosition())); 
      } 

   }


   // Code to run when the op mode is first disabled goes here
   @Override public void stop () {
   }


   /*
    * This method scales the joystick input so for low joystick values, the
    * scaled value is less than linear.  This is to make it easier to drive
    * the robot more precisely at slower speeds.
    */
   double scaleInput(double dVal) {
      //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
      double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

      // Get the corresponding index for the scaleInput array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }

      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }

      return dScale;
   }

   /// Scale the robot rotating power
   double scaleRotatePower(double dVal) {
      //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
      double[] scaleArray = {0.0, 0.15, 0.18, 0.21, 0.24, 0.27, 0.3, 0.33, 0.36, 0.39, 0.42, 0.45, 0.48, 0.51, 0.54, 0.57, 0.60};

      // Get the corresponding index for the scaleInput array.
      int index = (int) (dVal * 16.0);
      if (index < 0) {
         index = -index;
      } else if (index > 16) {
         index = 16;
      }

      double dScale = 0.0;
      if (dVal < 0) {
         dScale = -scaleArray[index];
      } else {
         dScale = scaleArray[index];
      }

      return dScale;
   }

} 
