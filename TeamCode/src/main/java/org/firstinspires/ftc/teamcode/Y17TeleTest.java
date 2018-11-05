/*
 * This is Program for TeleOp Run for FTC 2017-2018 Relic Recovery season
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/// TeleOp Run for League Meet0 
@TeleOp(name="Y17TeleTest", group="GG")
public class Y17TeleTest extends Y17LM0Common 
{

   /// Other variables
   static final double  MIN_BUTTON_INTERVAL = 0.3;

   //static final double  MAX_LIFT_ENC_COUNT = 21500;   // 1.2m
   //static final double  MAX_LIFT_ENC_COUNT = 21500*0.5/1.2;   // 0.5m=>20in for 3 glyphs
   //static final double  MAX_LIFT_ENC_COUNT = 21500*0.6/1.2;   // 0.5m=>20in for 3 glyphs

   /// ALL LIFT VARIABLES MOVED TO COMMON

   boolean DEBUG_DRIVE_MOTORS = false;

   static final boolean USE_X_TO_SHIFT_CLAW = false;
   static final boolean USE_PAD2_JOYSTICKS_FOR_CLAW = true;
   static final boolean USE_PAD2_LEFT_JOYSTICKS_FOR_INTAKE = true;   // use Pad2 left joystick fwd/bkwd to tweak intake opening
   static final boolean USE_QUICK_RESET = true;            // allow use right trigger to auto-reset grabber after shelfing
   static final double  QUICK_RESET_TIME = 3.0;            // allow 3 sec for grabber to reset, incl. lift, claw, and pusher

   static final boolean AUTO_ALIGN_WHEN_STACK = false;     // auto-align when stacking using button B if true. Testing found issues, OFF, driver use LT for manual align, 01/11
   static final boolean AUTO_LIFT_AFTER_GRAB = true;       // auto-lift the glyph off the ground when stacking/grabbing by button B
   //static final boolean AUTO_LOWER_WHEN_PUSH = true;       // auto lower the pusher to align with the bottom glyph for more efficient pushing
   static final boolean AUTO_LOWER_WHEN_PUSH = false;       // auto lower the pusher to align with the bottom glyph for more efficient pushing, disabled, 2018/02/10

   boolean USE_MECANUM_WHEELS = true;
   //boolean USE_MECANUM_FOR_SIDEWALK_ONLY = false;
   boolean USE_MECANUM_FOR_SIDEWALK_ONLY = true;
   //static final double MECANUM_RIGHT_BACK_SCALE = 1.3;    // compensate the unbalanced robot at right back wheel
   static final double MECANUM_RIGHT_BACK_SCALE = 1.0;    // compensate the unbalanced robot at right back wheel

   boolean USE_ENCODER_FOR_TELEOP = true;
   //boolean USE_ENCODER_FOR_TELEOP = false;
   //double ENCODER_MAX_POWER = 0.6; 
   //double ENCODER_MAX_DRIVE_POWER = 1.0; 
   double ENCODER_MAX_DRIVE_POWER = 0.8;   // 2017/12/01
   double ENCODER_MAX_ROTATE_POWER = 0.6; 
   double ENCODER_MAX_SIDEWALK_POWER = 0.5;   // 2018/02/09, cap Mecanum wheel sidewalk power at 0.5

   double RIGHT_POWER_RATIO = 1.00; 
   //double RIGHT_POWER_RATIO = 0.99; 
   //double RIGHT_POWER_RATIO = 0.98; 
   //double RIGHT_POWER_RATIO = 0.97; 

   double last_jaw_close_time_ = 0.0; 
   double last_jaw_open_time_ = 0.0; 
   double last_stacking_time_ = 0.0;  // last time B is pressed for stacking
   double last_reset_time_ = 0.0;     // last time right trigger is pressed for reseting grabber
   double last_push_time_ = 0.0;      // last start time for pushing
   int curr_push_start_enc_ = 0;      // starting lift encoder position for current push

   double STACK_JAW_CLOSE_TIME = 1.0;  // time for jaw to close and grab the glyph
   double STACK_DROP_DELAY = 0.5;  // extra delay to allow top claw to open before dropping the calw for stacking
   //double STACK_DROP_DELAY = 1.0;  // extra delay to allow top claw to open before dropping the calw for stacking
   double last_b1_b2_time_ = 0.0;  // last time B1/B2 is pressed for stacking

   //static final boolean USE_PAD1_FOR_RELIC = true;   // use Pad1 to control relic claw
   static final boolean USE_PAD1_FOR_RELIC = false;   // use Pad1 to control relic claw
   static final double RELIC_DRIVE_FACTOR = 0.7;   // slow down drive when trying to get relic
   boolean relic_mode_ = false; 
   double last_relic_start_time_ = 0.0;      // last time left stick button is pressed for activating relic mode
   double last_relic_releasing_time_ = 0.0;  // last time LB2 is pressed for releasing relic
   double last_relic_lowering_time_ = 0.0;   // last time Pad2/Y is pressed for lowering relic
   static final boolean USE_LOW_SEN_DRIVE_FOR_RELIC = true;   // use low sensitivity drive mode for relic and balancing
   boolean low_sen_drive_ = false;

   /// Intake system control
   double intake_state_change_time_ = 0.0;     // the last time for the intake state change
   static final double INTAKE_CR_SERVO_TURN_TIME = 2.0; 

   double last_intake_fold_time_ = 0.0;      // last time intake servo is folding
   boolean mag_intake_left_on_ = false;      // magnetic limit switch triggered
   double mag_intake_left_trig_time_ = 0.0; // last time left magnetic sensor triggered 
   boolean mag_intake_left_last_state_ = true;  // magnetic limit switch last state, ACTICE LOW
   boolean mag_intake_left_curr_state_ = true;  // magnetic limit switch curr state, ACTICE LOW
   boolean mag_intake_right_on_ = false;      // magnetic limit switch triggered
   double mag_intake_right_trig_time_ = 0.0; // last time left magnetic sensor triggered 
   boolean mag_intake_right_last_state_ = true;  // magnetic limit switch last state, ACTICE LOW
   boolean mag_intake_right_curr_state_ = true;  // magnetic limit switch curr state, ACTICE LOW

   boolean touch_intake_left_curr_state_ = true;  // right touch sensor curr state, ACTICE LOW
   double touch_intake_left_trig_time_ = 0.0; // last time left touch sensor triggered 
   boolean touch_intake_right_curr_state_ = true;  // right touch sensor curr state, ACTICE LOW
   double touch_intake_right_trig_time_ = 0.0; // last time right touch sensor triggered 
   double INTAKE_TOUCH_PUSH_TIME = 0.1;        // keep pushing out for 0.1 sec to avoid osillation
   boolean intake_arm_extended_ = false;       // 

   static final boolean INTAKE_AUTO_GRAB = false;   // automatically grab the glyph when it's aligned
   ///  Constructor
   public Y17TeleTest() {
   }


   ///  Code to run when the op mode is initialized goes here
   @Override public void init() {
      super.init();

      /// TODO: add tele-op specific hardware here
      if( USE_ENCODER_FOR_TELEOP ) {
         motorLF_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorLB_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorRF_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorRB_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         //motorLF_.setMaxSpeed(4000);  // unavailable for SDK V3.10

         if(USE_LIFT) motorLift_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         if(USE_RELIC) motorRelic_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         if(USE_GLYPH_WHEELS) {
            motor_glyph_left_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor_glyph_right_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         }
      }

      if( USE_MECANUM_WHEELS ) {
         motorLF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         motorLB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         motorRF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         motorRB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      }

      last_jaw_close_time_ = 0.0; 
      last_jaw_open_time_ = 0.0; 
      last_stacking_time_ = 0.0; 
      last_reset_time_ = 0.0; 
      last_push_time_ = 0.0; 
      last_b1_b2_time_ = 0.0; 

      servo_jewel_.setPosition(SERVO_JEWEL_INIT);
      servo_knock_.setPosition(SERVO_KNOCK_INIT);
      if(USE_KICKER) servo_kicker_.setPosition(KICKER_INIT);
      if(USE_PUSHER) servo_pusher_.setPosition(CR_SERVO_STOP);

      if( USE_RELIC ) {
         servo_relic_arm_.setPosition(servo_relic_arm_pos_);
         servo_relic_claw_.setPosition(servo_relic_claw_pos_);
      }
   }

   @Override public void init_loop() {
      super.init_loop(); 

      if( USE_ENCODER_FOR_TELEOP ) {
         motorLF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorLB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorRF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorRB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);

         if(USE_LIFT) motorLift_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
         if(USE_RELIC) motorRelic_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
         if(USE_GLYPH_WHEELS) {
            motor_glyph_left_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
            motor_glyph_right_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         }
      } 
   }

   @Override public void loop () {
      curr_time_ = timer_.time(); 

      if(USE_LOW_SEN_DRIVE_FOR_RELIC) {
         low_sen_drive_ = (rsb1_cnt_%2)==1; 
      }

      boolean stack_by_b = false;               
      boolean auto_align_for_stack_on = false;
      if ( gamepad1.b || gamepad2.b ) stack_by_b = true;  // button B is pressed to stack glyph
      if( AUTO_ALIGN_WHEN_STACK && stack_by_b ) {
         ///< FIXME! Detect and handle RevRGB with problem.
         if( !isGlyphSeen() ) {
            stack_by_b = false;               // don't lower lift, not ready yet
         } else if( !isGlyphAligned() ) {
            if( gamepad1.left_trigger>0.5 || gamepad2.left_trigger>0.5 ) {
               // auto-grab is ON
            } else {
               auto_align_for_stack_on = true;   // try to auto-align
            }
         } else {
            // already aligned, do nothing 
         }
      }

      if( USE_RELIC ) { 
         if( USE_PAD1_FOR_RELIC && lsb2_cnt_==0 ) {
            relic_mode_ = ((lsb1_cnt_%2)==1); 
         } else {
            relic_mode_ = ((lsb2_cnt_%2)==1); 
         }
      }

      power_lift = 0.0;

      double power_lf = 0, power_lb = 0, power_rf = 0, power_rb = 0;
      double power_sweeper = 0;
      double lsy = 0, lsx = 0, rsy = 0, rsx = 0;
      double drive_power_f = 1.0; 
      //if( lb1_cnt_%2 != 0 ) drive_power_f = 1.25;  // 25% boost for dflt, => full power
      //if( lb1_cnt_%2 != 0 ) drive_power_f = 0.75;  // 25% slowdown for dflt, for shelfing and balancing 

      servo_left_jaw_pos_ = CR_SERVO_STOP;
      servo_right_jaw_pos_ = CR_SERVO_STOP;
      servo_left_jaw2_pos_ = CR_SERVO_STOP;
      servo_right_jaw2_pos_ = CR_SERVO_STOP;

      // rsy: right_stick_y ranges from -1 to 1, where -1 is full up, and 1 is full down
      // rsx: right_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
      rsy = -gamepad1.right_stick_y;
      rsx = gamepad1.right_stick_x;
/*
      if (gamepad1.a || gamepad2.a) {
         power_lift = 0.2; 
      } else if (gamepad1.b || gamepad2.b){
         power_lift = -0.2; 
      } 
*/
      if( (curr_time_-last_button_time_) > MIN_BUTTON_INTERVAL) {
         if (gamepad1.x){
            x1_cnt_++;
            last_button_time_ = curr_time_;
         } else if (gamepad1.y) {
            y1_cnt_++;
            last_button_time_ = curr_time_;
         }else if (gamepad1.a) {
            a1_cnt_++;
            last_button_time_ = curr_time_;
         } else if (gamepad1.b) {
            b1_cnt_++;
            if(last_b1_b2_time_==0.0 || last_b1_b2_time_<last_button_time_) last_b1_b2_time_ = curr_time_;   // not holding
            last_button_time_ = curr_time_;
         } else if (gamepad1.left_bumper) {
            lb1_cnt_++;
            last_button_time_ = curr_time_;
         } else if (gamepad1.right_bumper) {
            rb1_cnt_++;
            last_button_time_ = curr_time_;
         } else if (gamepad1.left_stick_button) {
            lsb1_cnt_++;
            if( USE_RELIC && USE_PAD1_FOR_RELIC ) {   // reset all button counters for Pad2
               a1_cnt_ = 0; 
               b1_cnt_ = 0; 
               x1_cnt_ = 0; 
               y1_cnt_ = 0; 
               lb1_cnt_ = 0; 
               rb1_cnt_ = 0; 
               rsb1_cnt_ = 0; 
               if((lsb1_cnt_%2)>0) last_relic_start_time_ = curr_time_;
               else last_relic_start_time_ = 0.0; 
            }
            last_button_time_ = curr_time_;
         } else if (gamepad1.right_stick_button) {
            rsb1_cnt_++;
            last_button_time_ = curr_time_;
         }
      }
      if( (curr_time_-last_button_time2_) > MIN_BUTTON_INTERVAL) {
         if (gamepad2.x){
            x2_cnt_++;
            last_button_time2_ = curr_time_;
         } else if (gamepad2.y) {
            y2_cnt_++;
            last_button_time2_ = curr_time_;
         }else if (gamepad2.a) {
            a2_cnt_++;
            last_button_time2_ = curr_time_;
         } else if (gamepad2.b) {
            b2_cnt_++;
            if(last_b1_b2_time_==0.0 || last_b1_b2_time_<last_button_time2_) last_b1_b2_time_ = curr_time_; 
            last_button_time2_ = curr_time_;
         } else if (gamepad2.left_bumper) {
            lb2_cnt_++;
            last_button_time2_ = curr_time_;
         } else if (gamepad2.right_bumper) {
            rb2_cnt_++;
            last_button_time2_ = curr_time_;
         } else if (gamepad2.left_stick_button) {
            lsb2_cnt_++;
            if( USE_RELIC ) {   // reset all button counters for Pad2
               a2_cnt_ = 0; 
               b2_cnt_ = 0; 
               x2_cnt_ = 0; 
               y2_cnt_ = 0; 
               lb2_cnt_ = 0; 
               rb2_cnt_ = 0; 
               rsb2_cnt_ = 0; 
               if((lsb2_cnt_%2)>0) last_relic_start_time_ = curr_time_;
               else last_relic_start_time_ = 0.0; 
            }
            last_button_time2_ = curr_time_;
         } else if (gamepad2.right_stick_button) {
            rsb2_cnt_++;
            last_button_time2_ = curr_time_;
         }
      }


      if( Math.abs(rsx) > 0.1 ) {
         /// Rotate/spin the robot

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
         power_rf = (double) scaleDrivePower(power_rf,drive_power_f);
         power_lf = (double) scaleDrivePower(power_lf,drive_power_f);

         if( USE_LOW_SEN_DRIVE_FOR_RELIC && low_sen_drive_ ) {
            power_rf = (double) scaleDrivePowerLowSensitivity(power_rf,/*drive_power_f*/1.0);
            power_lf = (double) scaleDrivePowerLowSensitivity(power_lf,/*drive_power_f*/1.0);
         }

         power_rf = Range.clip(power_rf, -1, 1);
         power_lf = Range.clip(power_lf, -1, 1);

         /// LR is same as LF, RR is same as RF
         power_lb = power_lf;
         power_rb = power_rf;

      }

/*
      /// Use left/right trigger to drive forward/backward
      boolean trigger_for_lift = false;
      if( a1_cnt_%2==1 && USE_LIFT )  {
         if( Math.abs(gamepad1.left_trigger)>0.1 ) {
            power_lift = gamepad1.left_trigger; 
         }
         else if( Math.abs(gamepad1.right_trigger)>0.1 ) {
            power_lift = -gamepad1.right_trigger; 
         }
         trigger_for_lift = true;
      }
*/
      
      if( DEBUG_DRIVE_MOTORS ) {
         boolean test_drive = false;
         if( gamepad1.left_bumper ) {
            power_lf = 1.0; 
            test_drive = true;
         } else if( gamepad1.right_bumper ) {
            power_lf = -1.0; 
            test_drive = true;
         } else if( Math.abs(gamepad1.left_trigger)>0.1 ) {
            power_lf = gamepad1.left_trigger; 
            test_drive = true;
         } else if( Math.abs(gamepad1.right_trigger)>0.1 ) {
            power_lf = -gamepad1.right_trigger; 
            test_drive = true;
         }
         if( test_drive ) {
            power_lf = Range.clip(power_lf, -1, 1);
            power_lb = power_lf; 
            power_rf = power_rb = -power_lf*RIGHT_POWER_RATIO; 
         }
      }

      /// Control Mecanum wheels by left joystick
      //if( USE_MECANUM_WHEELS && Math.abs(rsx) < 0.2 ) 
         //lsy = -gamepad1.left_stick_y;   // throttle
         //lsx = gamepad1.left_stick_x;    // direction; lsx>0, turn right <=> RF<LF, left wheels turn faster
      /// Use Mecanum wheels by right joystick
      boolean drive_sidewalk = false;
      if( USE_MECANUM_WHEELS && Math.abs(rsx) > JOYSTICK_DEAD_ZONE ) {
         lsx = gamepad1.right_stick_x;    // direction; lsx>0, turn right <=> RF<LF, left wheels turn faster
         lsy = -gamepad1.right_stick_y;   // throttle
         if( USE_MECANUM_FOR_SIDEWALK_ONLY ) lsy = 0; 
         drive_sidewalk = true;

         power_lf = lsy + lsx; 
         power_lb = lsy - lsx; 
         power_rf = -lsy + lsx; 
         power_rb = -lsy - lsx; 

         power_lf = Range.clip(power_lf, -1, 1);
         power_lb = Range.clip(power_lb, -1, 1);
         power_rf = Range.clip(power_rf, -1, 1);
         power_rb = Range.clip(power_rb, -1, 1);

         power_lf = (double) scaleDrivePower(power_lf, drive_power_f);
         power_lb = (double) scaleDrivePower(power_lb, drive_power_f);
         power_rf = (double) scaleDrivePower(power_rf, drive_power_f);
         power_rb = (double) scaleDrivePower(power_rb, drive_power_f);
         power_rb *= MECANUM_RIGHT_BACK_SCALE; 

         double max_mw_power = ENCODER_MAX_SIDEWALK_POWER;      // cap the max wheel power for sidewalk
         power_lf = Range.clip(power_lf, -max_mw_power, max_mw_power);
         power_lb = Range.clip(power_lb, -max_mw_power, max_mw_power);
         power_rf = Range.clip(power_rf, -max_mw_power, max_mw_power);
         power_rb = Range.clip(power_rb, -max_mw_power, max_mw_power); 
      }

      /// Flip robot
      //boolean flip_robot = true; // needed if grabber facing backward
      boolean flip_robot = false;  // grabber facing forward
      if( flip_robot ) {
         double p = power_lf; 
         power_lf = power_rf; 
         power_rf = p; 
         power_lb = power_lf; 
         power_rb = power_rf; 
      }

      /// Automatically align the robot with the glyph
      if( ENABLE_AUTO_GRAB && USE_MECANUM_WHEELS ) {
         boolean auto_grab_on = false;       // auto-grab engaged
         boolean seen_glyph = false;         // glyph curently seen
         boolean prev_seen_glyph = false;    // glyph seen shortly ago
         if( gamepad1.left_trigger>0.5 || gamepad2.left_trigger>0.5) {
            auto_grab_on = true; 
         } else if( auto_align_for_stack_on ) {
            auto_grab_on = true;   // use same codes to turn robot for auto-align
         }

         if( auto_grab_on ) {
            seen_glyph = isGlyphSeen(); 
            if( !seen_glyph ) {
               prev_seen_glyph = (glyph_seen_t_>0.0) && (curr_time_<(glyph_seen_t_+AUTO_GRAB_TURN_TIME+AUTO_GRAB_SHIFT_TIME)); 
            }
         } else {
            glyph_aligned_ = false;   // reset if left trigger is released
            glyph_seen_t_ = 0.0; 
         }

         if( ENABLE_AUTO_GRAB_FOLLOW && auto_grab_on && 
             !seen_glyph && prev_seen_glyph && 
             !auto_align_for_stack_on             // SKIP for auto-align
             ) {
            if( ENABLE_AUTO_GRAB_FOLLOW_DIAG && prev_auto_grab_move_==3 ) { // shift left
               // go left-front, 135-deg
               power_lf = 0.0; 
               power_rb = 0.0; 
               power_rf = -AUTO_GRAB_TURN_POWER; 
               power_lb = AUTO_GRAB_TURN_POWER; 
            } else if( ENABLE_AUTO_GRAB_FOLLOW_DIAG && prev_auto_grab_move_==4 ) {  // shift right
               // go right-front, 45-deg
               power_rb = -AUTO_GRAB_TURN_POWER; 
               power_lf = AUTO_GRAB_TURN_POWER; 
               power_lb = 0.0; 
               power_rf = 0.0; 
            } else {
               // go forward, 90-deg
               power_lf = power_lb = AUTO_GRAB_TURN_POWER; 
               power_rf = power_rb = -AUTO_GRAB_TURN_POWER; 
            }
         } else if( auto_grab_on && (seen_glyph || prev_seen_glyph) ) {
            // Auto-grab if glyph is seen or was seen shortly ago
            if( glyph_seen_t_==0.0 ) {
               // store the first set of values, which are needed to determine the state of glyph
               dist0_mr_range_ = dist_mr_range_; 
               dist0_left_ = dist_left_; 
               dist0_right_ = dist_right_; 
               glyph_seen_t_ = curr_time_; 
               glyph_seen_heading_ = getHeading();
            }

            prev_auto_grab_move_ = auto_grab_move_;  // save the move
            auto_grab_move_ = 0;  // 0 stop, 1 turn left, 2 turn right, 3 shift left, 4 shift right
            if( dist_left_<MIN_DIST_GLYPH && dist_right_<MIN_DIST_GLYPH ) {
               glyph_aligned_ = true;
            } 
            
            if( !glyph_aligned_ ) {
               double elapsed_time = curr_time_ - glyph_seen_t_; 
               double turn_deg = Math.abs(getHeadingDiff( glyph_seen_heading_ ));

               // Diamond, special handling, necessary?? TBD
               if( dist0_left_ <= dist0_right_ ) {
                  if( elapsed_time<=AUTO_GRAB_TURN_TIME && turn_deg<AUTO_GRAB_TURN_MAX_DEG ) {
                     // turn left, N1/S1/S2/L1
                     auto_grab_move_ = 1; 
                  } else if( elapsed_time <= (AUTO_GRAB_TURN_TIME+AUTO_GRAB_SHIFT_TIME) ) {
                     if( dist_left_<MIN_DIST_GLYPH ) {
                        // shift left, N1/S2 => N1
                        auto_grab_move_ = 3; 
                     } else {
                        // shift right, L1 => N3 
                        auto_grab_move_ = 4; 
                     }
                  } else { 
                     // timeout
                     glyph_seen_t_ = 0.0; 
                  }
               } else if( dist0_left_ > dist0_right_ ) {
                  if( elapsed_time<=AUTO_GRAB_TURN_TIME && turn_deg<AUTO_GRAB_TURN_MAX_DEG ) {
                     // turn right, N3/S3/S4/L3
                     auto_grab_move_ = 2; 
                  } else if( elapsed_time <= (AUTO_GRAB_TURN_TIME+AUTO_GRAB_SHIFT_TIME) ) {
                     if( dist_right_<MIN_DIST_GLYPH ) {
                        // shift right, N3/S4 => N3
                        auto_grab_move_ = 4; 
                     } else {
                        // shift left, L3 => N1 
                        auto_grab_move_ = 3; 
                     }
                  } else { 
                     // timeout
                     glyph_seen_t_ = 0.0; 
                  }
               } 
            }

            if( auto_align_for_stack_on && auto_grab_move_>2 ) {
               auto_grab_move_ = 0;    // skip 2nd step of shifting
            }

            // Adjust wheel power accordingly
            if(auto_grab_move_==1) { // turn left
               power_lf = power_lb = 0.0; 
               power_rf = power_rb = -AUTO_GRAB_TURN_POWER; 
            } else if(auto_grab_move_==2) { // turn right
               power_lf = power_lb = AUTO_GRAB_TURN_POWER; 
               power_rf = power_rb = 0.0; 
            } else if(auto_grab_move_==3) { // shift left
               power_lf = power_rf = -AUTO_GRAB_SHIFT_POWER; 
               //power_lb = power_rb = +AUTO_GRAB_SHIFT_POWER; 
               power_lb = +AUTO_GRAB_SHIFT_POWER; 
               power_rb = MECANUM_RIGHT_BACK_SCALE*AUTO_GRAB_SHIFT_POWER; 
            } else if(auto_grab_move_==4) { // shift right
               power_lf = power_rf = +AUTO_GRAB_SHIFT_POWER; 
               //power_lb = power_rb = -AUTO_GRAB_SHIFT_POWER; 
               power_lb = -AUTO_GRAB_SHIFT_POWER; 
               power_rb = -MECANUM_RIGHT_BACK_SCALE*AUTO_GRAB_SHIFT_POWER; 
            }
         }
      }

      /// Set power values for all motors after clipping
      power_lf = Range.clip(power_lf, -1, 1);
      power_lb = Range.clip(power_lb, -1, 1);
      power_rf = Range.clip(power_rf, -1, 1);
      power_rb = Range.clip(power_rb, -1, 1); 

      if( USE_PAD1_FOR_RELIC && relic_mode_ ) {  // no move
         power_lf = power_lb = power_rf = power_rb = 0.0; 
      } 
      if( USE_RELIC && low_sen_drive_ && RELIC_DRIVE_FACTOR>0.0 && RELIC_DRIVE_FACTOR<1.0 ) {  // scale down drive for grabbing relic
         if( USE_LOW_SEN_DRIVE_FOR_RELIC ) {
           // already handled by scaleDrivePowerLowSensitivity 
         } else {
            power_lf *= RELIC_DRIVE_FACTOR; 
            power_lb *= RELIC_DRIVE_FACTOR; 
            power_rf *= RELIC_DRIVE_FACTOR; 
            power_rb *= RELIC_DRIVE_FACTOR; 
         }
      }

      motorRF_.setPower(power_rf);
      motorRB_.setPower(power_rb);
      motorLF_.setPower(power_lf);
      motorLB_.setPower(power_lb);
      

      if( USE_GLYPH_WHEELS && !relic_mode_ ) {
         double power_gl = 0.0;
         double rsy1 = -gamepad1.right_stick_y;
         if( power_gl==0.0 && USE_PAD2_JOYSTICKS_FOR_CLAW ) {
            if( USE_INTAKE_SERVO ) {
               if( (rb2_cnt_%2) == 0 ) {
                  power_gl = -GLYPH_LEFT_PUSH_POWER;  // suck by dflt
               } else if( (rb2_cnt_%2) == 1 ) {
                  power_gl = GLYPH_LEFT_PUSH_POWER;  // push
               } 
            } else { 
               // no intake servo, use right bumper to turn on/off the flywheel; OFF by dflt
               if( (rb2_cnt_%2) == 1 ) {
                  power_gl = GLYPH_LEFT_PUSH_POWER;  // push
               }
            }
         }
         if( power_gl==0.0 ) {
            if( rsy1>0.8 ) {
               power_gl = GLYPH_LEFT_PUSH_POWER; 
            } else if( rsy1<-0.8 ) {
               power_gl = -GLYPH_LEFT_PUSH_POWER; 
            }
         }

         if( USE_INTAKE_SERVO ) {
            if( USE_INTAKE_SERVO_CR ) {
               /// Use CR servo to avoid damaging servos, and rubberband to hold flywheel position for intake
               //servo_left_intake_pos_ = SERVO_LEFT_INTAKE_OPEN_CR; 
               //servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_OPEN_CR; 
               double diff_t = curr_time_ - last_intake_fold_time_; 
               if( USE_INTAKE_MAG_SWITCH ) {
                  mag_intake_left_curr_state_ = mag_intake_left_.getState(); 
                  mag_intake_right_curr_state_ = mag_intake_right_.getState(); 
               }
               if( USE_INTAKE_TOUCH ) {
                  touch_intake_left_curr_state_ = touch_intake_left_.getState(); 
                  touch_intake_right_curr_state_ = touch_intake_right_.getState(); 
               }

               if( (lb2_cnt_%2) != 0 ) {
                  // fold them back when shelfing glyphs
                  if( intake_curr_state_!=INTAKE_SHELF ) { 
                     intake_prev_state_ = intake_curr_state_; 
                     intake_state_change_time_ = curr_time_; 
                  }
                  intake_curr_state_ = INTAKE_SHELF ;
                  //if( (curr_time_-intake_state_change_time_)<INTAKE_CR_SERVO_TURN_TIME ) {  // FIXME
                     servo_left_intake_pos_ = SERVO_LEFT_INTAKE_SHELF_CR; 
                     servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_SHELF_CR; 
                  //} else {
                  //   servo_left_intake_pos_ = CR_SERVO_STOP; 
                  //   servo_right_intake_pos_ = CR_SERVO_STOP;
                  //}
                  power_gl = 0.0; 
                  rb2_cnt_ = 0;
                  last_intake_fold_time_ = curr_time_; 
                  mag_intake_left_on_ = false; 
                  mag_intake_right_on_ = false; 
                  intake_arm_extended_ = true;

               } else if( stack_by_b ) {
                  // open intake
                  power_gl = 0.0; 

                  if( USE_INTAKE_MAG_SWITCH ) {
                     // open the intake up a little to avoid interfering with the claw
                     if( !mag_intake_left_curr_state_ ) {
                        servo_left_intake_pos_ = SERVO_LEFT_INTAKE_SHELF_CR; 
                     } else {
                        servo_left_intake_pos_ = CR_SERVO_STOP; 
                     }
                     if( !mag_intake_right_curr_state_ ) {
                        servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_SHELF_CR; 
                     } else {
                        servo_right_intake_pos_ = CR_SERVO_STOP; 
                     }
                  }
                  lb2_cnt_ = 0;
                  rb2_cnt_ = 0;
               } else if( power_gl==GLYPH_LEFT_PUSH_POWER ) {
                  if( intake_curr_state_!=INTAKE_PUSH ) { 
                     intake_prev_state_ = intake_curr_state_; 
                     intake_state_change_time_ = curr_time_; 
                  }
                  intake_curr_state_ = INTAKE_PUSH;
                  // pushing unwanted glyphs 
                  if( USE_INTAKE_MAG_SWITCH ) {
                     /*
                     if( !mag_intake_left_on_ && mag_intake_left_curr_state_ ) {  // HI, insufficient magnetic field
                        servo_left_intake_pos_ = 1.0;
                        //servo_right_intake_pos_ = 1-servo_left_intake_pos_;
                     } else {
                        mag_intake_left_on_ = true;
                        servo_left_intake_pos_ = SERVO_LEFT_INTAKE_GRAB_CR; 
                        //servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_GRAB_CR; 
                     }

                     if( !mag_intake_right_on_ && mag_intake_right_curr_state_ ) {  // HI, insufficient magnetic field
                        servo_right_intake_pos_ = 0.0; 
                     } else {
                        mag_intake_right_on_ = true;
                        servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_GRAB_CR; 
                     }
                     */
                     // open the intake up a little for pushing
                     if( !mag_intake_left_curr_state_ ) {
                        servo_left_intake_pos_ = SERVO_LEFT_INTAKE_SHELF_CR; 
                     } else {
                        servo_left_intake_pos_ = CR_SERVO_STOP; 
                     }
                     if( !mag_intake_right_curr_state_ ) {
                        servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_SHELF_CR; 
                     } else {
                        servo_right_intake_pos_ = CR_SERVO_STOP; 
                     }
                  } else {
                     if( last_intake_fold_time_>0 && (curr_time_-last_intake_fold_time_)<INTAKE_SERVO_CR_REC_TIME ) { 
                        // turn servo to help it go back to OPEN position
                        //servo_left_intake_pos_ = SERVO_LEFT_INTAKE_INIT_CR; 
                        //servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_INIT_CR; 
                        servo_left_intake_pos_ = 1.0;
                        servo_right_intake_pos_ = 1-servo_left_intake_pos_;
                     } else {
                        servo_left_intake_pos_ = SERVO_LEFT_INTAKE_OPEN_CR; 
                        servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_OPEN_CR; 
                     }
                  }
                  lb2_cnt_ = 0;
               } else if( power_gl==-GLYPH_LEFT_PUSH_POWER ) {  
                  // pulling glyph in, sucking
                  if( intake_curr_state_!=INTAKE_PULL ) { 
                     intake_prev_state_ = intake_curr_state_; 
                     intake_state_change_time_ = curr_time_; 
                  }
                  intake_curr_state_ = INTAKE_PULL;

                  if( ENABLE_AUTO_GRAB && INTAKE_AUTO_GRAB )  {
                     glyph_aligned_ = isGlyphAligned(); 
                  }

                  if( USE_INTAKE_MAG_SWITCH ) {
                     if( !mag_intake_left_on_ && mag_intake_left_curr_state_ ) {  
                        // not aligned, keep turning
                        servo_left_intake_pos_ = SERVO_LEFT_INTAKE_INIT_CR; 
                     } else {
                        if( USE_INTAKE_MAG_HOLD_POS ) { 
                           if( !mag_intake_left_curr_state_ && mag_intake_left_last_state_ ) {           
                              // entering
                              servo_left_intake_pos_ = CR_SERVO_STOP; 
                              mag_intake_left_on_ = true;
                              mag_intake_left_trig_time_ = curr_time_; 
                           } else if( !mag_intake_left_curr_state_ && !mag_intake_left_last_state_ ) {   
                              // staying
                              servo_left_intake_pos_ = CR_SERVO_STOP; 
                           } else if( mag_intake_left_curr_state_ && !mag_intake_left_last_state_ ) {    
                              // leaving, which way? 
                              // Assuming that it's pushed outward by glyph, try to turn inward
                              servo_left_intake_pos_ = SERVO_LEFT_INTAKE_INIT_CR; 
                           } else {
                              // both last and curr states are high
                              // Assuming that it's pushed outward by glyph, try to turn inward
                              servo_left_intake_pos_ = SERVO_LEFT_INTAKE_INIT_CR; 
                           }
                        } else {
                           // magnetic switch triggered, stop
                           mag_intake_left_on_ = true;
                           servo_left_intake_pos_ = SERVO_LEFT_INTAKE_GRAB_CR; 
                        }
                     }

                     if( !mag_intake_right_on_ && mag_intake_right_curr_state_ ) {
                        // not aligned, keep turning inward
                        servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_INIT_CR;
                     } else {
                        if( USE_INTAKE_MAG_HOLD_POS ) { 
                           if( !mag_intake_right_curr_state_ && mag_intake_right_last_state_ ) {           
                              // entering
                              servo_right_intake_pos_ = CR_SERVO_STOP;
                              mag_intake_right_on_ = true;
                              mag_intake_right_trig_time_ = curr_time_; 
                           } else if( !mag_intake_right_curr_state_ && !mag_intake_right_last_state_ ) {   
                              // staying
                              servo_right_intake_pos_ = CR_SERVO_STOP;
                           } else if( mag_intake_right_curr_state_ && !mag_intake_right_last_state_ ) {    
                              // leaving, which way? 
                              // Assuming that it's pushed outward by glyph, try to turn inward
                              servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_INIT_CR;
                           } else {
                              // both last and curr states are high
                              // Assuming that it's pushed outward by glyph, try to turn inward
                              servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_INIT_CR;
                           }
                        } else {
                           // magnetic switch triggered, stop
                           mag_intake_right_on_ = true;
                           servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_GRAB_CR; 
                        }
                     }
                  } else {
                     if( last_intake_fold_time_>0 && (curr_time_-last_intake_fold_time_)<INTAKE_SERVO_CR_REC_TIME ) {
                        // turn servo to help it go back to OPEN position
                        servo_left_intake_pos_ = 1.0;
                        servo_right_intake_pos_ = 1-servo_left_intake_pos_;
                     } else {
                        servo_left_intake_pos_ = SERVO_LEFT_INTAKE_GRAB_CR; 
                        servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_GRAB_CR; 
                     }
                  }
                  lb2_cnt_ = 0;
               }

               if( USE_PAD2_LEFT_JOYSTICKS_FOR_INTAKE && power_gl==-GLYPH_LEFT_PUSH_POWER ) {  // sucking
                  double lsy2 = -gamepad2.left_stick_y;
                  double lsy2_threshold = 0.3;
                  double max_change = 0.10;
                  if(  lsy2>lsy2_threshold ) {  
                     // open wider
                     double r = (lsy2-lsy2_threshold)/(1.0-lsy2_threshold);
                     servo_left_intake_pos_ = 0.5 - r*0.5;
                     servo_right_intake_pos_ = 1-servo_left_intake_pos_; 
                  } else if( lsy2<-lsy2_threshold ) {
                     // reduce opening
                     double r = (lsy2+lsy2_threshold)/(1.0-lsy2_threshold);
                     servo_left_intake_pos_ = 0.5 - r*0.5;
                     servo_right_intake_pos_ = 1-servo_left_intake_pos_; 
                  } else {  
                     // dflt opening, no change
                     //servo_left_intake_pos_ = SERVO_LEFT_INTAKE_GRAB; 
                     //servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_GRAB; 
                  }
               }

               if( curr_time_<2.0 || !intake_arm_extended_ ) power_gl = 0.0;  // FIXME for debug
               //if( intake_curr_state_ == INTAKE_INIT ) power_gl = 0.0;  

               if( USE_INTAKE_MAG_SWITCH ) {
                  mag_intake_left_last_state_ = mag_intake_left_curr_state_; 
                  mag_intake_right_last_state_ = mag_intake_right_curr_state_; 
               }
            } else {
               /// Use 180 servos
               servo_left_intake_pos_ = SERVO_LEFT_INTAKE_OPEN; 
               servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_OPEN; 
               if( (lb2_cnt_%2) != 0 ) {
                  // fold them back when shelfing glyphs
                  servo_left_intake_pos_ = SERVO_LEFT_INTAKE_SHELF; 
                  servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_SHELF; 
                  power_gl = 0.0; 
                  rb2_cnt_ = 0;
               } else if( stack_by_b ) {
                  // open in take
                  power_gl = 0.0; 
                  lb2_cnt_ = 0;
                  rb2_cnt_ = 0;
               } else if( power_gl==GLYPH_LEFT_PUSH_POWER ) {  
                  // pushing unwanted glyphs 
                  servo_left_intake_pos_ = SERVO_LEFT_INTAKE_OPEN; 
                  servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_OPEN; 
                  lb2_cnt_ = 0;
               } else if( power_gl==-GLYPH_LEFT_PUSH_POWER ) {  
                  // pulling glyph in
                  servo_left_intake_pos_ = SERVO_LEFT_INTAKE_GRAB; 
                  servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_GRAB; 
                  lb2_cnt_ = 0;
               }

               if( USE_PAD2_LEFT_JOYSTICKS_FOR_INTAKE && power_gl==-GLYPH_LEFT_PUSH_POWER ) {  // sucking
                  double lsy2 = -gamepad2.left_stick_y;
                  double lsy2_threshold = 0.3;
                  double max_change = 0.10;
                  if(  lsy2>lsy2_threshold ) {  
                     // open wider
                     double r = (lsy2-lsy2_threshold)/(1.0-lsy2_threshold);
                     servo_left_intake_pos_ = SERVO_LEFT_INTAKE_GRAB - r*max_change; 
                     servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_GRAB + r*max_change; 
                  } else if( lsy2<-lsy2_threshold ) {
                     // reduce opening
                     double r = (lsy2+lsy2_threshold)/(1.0-lsy2_threshold);
                     servo_left_intake_pos_ = SERVO_LEFT_INTAKE_GRAB - r*max_change; 
                     servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_GRAB + r*max_change; 
                  } else {  
                     // dflt opening
                     servo_left_intake_pos_ = SERVO_LEFT_INTAKE_GRAB; 
                     servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_GRAB; 
                  }
               }
            }

            if( USE_INTAKE_SERVO_CR && USE_INTAKE_TOUCH ) {
               if( intake_curr_state_ != INTAKE_INIT ) {
                  if( !touch_intake_left_curr_state_ ) { // left arm caved, push it outward
                     servo_left_intake_pos_ = SERVO_LEFT_INTAKE_SHELF_CR; 
                     touch_intake_left_trig_time_ = curr_time_; 
                  } else if( INTAKE_TOUCH_PUSH_TIME>0 && (curr_time_-touch_intake_left_trig_time_)<INTAKE_TOUCH_PUSH_TIME ) {
                     servo_left_intake_pos_ = SERVO_LEFT_INTAKE_SHELF_CR; 
                  }

                  if( !touch_intake_right_curr_state_ ) { // right arm caved, push it outward
                     servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_SHELF_CR; 
                     touch_intake_right_trig_time_ = curr_time_; 
                  } else if( INTAKE_TOUCH_PUSH_TIME>0 && (curr_time_-touch_intake_right_trig_time_)<INTAKE_TOUCH_PUSH_TIME ) {
                     servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_SHELF_CR; 
                  }
                  
               }
            } 

            servo_left_intake_pos_ = Range.clip(servo_left_intake_pos_,0,1);
            ///servo_left_intake_pos_ = Range.clip(servo_left_intake_pos_,SERVO_LEFT_INTAKE_SHELF,SERVO_LEFT_INTAKE_INIT);
            servo_left_intake_.setPosition( servo_left_intake_pos_ ); 
            servo_right_intake_pos_ = Range.clip(servo_right_intake_pos_,0,1);
            //servo_right_intake_pos_ = Range.clip(servo_right_intake_pos_,SERVO_RIGHT_INTAKE_INIT,SERVO_RIGHT_INTAKE_SHELF);
            if(USE_HITEC_DIGITAL) { servo_right_intake_pos_ = Range.clip(servo_right_intake_pos_,0.1,0.9); }
            servo_right_intake_.setPosition( servo_right_intake_pos_ ); 
         }

         power_gl = Range.clip(power_gl, -1, 1);
         motor_glyph_left_.setPower(power_gl);
         motor_glyph_right_.setPower(-power_gl); 
      } 

      /// Use digital pad to control lift and claw
      boolean manual_claw_control = false;
      if( gamepad1.dpad_up || gamepad2.dpad_up ) { // raise, auto-close claw
         power_lift = 1.0;
         servo_left_jaw_pos_ = LEFT_JAW_CLOSE_HOLD;
         servo_right_jaw_pos_ = RIGHT_JAW_CLOSE_HOLD;
         servo_left_jaw2_pos_ = LEFT_JAW_CLOSE_HOLD;
         servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE_HOLD;
         last_jaw_close_time_ = curr_time_; 
         manual_claw_control = true;
         curr_push_start_enc_ = 0; 
      } else if( gamepad1.dpad_down || gamepad2.dpad_down) {  // lower
         power_lift = LIFT_DOWN_POWER;
         manual_claw_control = true;
         curr_push_start_enc_ = 0; 
      }
      if( gamepad1.dpad_right || gamepad2.dpad_right) {  // open claw
         if( !USE_X_TO_SHIFT_CLAW || !(gamepad1.x || gamepad2.x) ) {  // open claw
            servo_left_jaw_pos_ = LEFT_JAW_OPEN;
            servo_right_jaw_pos_ = RIGHT_JAW_OPEN;
            servo_left_jaw2_pos_ = LEFT_JAW_OPEN;
            servo_right_jaw2_pos_ = RIGHT_JAW_OPEN;

            last_jaw_open_time_ = curr_time_; 

            if( AUTO_LOWER_WHEN_PUSH && curr_push_start_enc_==0 ) { curr_push_start_enc_ = motorLift_.getCurrentPosition(); }
         } else { // shift right
            servo_left_jaw_pos_ = LEFT_JAW_CLOSE;
            servo_right_jaw_pos_ = RIGHT_JAW_OPEN_SLOW;
            servo_left_jaw2_pos_ = LEFT_JAW_CLOSE;
            servo_right_jaw2_pos_ = RIGHT_JAW_OPEN_SLOW;
         }
         manual_claw_control = true;
      } else if( gamepad1.dpad_left || gamepad2.dpad_left ) {  // close claw
         if( !USE_X_TO_SHIFT_CLAW || !(gamepad1.x || gamepad2.x) ) {  // open claw
            servo_left_jaw_pos_ = LEFT_JAW_CLOSE;
            servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
            servo_left_jaw2_pos_ = LEFT_JAW_CLOSE;
            servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE;

            last_jaw_close_time_ = curr_time_; 
         } else { // shift left
            servo_left_jaw_pos_ = LEFT_JAW_OPEN_SLOW;
            servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
            servo_left_jaw2_pos_ = LEFT_JAW_OPEN_SLOW;
            servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE;

         }
         manual_claw_control = true;
      } else if ( last_stacking_time_>0 && curr_time_<last_stacking_time_+STACK_JAW_CLOSE_TIME ) {
         // Close for 1sec after stacking
         servo_left_jaw_pos_ = LEFT_JAW_CLOSE;
         servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
         servo_left_jaw2_pos_ = LEFT_JAW_CLOSE;
         servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE;
         last_b1_b2_time_ = 0.0; 

      } else if( last_jaw_close_time_>last_jaw_open_time_ ) {  // close claw hold
         servo_left_jaw_pos_ = LEFT_JAW_CLOSE_HOLD;
         servo_right_jaw_pos_ = RIGHT_JAW_CLOSE_HOLD;
         servo_left_jaw2_pos_ = LEFT_JAW_CLOSE_HOLD;
         servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE_HOLD;
      } 

      if( USE_LIFT && !relic_mode_ ) {
         if( USE_ENCODER_FOR_TELEOP && MAX_LIFT_ENC_COUNT>0 ) {
            if( manual_claw_control ) { // 
               x1_cnt_ = 0; x2_cnt_ = 0; y1_cnt_ = 0; y2_cnt_ = 0; 
            }

            int lift_enc = motorLift_.getCurrentPosition(); 

            boolean lift_glyph_above6 = ((y1_cnt_%2)==1 || (y2_cnt_%2)==1);  // use Y to lift glyph ~8in off the ground for stacking
            //boolean lift_glyph_above12 = false ;  // disable
            boolean lift_glyph_above12 = ((x1_cnt_%2)==1 || (x2_cnt_%2)==1); 
            boolean auto_reset_grabber = false; 
            if(USE_QUICK_RESET) {
              if( gamepad1.right_trigger>0.5 || gamepad2.right_trigger>0.5 )  {
                 auto_reset_grabber = true; 
                 last_reset_time_ = curr_time_; 
              } else if( manual_claw_control ) { // override by manual control
                 last_reset_time_ = 0.0;
              } else if( curr_time_<last_reset_time_+QUICK_RESET_TIME ) {
                 auto_reset_grabber = true; 
              }
            }
            if( auto_reset_grabber ) {
               if( lift_enc>MIN_LIFT_ENC_COUNT ) {
                  power_lift = LIFT_DOWN_POWER;
               } else {
                  power_lift = 0.0; 
               }
               servo_left_jaw_pos_ = LEFT_JAW_OPEN;
               servo_right_jaw_pos_ = RIGHT_JAW_OPEN;
               servo_left_jaw2_pos_ = LEFT_JAW_OPEN;
               servo_right_jaw2_pos_ = RIGHT_JAW_OPEN;
               servo_pusher_pos_ = PUSHER_PULL;   
               last_push_time_ = 0; 
               servo_kicker_pos_ = KICKER_DOWN; 

               x1_cnt_ = 0; x2_cnt_ = 0; y1_cnt_ = 0; y2_cnt_ = 0; 
            } else if( gamepad1.a || gamepad2.a) {
               if( lift_enc>LIFT_ENC_COUNT_PER_GLYPH/3 ) {   // ~2in off
                  power_lift=0.0; 
                  //power_lift=MIN_LIFT_HOLD_POWER; 
               } else {
                  power_lift=1.0; 
               }
               servo_left_jaw_pos_ = LEFT_JAW_CLOSE;
               servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
               servo_left_jaw2_pos_ = LEFT_JAW_CLOSE;
               servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE;

               last_jaw_close_time_ = curr_time_; 
               x1_cnt_ = 0; x2_cnt_ = 0; y1_cnt_ = 0; y2_cnt_ = 0; 
            //} else if ( gamepad1.b || gamepad2.b ) {
            } else if ( stack_by_b ) {
               if( STACK_DROP_DELAY>0.0 && last_b1_b2_time_>0.0 && (curr_time_-last_b1_b2_time_)<STACK_DROP_DELAY ) {
                  //power_lift = 0.0; 
                  power_lift=MIN_LIFT_HOLD_POWER; 
               } else if( lift_enc>MIN_LIFT_ENC_COUNT ) {
                  power_lift = LIFT_DOWN_POWER;
               } else {
                  power_lift = 0.0; 
               }
               servo_left_jaw_pos_ = LEFT_JAW_OPEN;
               servo_right_jaw_pos_ = RIGHT_JAW_OPEN;
               servo_left_jaw2_pos_ = LEFT_JAW_OPEN;
               servo_right_jaw2_pos_ = RIGHT_JAW_OPEN;
               last_stacking_time_ = curr_time_; 

               x1_cnt_ = 0; x2_cnt_ = 0; y1_cnt_ = 0; y2_cnt_ = 0; 
            } else if( !manual_claw_control && lift_glyph_above12 ) {
               double tg_enc = LIFT_ENC_COUNT_PER_GLYPH*7/3;  
               if( lift_enc>tg_enc ) {
                  power_lift=0.0; 
               } else if( lift_enc>tg_enc-LIFT_DIP_ENC_COUNT) {
                  power_lift=MIN_LIFT_HOLD_POWER; 
               } else {
                  power_lift=1.0; 
               }
               servo_left_jaw_pos_ = LEFT_JAW_CLOSE_HOLD;
               servo_right_jaw_pos_ = RIGHT_JAW_CLOSE_HOLD;
               servo_left_jaw2_pos_ = LEFT_JAW_CLOSE_HOLD;
               servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE_HOLD;

               y1_cnt_ = 0; y2_cnt_ = 0; 
            } else if( !manual_claw_control && lift_glyph_above6 ) {
               double tg_enc = LIFT_ENC_COUNT_PER_GLYPH*4/3;  
               if( lift_enc>tg_enc ) {
                  power_lift=0.0; 
               } else if( lift_enc>tg_enc-LIFT_DIP_ENC_COUNT ) {
                  power_lift=MIN_LIFT_HOLD_POWER; 
               } else {
                  power_lift=1.0; 
               }
               servo_left_jaw_pos_ = LEFT_JAW_CLOSE_HOLD;
               servo_right_jaw_pos_ = RIGHT_JAW_CLOSE_HOLD;
               servo_left_jaw2_pos_ = LEFT_JAW_CLOSE_HOLD;
               servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE_HOLD;

               x1_cnt_ = 0; x2_cnt_ = 0; 
            } 

            if( AUTO_LIFT_AFTER_GRAB && !manual_claw_control && last_stacking_time_>0 && power_lift==0.0 ) {
               double dt = curr_time_-last_stacking_time_; 
               if( dt>STACK_JAW_CLOSE_TIME && dt<2*STACK_JAW_CLOSE_TIME ) {
                  double tg_enc = LIFT_ENC_COUNT_PER_GLYPH/6;     // 1in off the ground
                  if( lift_enc>tg_enc ) {
                     power_lift=0.0; 
                  } else if( lift_enc>tg_enc-LIFT_DIP_ENC_COUNT ) {
                     power_lift=MIN_LIFT_HOLD_POWER; 
                  } else {
                     power_lift=1.0; 
                  }
               }
            }

            if( AUTO_LOWER_WHEN_PUSH && curr_push_start_enc_>0 ) {
               if( !manual_claw_control && power_lift==0.0 && last_push_time_>0 ) {
                  //int n_glyph = (int)(curr_push_start_enc_/LIFT_ENC_COUNT_PER_GLYPH); 
                  //double tg_enc = n_glyph * LIFT_ENC_COUNT_PER_GLYPH;
                  double tg_enc = curr_push_start_enc_-LIFT_ENC_COUNT_PER_GLYPH/3;  // drop 2 inches
                  if( tg_enc < MIN_LIFT_ENC_COUNT ) tg_enc = MIN_LIFT_ENC_COUNT; 
                  if( lift_enc>tg_enc ) {
                     power_lift = LIFT_DOWN_POWER;    // lower the lift to align with the bottom of the targeted height
                  }
               }
               if( last_push_time_>0 && curr_time_>(last_push_time_+1.0) ) curr_push_start_enc_ = 0;  // 1sec timeout after pushing start
            }

            //boolean override = gamepad1.y; 
            //boolean override_lift_power = (gamepad1.right_bumper||gamepad2.right_bumper);
            boolean override_lift_power = (gamepad2.right_bumper);
            if( !override_lift_power ) {
               if( power_lift>0.0 && lift_enc>MAX_LIFT_ENC_COUNT ) {  // prevent over-extending
                  power_lift = Range.clip(power_lift,-MIN_LIFT_POWER,MIN_LIFT_POWER); 
               } else if( power_lift<0.0 && lift_enc<MIN_LIFT_ENC_COUNT ) {   // prevent reverse
                  power_lift = Range.clip(power_lift,-MIN_LIFT_POWER,MIN_LIFT_POWER); 
               } 
            }

            if( AUTO_LOWER_WHEN_PUSH && power_lift>0 ) curr_push_start_enc_=0; 
         }
         motorLift_.setPower(power_lift);

         if( !manual_claw_control &&  ENABLE_AUTO_GRAB && INTAKE_AUTO_GRAB && glyph_aligned_ )  {
            // automatically grab the aligned glyph
            servo_left_jaw_pos_ = LEFT_JAW_CLOSE;
            servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
         }
      }

      if( power_lift<0.0 ) {
         servo_kicker_pos_ = KICKER_DOWN;   // keep pusher down when lowering lift
         servo_pusher_pos_ = PUSHER_PULL;   // pull pusher back when lowering lift
         last_push_time_ = 0; 
      //} else if( gamepad1.right_bumper || gamepad2.right_bumper ) {
      } else if( gamepad1.right_bumper || (!USE_PAD2_JOYSTICKS_FOR_CLAW && gamepad2.right_bumper) ) {
         servo_kicker_pos_ = KICKER_UP; 
         servo_pusher_pos_ = PUSHER_PUSH; 
         last_push_time_ = curr_time_; 
      //} else if( gamepad1.left_bumper || gamepad2.left_bumper ) {
      } else if( gamepad1.left_bumper || (!USE_PAD2_JOYSTICKS_FOR_CLAW && gamepad2.left_bumper) ) {
         servo_pusher_pos_ = PUSHER_PULL;   
         last_push_time_ = 0; 
      } else {
         servo_kicker_pos_ = KICKER_DOWN; 
         servo_pusher_pos_ = CR_SERVO_STOP; 
      }

      /// Driver2 use left/right joystick to control bottom/right claw
      //boolean use_pad2_claw_control = USE_PAD2_JOYSTICKS_FOR_CLAW;
      if( USE_PAD2_JOYSTICKS_FOR_CLAW && !relic_mode_ ) {
         double lsy2 = 0, lsx2 = 0, rsy2 = 0, rsx2 = 0;
         lsy2 = -gamepad2.left_stick_y;
         lsx2 = gamepad2.left_stick_x;
         rsy2 = -gamepad2.right_stick_y;
         rsx2 = gamepad2.right_stick_x;

         if( !USE_PAD2_LEFT_JOYSTICKS_FOR_INTAKE && lsy2>0.6 ) {
            servo_pusher_pos_ = PUSHER_PUSH; 
            last_push_time_ = curr_time_; 
            servo_left_jaw2_pos_ = LEFT_JAW_OPEN;
            servo_right_jaw2_pos_ = RIGHT_JAW_OPEN;
         } else if( !USE_PAD2_LEFT_JOYSTICKS_FOR_INTAKE && lsy2<-0.6) {
            servo_pusher_pos_ = PUSHER_PULL; 
            last_push_time_ = 0; 
            servo_left_jaw2_pos_ = LEFT_JAW_OPEN;
            servo_right_jaw2_pos_ = RIGHT_JAW_OPEN;
         } else if( lsx2>0.6 ) {  // shift left
            servo_pusher_pos_ = CR_SERVO_STOP; 
            servo_left_jaw2_pos_ = LEFT_JAW_OPEN_SLOW;
            servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE;
         } else if( lsx2<-0.6 ) {  // shift right
            servo_pusher_pos_ = CR_SERVO_STOP; 
            servo_left_jaw2_pos_ = LEFT_JAW_CLOSE;
            servo_right_jaw2_pos_ = RIGHT_JAW_OPEN_SLOW;
         }

         if( rsy2>0.6 ) {
            servo_pusher_pos_ = PUSHER_PUSH; 
            last_push_time_ = curr_time_; 
            servo_left_jaw_pos_ = LEFT_JAW_OPEN;
            servo_right_jaw_pos_ = RIGHT_JAW_OPEN;
         } else if( rsy2<-0.6) {
            servo_pusher_pos_ = PUSHER_PULL; 
            last_push_time_ = 0; 
            servo_left_jaw_pos_ = LEFT_JAW_OPEN;
            servo_right_jaw_pos_ = RIGHT_JAW_OPEN;
         } else if( rsx2>0.6 ) {  // shift left
            servo_pusher_pos_ = CR_SERVO_STOP; 
            servo_left_jaw_pos_ = LEFT_JAW_OPEN_SLOW;
            servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
         } else if( rsx2<-0.6 ) {  // shift right
            servo_pusher_pos_ = CR_SERVO_STOP; 
            servo_left_jaw_pos_ = LEFT_JAW_CLOSE;
            servo_right_jaw_pos_ = RIGHT_JAW_OPEN_SLOW;
         } 
      } 

      if( USE_INTAKE_SERVO_CR && curr_time_<1 ) {
         // close the jaw first to avoid caught with arms
         servo_left_jaw_pos_ = LEFT_JAW_CLOSE; 
         servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
      }
      servo_left_jaw_pos_ = Range.clip(servo_left_jaw_pos_,0,1.0);
      servo_right_jaw_pos_ = Range.clip(servo_right_jaw_pos_,0,1.0);
      servo_left_jaw2_pos_ = Range.clip(servo_left_jaw2_pos_,0,1.0);
      servo_right_jaw2_pos_ = Range.clip(servo_right_jaw2_pos_,0,1.0);
      servo_kicker_pos_ = Range.clip(servo_kicker_pos_,0,1.0);
      servo_pusher_pos_ = Range.clip(servo_pusher_pos_,0,1.0);

      if( !relic_mode_) { 
         servo_left_jaw_.setPosition(servo_left_jaw_pos_);
         servo_right_jaw_.setPosition(servo_right_jaw_pos_);
         servo_left_jaw2_.setPosition(servo_left_jaw2_pos_);
         servo_right_jaw2_.setPosition(servo_right_jaw2_pos_);

         if(USE_KICKER) servo_kicker_.setPosition(servo_kicker_pos_);
         if(USE_PUSHER) servo_pusher_.setPosition(servo_pusher_pos_);
      }

      if(USE_RELIC) {
      if( relic_mode_) {
         /// Driver2 to fetch/deliver relic
         //  Use left_joystick to extend/retract the slide
         //  Use right_joystick to lower/rise arm
         power_relic_ = 0.0; 
         boolean extending_arm = gamepad2.a; 
         boolean closing_claw = gamepad2.b; 
         boolean lifting_relic = gamepad2.x; 
         boolean delivering_relic = gamepad2.y; 
         boolean wrapping_up = gamepad2.left_bumper;   // 
         if( USE_PAD1_FOR_RELIC ) {
            extending_arm = gamepad1.a; 
            closing_claw = gamepad1.b; 
            lifting_relic = gamepad1.x; 
            delivering_relic = gamepad1.y; 
            wrapping_up = gamepad1.left_bumper;
         }

         if( extending_arm ) {
            //  Use A for initial alignment
            int relic_enc = Math.abs(motorRelic_.getCurrentPosition()); 
            if( relic_enc<MAX_RELIC_SLIDE_ENC_COUNT ) {
               power_relic_ = RELIC_EXTEND_POWER; 
            } else { 
               power_relic_ = 0.0; 
            }

            if( relic_enc>1*MAX_RELIC_SLIDE_ENC_COUNT/3 ) {
               servo_relic_arm_pos_ = SERVO_RELIC_ARM_GRAB; 
               servo_relic_claw_pos_ = SERVO_RELIC_CLAW_OPEN;
            } 
            last_relic_releasing_time_ = 0; 
            last_relic_lowering_time_ = 0; 
         } else if ( closing_claw ) {
            // close the claw
            servo_relic_claw_pos_ = SERVO_RELIC_CLAW_CLOSE;
         } else if ( lifting_relic ) {
            // retract the slide and flip the relic up
            int relic_enc = motorRelic_.getCurrentPosition(); 
            if( relic_enc>MAX_RELIC_SLIDE_ENC_COUNT*0.9 ) {
               power_relic_ = RELIC_RETRACT_POWER; 
            } else { 
               power_relic_ = 0.0; 
            }
            servo_relic_arm_pos_ = SERVO_RELIC_ARM_RISE; 
            servo_relic_claw_pos_ = SERVO_RELIC_CLAW_CLOSE;
            last_relic_releasing_time_ = 0; 
            last_relic_lowering_time_ = 0; 
         } else if ( delivering_relic ) {
            // deliver the relic
            int relic_enc = motorRelic_.getCurrentPosition(); 
            if( relic_enc<MAX_RELIC_SLIDE_ENC_COUNT ) {
               power_relic_ = RELIC_EXTEND_POWER; 
            } else { 
               power_relic_ = 0.0; 
            }
            servo_relic_claw_pos_ = SERVO_RELIC_CLAW_CLOSE;
            if( !SLOW_LOWER_RELIC ) {
               servo_relic_arm_pos_ = SERVO_RELIC_ARM_DELIVER; 
            } else {
               if( last_relic_lowering_time_==0.0 ) {
                  last_relic_lowering_time_ = (USE_PAD1_FOR_RELIC ? last_button_time_ : last_button_time2_);
               } 
               double diff_t = Math.abs(curr_time_-last_relic_lowering_time_); 
               servo_relic_arm_pos_ = SERVO_RELIC_ARM_RISE-diff_t*0.20;  // 
               if( servo_relic_arm_pos_ < SERVO_RELIC_ARM_DELIVER ) {
                  servo_relic_arm_pos_ = SERVO_RELIC_ARM_DELIVER; 
               }
            }
         } else if ( wrapping_up ) {
            // opening the claw gradually to let the relic go, and retracting the slide
            if( last_relic_releasing_time_ == 0.0 ) {
               last_relic_releasing_time_ = (USE_PAD1_FOR_RELIC ? last_button_time_ : last_button_time2_); 
            }
            double diff_t = Math.abs(curr_time_-last_relic_releasing_time_); 
            if( diff_t<0.5 ) {
               //servo_relic_claw_pos_ -= diff_t*0.01;  // 
               //servo_relic_claw_pos_ -= diff_t*RELIC_RELEASE_SPEED;  // 
               servo_relic_claw_pos_ = SERVO_RELIC_CLAW_CLOSE - diff_t*RELIC_RELEASE_SPEED;  // 
            //} else if( diff_t>1.5 ) { // 1 sec delay
            } else if( diff_t>1.0 ) {   // 0.5 sec delay
               //servo_relic_claw_pos_ -= (diff_t-1)*0.01;  // upto 0.02 per update, so max change 0.2 per sec
               servo_relic_claw_pos_ = SERVO_RELIC_CLAW_CLOSE - (diff_t-0.5)*RELIC_RELEASE_SPEED;  // upto 0.02 per update, so max change 0.2 per sec
            } else {
               // 1sec delay to let the relic self-adjust by gravity
            }
            if( servo_relic_claw_pos_<SERVO_RELIC_CLAW_RELEASE ) {
               power_relic_ = 0.2*RELIC_RETRACT_POWER; 
            } 
         }

         double lsy2 = -gamepad2.left_stick_y;
         if( USE_PAD1_FOR_RELIC ) { lsy2 = -gamepad1.left_stick_y; }
         boolean manual_relic_control = false;
         if( Math.abs(lsy2) > JOYSTICK_DEAD_ZONE ) {
            power_relic_ = lsy2; 
            power_relic_ = (double) scaleDrivePower(power_relic_, /*drive_power_f*/1.0); 
            power_relic_ = Range.clip( power_relic_, -1, 1 );
            manual_relic_control = true;
         }

         double rsy2 = -gamepad2.right_stick_y;
         if( USE_PAD1_FOR_RELIC ) { rsy2 = -gamepad1.right_stick_y; }
         if( Math.abs(rsy2) > JOYSTICK_DEAD_ZONE ) {
            boolean control_arm = ((rsb2_cnt_%2)==0) ; 
            if( USE_PAD1_FOR_RELIC ) { control_arm = (rsb1_cnt_%2)==0; }
            if( curr_time_>last_button_time2_+0.1 ) {  // update 10times per sec
               if( control_arm )  {
                  servo_relic_arm_pos_ += rsy2*0.02;   // upto 0.02 per update. So max change 0.2 per sec.
               } else {
                  servo_relic_claw_pos_ -= rsy2*0.02;  // upto 0.03 per update, so max change 0.2 per sec
               }
               last_button_time2_ = curr_time_; 
            }
         }

         power_relic_ = Range.clip(power_relic_, -1, 1);
         motorRelic_.setPower(power_relic_); 
         servo_relic_arm_pos_ = Range.clip(servo_relic_arm_pos_,0,1);
         servo_relic_arm_.setPosition( servo_relic_arm_pos_ ); 
         servo_relic_claw_pos_ = Range.clip(servo_relic_claw_pos_,0,1);
         servo_relic_claw_.setPosition( servo_relic_claw_pos_ ); 

         if( USE_INTAKE_SERVO ) {
            if( intake_curr_state_!=INTAKE_INIT ) { 
               intake_prev_state_ = intake_curr_state_; 
               intake_state_change_time_ = curr_time_; 
            }
            intake_curr_state_ = INTAKE_INIT ; 
            /// Fold the intake module into initial position for better protection
            if( curr_time_<last_relic_start_time_+2.0 ) {
               servo_left_jaw_pos_ = LEFT_JAW_CLOSE;
               servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
            } else {
               servo_left_jaw_pos_ = CR_SERVO_STOP;
               servo_right_jaw_pos_ = CR_SERVO_STOP;
            }
            servo_left_jaw_.setPosition(servo_left_jaw_pos_);
            servo_right_jaw_.setPosition(servo_right_jaw_pos_);

            if( USE_INTAKE_SERVO_CR ) {
               if( (curr_time_-intake_state_change_time_)<INTAKE_CR_SERVO_TURN_TIME ) {
                  servo_left_intake_pos_ = SERVO_LEFT_INTAKE_INIT_CR; 
                  servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_INIT_CR; 
               } else {
                  servo_left_intake_pos_ = CR_SERVO_STOP; 
                  servo_right_intake_pos_ = CR_SERVO_STOP;
               }
            } else {
               // fold intake motor in for better protection
               servo_left_intake_pos_ = SERVO_LEFT_INTAKE_INIT; 
               servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_INIT; 
            }
            servo_left_intake_.setPosition( servo_left_intake_pos_ ); 
            servo_right_intake_.setPosition( servo_right_intake_pos_ ); 
            motor_glyph_left_.setPower(0);
            motor_glyph_right_.setPower(0); 
         }

         motorLift_.setPower(0.0);    // keep lift still
      } else {
         motorRelic_.setPower(0.0);   // driving
      }
      }


      /// Hold jewel module position
      servo_jewel_.setPosition(SERVO_JEWEL_INIT);
      servo_knock_.setPosition(SERVO_KNOCK_INIT);

      // Send telemetry data back to driver station for debugging
      boolean show_wheel_power = true;
      boolean show_glyph_wheel_power = true;
      boolean show_intake_servo = true;
      boolean show_lift_pos = true;
      boolean show_sweeper_pos = false;
      boolean show_heading = false;
      boolean show_voltage = false;
      boolean show_mr_range = false;
      boolean show_rev_range = false;
      boolean show_auto_grab = true;

      telemetry.addData("GiftGears(OR)", "Team:"+(isRedTeam()?"RED":"BLUE")+", Time:"+String.format("%.2f",curr_time_));

      if( show_wheel_power )  telemetry.addData("WheelPower", "Factor="+String.format("%.2f",drive_power_f)+"LF/RF/LB/RB: " + String.format("%.2f", motorLF_.getPower()) + "/" + String.format("%.2f", motorRF_.getPower()) + "/" + String.format("%.2f", motorLB_.getPower()) + "/" + String.format("%.2f", motorRB_.getPower())); 
      if( USE_GLYPH_WHEELS && show_glyph_wheel_power ) telemetry.addData("GlyphWheelPower", "LEFT/RIGHT: " + String.format("%.2f", motor_glyph_left_.getPower())+"/"+String.format("%.2f", motor_glyph_right_.getPower())); 
      if( USE_INTAKE_SERVO && show_intake_servo ) telemetry.addData("IntakeServoPos", String.format("left=%.2f, right=%.2f",servo_left_intake_.getPosition(),servo_right_intake_.getPosition()));

      if( DEBUG_DRIVE_MOTORS ) {
         telemetry.addData("RightPowerRatio", String.format("%.2f", RIGHT_POWER_RATIO)); 
         telemetry.addData("EncPos", ": LF="+String.valueOf(motorLF_.getCurrentPosition())+", RF="+String.valueOf(motorRF_.getCurrentPosition())+", LB="+String.valueOf(motorLB_.getCurrentPosition())+",RB="+String.valueOf(motorRB_.getCurrentPosition()));
      }
      if( show_heading && imu_!=null ) {
         telemetry.addData("IMU", "heading: " + String.format("%.2f", getHeading())); 
      } 
      if( show_voltage ) {
         telemetry.addData("Battery", String.format("#sensors: %d, voltage: %.2fV", numVoltageSensors(), getBatteryVoltage(true)));
      }
      if( USE_LIFT && USE_ENCODER_FOR_TELEOP && show_lift_pos ) {
         telemetry.addData("Lift EncPose", ": "+String.valueOf(motorLift_.getCurrentPosition())+", Power="+String.valueOf(motorLift_.getPower())+", curr_drop_pos="+String.valueOf(curr_push_start_enc_)); 
      } 
      if( USE_RELIC ) telemetry.addData("Relic: ", "Mode="+String.valueOf(relic_mode_)+", Motor Enc="+String.valueOf(motorRelic_.getCurrentPosition())+", Power="+String.valueOf(motorRelic_.getPower())+"; ArmPos="+String.valueOf(servo_relic_arm_pos_)+", ClawPos="+String.valueOf(servo_relic_claw_pos_)); 

      if( show_mr_range && mr_range_!=null ) {
         telemetry.addData("MRRangeSensor", String.format("ultra/opt/dist=%4d/%.2f/%.2f",mr_range_.rawUltrasonic(),mr_range_.cmOptical(),mr_range_.getDistance(DistanceUnit.CM)));
      }

      if( ENABLE_AUTO_GRAB && show_rev_range ) {
         telemetry.addData("REV_RANGE_LEFT", String.format("R/G/B/Alpha=%d/%d/%d/%d, Distance=%.2fcm",rev_range_left_.red(),rev_range_left_.green(),rev_range_left_.blue(),rev_range_left_.alpha(),rev_range_left_.getDistance(DistanceUnit.CM)));      
         telemetry.addData("REV_RANGE_RIGHT", String.format("R/G/B/Alpha=%d/%d/%d/%d, Distance=%.2fcm",rev_range_right_.red(),rev_range_right_.green(),rev_range_right_.blue(),rev_range_right_.alpha(),rev_range_right_.getDistance(DistanceUnit.CM)));      
      } 
      if( ENABLE_AUTO_GRAB && show_auto_grab ) {
         telemetry.addData("AUTO_GRAB", String.format("Aligned=%s; Init MR/Left/Right/Time/Heading=%.2f/%.2f/%.2f/%.2f/%.2f; Elapsed=%.2f, Curr MR/Left/Right/Heading=%.2f/%.2f/%.2f/%.2f; Move(Curr/Prev)=%d/%d",(glyph_aligned_?"TRUE":"FALSE"),dist0_mr_range_,dist0_left_,dist0_right_,glyph_seen_t_,glyph_seen_heading_,curr_time_-glyph_seen_t_,dist_mr_range_,dist_left_,dist_right_,heading_,auto_grab_move_,prev_auto_grab_move_)); 
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
   double scaleDrivePower(double dVal, double factor) {
      //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
      //double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};  // Y16, w/o encoder
      double[] scaleArray = {0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.27, 0.30, 0.34, 0.38, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00};  // Y17, with encoder

      // Get the corresponding index for the scaleDrivePower array.
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

      if( USE_ENCODER_FOR_TELEOP ) dScale *= ENCODER_MAX_DRIVE_POWER; 
      if( factor>0.0 && factor<=1.5) dScale *= factor; 
      return dScale;
   }

   /// Overloaded vaersion with default factor of 1.0
   double scaleDrivePower(double dVal) {
      return scaleDrivePower(dVal, 1.0); 
   } 

   /// Low sensitivity drive mode for balancing and relic
   double scaleDrivePowerLowSensitivity(double dVal, double factor) {
      //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
      //double[] scaleArray = {0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.27, 0.30, 0.34, 0.38, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00};  // Y17, with encoder
      //double[] scaleArray = {0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.27, 0.30, 0.34, 0.38, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00};  // Y17, with encoder
      double[] scaleArray = {0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36, 0.38, 0.40, 0.42, 0.44};  // Y17, 

      // Get the corresponding index for the scaleDrivePower array.
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

      if( USE_ENCODER_FOR_TELEOP ) dScale *= ENCODER_MAX_DRIVE_POWER; 
      if( factor>0.0 && factor<=1.5) dScale *= factor; 
      return dScale;
   }
   /// Scale the robot rotating power
   double scaleRotatePower(double dVal) {
      //                    { 0.0, 0.06, 0.13, 0.19, 0.25, 0.31, 0.38, 0.44, 0.50, 0.56, 0.63, 0.69, 0.75, 0.81, 0.88, 0.94, 1.00 };  // linear scale
      //double[] scaleArray = {0.0, 0.15, 0.18, 0.21, 0.24, 0.27, 0.3, 0.33, 0.36, 0.39, 0.42, 0.45, 0.48, 0.51, 0.54, 0.57, 0.60};   // Y16, w/o encoder
      //double[] scaleArray = {0.0, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26, 0.28, 0.3, 0.33, 0.36, 0.39, 0.42, 0.45, 0.48, 0.51};   // Y16, w/o encoder
      //double[] scaleArray = {0.0, 0.15, 0.18, 0.21, 0.24, 0.27, 0.3, 0.33, 0.36, 0.39, 0.42, 0.45, 0.48, 0.51, 0.54, 0.57, 0.60};   // Y17, with encoder for Mecanum 6
      double[] scaleArray = {0.0, 0.35, 0.4, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.78, 0.80, 0.85, 0.90, 0.93, 0.97, 1.0};   // Y17, with encoder for Mecanum 6

      // Get the corresponding index for the scalePower array.
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

      if( USE_ENCODER_FOR_TELEOP ) dScale *= ENCODER_MAX_ROTATE_POWER; 
      return dScale;
   }

} 
