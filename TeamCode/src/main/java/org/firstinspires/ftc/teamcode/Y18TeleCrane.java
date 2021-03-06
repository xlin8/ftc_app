/*
 * This is Program for TeleOp Run for CraneBot for FTC 2018-2019 Rover Ruckus season
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/// TeleOp Run for 
@TeleOp(name="Y18TeleCrane", group="GG")
//@Disabled
public class Y18TeleCrane extends Y18CommonCrane
{
   /// Trip control variables
   boolean crater_trip_ = true;
   boolean fast_low_sen_drive_ = false;               // fast low sensitivity drive, 1.4X speed
   boolean single_driver_ = false;

   static boolean ALLOW_FLIP_ROBOT = false;           // no need for Rover Ruckus
   static boolean MUTUAL_EXCLUSIVE_A_X = true;        // keep A/X mutual exclusive

   /// Other variables
   static final double  MIN_BUTTON_INTERVAL = 0.3;
   boolean DEBUG_DRIVE_MOTORS = false;

   boolean USE_MECANUM_WHEELS = true;
   boolean USE_MECANUM_FOR_SIDEWALK_ONLY = true;
   static final double MECANUM_RIGHT_BACK_SCALE = 1.0;    // compensate the unbalanced robot at right back wheel

   boolean USE_ENCODER_FOR_TELEOP = true;
   double ENCODER_MAX_DRIVE_POWER = 1.0;   
   double ENCODER_MAX_ROTATE_POWER = 0.3;
   double ENCODER_MAX_SIDEWALK_POWER = 0.5;   // 2018/02/09, cap Mecanum wheel sidewalk power at 0.5

   double RIGHT_POWER_RATIO = 1.00;


   static final boolean USE_LOW_SEN_DRIVE_FOR_DUMP = true;   // use low sensitivity drive mode for relic and balancing
   boolean low_sen_drive_ = false;
   boolean end_game_ = false;

   /// Manually lower arm and reset the encoder
   double arm_reset_start_time_ = 0.0;

   static final boolean AUTO_LIFT_CONTROL = true;  // automatically lower and rise the hangling lift
   static final int AUTO_LIFT_ENC = 5000 ;
   static final int AUTO_LIFT_UP_ENC = 1000 ;
   static final int LIFT_ENC_CNT_END_GAME = 10000;

   int last_crane_tg_enc_ = 0;
   double last_crane_power_ = 0.0;

   ///  Constructor
   public Y18TeleCrane() {
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
      }

      if( USE_MECANUM_WHEELS ) {
         motorLF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         motorLB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         motorRF_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
         motorRB_.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      }

      servo_lift_pin_pos_ = LIFT_PIN_PULL ;


   }

   @Override public void init_loop() {
      super.init_loop();

      if( USE_ENCODER_FOR_TELEOP ) {
         motorLF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorLB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorRF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
         motorRB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);

      } 
      if(USE_LIFT) {
         motorLift_.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorLift_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
         servo_lift_pin_pos_ = LIFT_PIN_PULL;
         servo_lift_pin_.setPosition(servo_lift_pin_pos_);
      }
      if(USE_CRANE) {
         if( RUN_CRANE_ENC_POS ) {
            motor_crane_.setMode ( DcMotor.RunMode.RUN_TO_POSITION );
            if( USE_CRANE_DOUBLE ) {
               motor_crane2_.setMode ( DcMotor.RunMode.RUN_TO_POSITION );
            }
         } else {
            motor_crane_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
            if( USE_CRANE_DOUBLE ) {
               motor_crane2_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
            }
         }
      }
      if(USE_SWEEPER) motor_sweeper_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
   }

   @Override public void loop () {
      loop_cnt_++; 
      curr_time_ = timer_.time(); 

      //servo_lift_pin_pos_ = LIFT_PIN_STOP;

      if(USE_LOW_SEN_DRIVE_FOR_DUMP) {
         if( fast_low_sen_drive_ ) {
            low_sen_drive_ = (rsb1_cnt_ % 2) == 1;
         } else {
            low_sen_drive_ = (rsb1_cnt_ % 2) == 0;
         }
      }
      end_game_ = true;
      if(single_driver_) end_game_ = ((rb1_cnt_%2)==1) ;

      power_lift_ = 0.0;

      double power_lf = 0, power_lb = 0, power_rf = 0, power_rb = 0;
      double power_sweeper = 0;
      double lsy = 0, lsx = 0, rsy = 0, rsx = 0;
      double drive_power_f = 1.0; 

      // rsy: right_stick_y ranges from -1 to 1, where -1 is full up, and 1 is full down
      // rsx: right_stick_x ranges from -1 to 1, where -1 is full left and 1 is full right
      rsy = -gamepad1.right_stick_y;
      rsx = gamepad1.right_stick_x;


      if( (curr_time_-last_button_time_) > MIN_BUTTON_INTERVAL) {
         if (gamepad1.x){
            x1_cnt_++;
            if(MUTUAL_EXCLUSIVE_A_X && x1_cnt_%2==1) { a1_cnt_=0; } 
            last_button_time_ = curr_time_;
         } else if (gamepad1.y) {
            y1_cnt_++;
            last_button_time_ = curr_time_;
         }else if (gamepad1.a) {
            a1_cnt_++;
            if(MUTUAL_EXCLUSIVE_A_X && a1_cnt_%2==1) { x1_cnt_=0; } 
            last_button_time_ = curr_time_;
         } else if (gamepad1.b) {
            b1_cnt_++;
            last_button_time_ = curr_time_;
         } else if (gamepad1.left_bumper) {
            lb1_cnt_++;
            last_button_time_ = curr_time_;
         } else if (gamepad1.right_bumper) {
            rb1_cnt_++;
            last_button_time_ = curr_time_;
         } else if (gamepad1.left_stick_button) {
            lsb1_cnt_++;
            last_button_time_ = curr_time_;
         } else if (gamepad1.right_stick_button) {
            rsb1_cnt_++;
            last_button_time_ = curr_time_;
         }
      }
      if( (curr_time_-last_button_time2_) > MIN_BUTTON_INTERVAL) {
         if (gamepad2.x){
            x2_cnt_++;
            if(MUTUAL_EXCLUSIVE_A_X && x2_cnt_%2==1) { a2_cnt_=0; } 
            last_button_time2_ = curr_time_;
         } else if (gamepad2.y) {
            y2_cnt_++;
            last_button_time2_ = curr_time_;
         }else if (gamepad2.a) {
            a2_cnt_++;
            if(MUTUAL_EXCLUSIVE_A_X && a2_cnt_%2==1) { x2_cnt_=0; } 
            last_button_time2_ = curr_time_;
         } else if (gamepad2.b) {
            b2_cnt_++;
            last_button_time2_ = curr_time_;
         } else if (gamepad2.left_bumper) {
            lb2_cnt_++;
            last_button_time2_ = curr_time_;
         } else if (gamepad2.right_bumper) {
            rb2_cnt_++;
            last_button_time2_ = curr_time_;
         } else if (gamepad2.left_stick_button) {
            lsb2_cnt_++;
            last_button_time2_ = curr_time_;
         } else if (gamepad2.right_stick_button) {
            rsb2_cnt_++;
            last_button_time2_ = curr_time_;
         }
      }

      if (USE_SERVO_ARM_HOLDER && single_driver_== false) {
         if (lsb1_cnt_%2==1) {
            servoArmHolder_.setPosition(SERVO_ARM_RELEASE_POS);
         } else {
            servoArmHolder_.setPosition(SERVO_ARM_HOLD_POS);
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

         if( USE_LOW_SEN_DRIVE_FOR_DUMP && low_sen_drive_ ) {
            if( fast_low_sen_drive_ ) {
               power_rf = (double) scaleDrivePowerLowSensitivity(power_rf,1.4);
               power_lf = (double) scaleDrivePowerLowSensitivity(power_lf,1.4);
            } else {
               power_rf = (double) scaleDrivePowerLowSensitivity(power_rf,1.0);
               power_lf = (double) scaleDrivePowerLowSensitivity(power_lf,1.0);
            }
         }

         power_rf = Range.clip(power_rf, -1, 1);
         power_lf = Range.clip(power_lf, -1, 1);

         /// LR is same as LF, RR is same as RF
         power_lb = power_lf;
         power_rb = power_rf;

      }

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

      boolean arm_raised = false;
      boolean is_dumping = false;
      if( USE_CRANE ) {
         int crane_enc = motor_crane_.getCurrentPosition();
         boolean manual_crane_control = false;
         boolean reset_arm = false;

         if( rb2_cnt_ > 0 ) {
            reset_arm = true;
            if( arm_reset_start_time_==0.0 ) {
               arm_reset_start_time_ = curr_time_;
               motor_crane_.setTargetPosition(CRANE_RESET_POS);
               motor_crane_.setPower(CRANE_RESET_POWER);
               motor_crane2_.setTargetPosition(CRANE_RESET_POS);
               motor_crane2_.setPower(CRANE_RESET_POWER);
            }

            if( curr_time_ - arm_reset_start_time_ > CRANE_RESET_TIME ) {
               motor_crane_.setPower(0.0);
               motor_crane_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               motor_crane_.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               motor_crane2_.setPower(0.0);
               motor_crane2_.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               motor_crane2_.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               rb2_cnt_ = 0;
               arm_reset_start_time_ = 0;
            }
         } else if( RUN_CRANE_ENC_POS ) {
            int crane_tg_enc =  motor_crane_.getTargetPosition();
            if( !motor_crane_.isBusy() ) {
/* Manual control FIXME
               if( gamepad1.dpad_up ) { // raise, auto-close claw
                  if( crane_enc < MAX_CRANE_ENC_COUNT ) {
                     crane_tg_enc += 200; 
                  } 
                  manual_crane_control = true;
               } else if( gamepad1.dpad_down ) {  // lower
                  if( crane_enc > 0 ) {
                     crane_tg_enc -= 200; 
                  } 
                  manual_crane_control = true;
               } 
*/
            }

            if( !manual_crane_control && ((single_driver_ && (a1_cnt_%2==1 || x1_cnt_%2==1)) || a2_cnt_%2==1 || x2_cnt_==1) ) {
               if( (single_driver_ && x1_cnt_%2 == 1) || x2_cnt_%2 == 1 ) {
                  crane_tg_enc = CRANE_COLLECT_POS; 
                  a1_cnt_ = 0;
                  if (single_driver_ == true) {
                     b1_cnt_ = 0;
                  }
                  a2_cnt_ = b2_cnt_ = 0;
                  if( AUTO_SWEEPER ) {
                     if( crane_enc > MIN_CRANE_ENC_START_SWEEPER+100 ) {
                        y1_cnt_ = 0;
                        y2_cnt_ = 0;
                     } else if( ( (single_driver_ && (y1_cnt_==0 /*|| y1_cnt_==2*/)) || y2_cnt_==0 /*|| y2_cnt_==2*/) && crane_enc<MIN_CRANE_ENC_START_SWEEPER ) {
                        y1_cnt_ = 1;
                        y2_cnt_ = 1;
                     } 
                  } else {
                     y1_cnt_ = 0;   // start with sweep in
                     y2_cnt_ = 0;   // start with sweep in
                  }
                  crane_arm_dump_start_t = 0.0; 
               } else if ( (single_driver_ && a1_cnt_%2 == 1) || a2_cnt_%2 == 1) {
                  arm_raised = true;
                  if( crane_tg_enc == CRANE_DUMP_POS ) {
                     // keep it at DUMP
                  } else {
                     crane_tg_enc = CRANE_UP_POS; 
                  }
                  x1_cnt_ = 0;
                  if (single_driver_ == true) {
                     b1_cnt_ = 0;
                  }
                  b2_cnt_ = x2_cnt_ = 0; 
                  if( (single_driver_ && y1_cnt_==1) || y2_cnt_==1 ) {
                     y1_cnt_ = 2;    // stop sweeper, and ready for reverse
                     y2_cnt_ = 2;
                  }
                  if( (single_driver_ && gamepad1.b) || gamepad2.b ) {
                     crane_tg_enc = CRANE_DUMP_POS; 
                  } else {
                     crane_tg_enc = CRANE_UP_POS; 
                  }
                  if( USE_ONE_STAGE_LIFT ) {
                     crane_tg_enc = CRANE_DUMP_POS; 
                  }
                  if( !crater_trip_ ) {
                     crane_tg_enc = CRANE_DUMP_POS_DEPOT; 
                  }
               }
            }
            double crane_pwr = CRANE_POS_POWER; 
            if( ADAPTIVE_CRANE_POWER ) {
               if( crane_tg_enc==CRANE_UP_POS || (USE_ONE_STAGE_LIFT && (crane_tg_enc==CRANE_DUMP_POS)) ) {
                  if( crane_enc<0.5*crane_tg_enc ) {
                     crane_pwr *= 2.0;   // double
                  } else if( crane_enc>0.8*crane_tg_enc ) {
                     crane_pwr /= 1.5;   // half, braking
                  } else {
                     // dflt power
                  }
               } else if( crane_tg_enc==CRANE_COLLECT_POS ) {
                  if( crane_enc<0.6*crane_tg_enc ) {
                     crane_pwr *= 2.0;   // jumpstart
                  } else if( crane_enc<0.85*crane_tg_enc ) {
                     crane_pwr *= 1.5;    // 
                  } else {
                     crane_pwr /= 1.5;    // braking
                  }
                  /*
                  if( Math.abs(crane_enc)<20 ) {
                     crane_pwr = 0.0; 
                  } else if( Math.abs(crane_enc)<0.3*CRANE_UP_POS ) {
                     crane_pwr /= 2;    // more braking
                  } else if( Math.abs(crane_enc)<0.6*CRANE_UP_POS ) {
                     crane_pwr /= 1.5;  // start braking for last half
                  } else {
                     // dflt
                  }
                  */
               } else if( crane_tg_enc==CRANE_DUMP_POS ) {
                  crane_pwr = 0.2;       // 0.4 is too fast
               }
            } 

            if( gamepad1.left_bumper || gamepad2.left_bumper ) { 
               // manually lift the arm to pass the crater
               crane_tg_enc = CRANE_COLLECT_POS + 200; 
               crane_pwr = 0.3;
            }

            crane_tg_enc = (int)(Range.clip(crane_tg_enc,0,MAX_CRANE_ENC_COUNT));
            if( last_crane_tg_enc_ != crane_tg_enc ) {
               motor_crane_.setTargetPosition(crane_tg_enc);
               if(USE_CRANE_DOUBLE) motor_crane2_.setTargetPosition(crane_tg_enc);
               last_crane_tg_enc_ = crane_tg_enc;
            }

            crane_pwr = Range.clip(crane_pwr,0.0,1.0);
            if( crane_pwr != last_crane_power_ ) {
               motor_crane_.setPower(crane_pwr);
               if(USE_CRANE_DOUBLE) motor_crane2_.setPower(crane_pwr);
               last_crane_power_ = crane_pwr;
            }

            if( USE_CRANE_DOUBLE ) {
               if( last_crane_tg_enc_ != crane_tg_enc ) {

               }
               if( last_crane_power_ != crane_pwr ) {

               }
            }

            if( USE_CRANE_ARM ) {
               if( gamepad1.b || gamepad2.b ) {
                  if( TIMED_CRANE_ARM_DUMP ) {
                     // timed smooth dumping
                     if( crane_enc>CRANE_ARM_DUMP_ENC_RATIO*crane_tg_enc ) {
                        if( crane_arm_dump_start_t == 0.0 ) {
                           crane_arm_dump_start_t = curr_time_;
                        } 
                        if( (curr_time_-crane_arm_dump_start_t) < CRANE_ARM_DUMP_TIME ) {
                           servo_crane_arm_pos_ = CRANE_ARM_DUMP_HOLD + (CRANE_ARM_DUMP-CRANE_ARM_DUMP_HOLD)*(curr_time_-crane_arm_dump_start_t)/CRANE_ARM_DUMP_TIME;  // gradually dumping
                        } else {
                           servo_crane_arm_pos_ = CRANE_ARM_DUMP;      // dump
                        }
                     } else {
                        servo_crane_arm_pos_ = CRANE_ARM_DUMP_HOLD;    // same as ARM_COLLECT for crane
                     } 
                  } else if( crane_enc>0.90*crane_tg_enc && crane_enc<1.1*crane_tg_enc  ) {
                     // manual dump 
                     servo_crane_arm_pos_ = CRANE_ARM_DUMP; 
                  } 
                  y1_cnt_=0;  // stop sweeper
                  y2_cnt_=0;  // stop sweeper
               } else { 
                  servo_crane_arm_pos_ = CRANE_ARM_COLLECT; 
               }
               is_dumping = (servo_crane_arm_pos_ == CRANE_ARM_DUMP) ;
               if( is_dumping && !crater_trip_ ) servo_crane_arm_pos_ = CRANE_ARM_DUMP_DEPOT ;
               servo_crane_arm_.setPosition(servo_crane_arm_pos_); 
            }
            if( USE_CRANE_WINCH ) { 
               servo_crane_winch_pos_ = CR_SERVO_STOP;
               if( true ) {
                  if ( (!end_game_ && !fast_low_sen_drive_ && gamepad1.dpad_up) || gamepad2.dpad_up) {
                     servo_crane_winch_pos_ = CRANE_WINCH_EXTEND;
                  } else if ( (!end_game_ && !fast_low_sen_drive_&& gamepad1.dpad_down) || gamepad2.dpad_down) {
                     servo_crane_winch_pos_ = 1 - CRANE_WINCH_EXTEND;
                  } else if ( arm_raised || crane_enc > CRANE_WINCH_HOLD_ENC) {
                     servo_crane_winch_pos_ = CRANE_WINCH_HOLD;

                  }
               }

               servo_crane_winch_pos_ = Range.clip(servo_crane_winch_pos_,0.0,1.0);
               servo_crane_winch_.setPosition(servo_crane_winch_pos_); 
            }
         }
      } 

      if( USE_LIFT && (end_game_||fast_low_sen_drive_) ) {

         /// Use digital pad to control lift
         if( end_game_ || fast_low_sen_drive_ ) {
            if (gamepad1.dpad_up /*|| gamepad2.dpad_up*/) { // raise lift
               power_lift_ = LIFT_UP_POWER;
               b1_cnt_ = 0;
            } else if (gamepad1.dpad_down /*|| gamepad2.dpad_down*/) {  // lower lift
               power_lift_ = LIFT_DOWN_POWER;
               b1_cnt_ = 0;
            }
         }

         if( power_lift_==0.0) {
             if ( AUTO_LIFT_CONTROL ) {
                 int lift_enc = Math.abs(motorLift_.getCurrentPosition());
                 if( curr_time_<5.0 ) {
                    if (lift_enc < AUTO_LIFT_ENC) {
                        power_lift_ = LIFT_UP_POWER;
                    }
                 } else if( curr_time_>100.0 && curr_time_<105.0 ) {
                     if (lift_enc > AUTO_LIFT_UP_ENC) {
                        power_lift_ = LIFT_DOWN_POWER;
                     }
                 } else if ( single_driver_ == false) {
                    if (curr_time_ < 105.0) {
                       b1_cnt_ = 0;
                    } else if ((b1_cnt_%2) == 1){
                       if (lift_enc <= LIFT_ENC_CNT_END_GAME) {
                          power_lift_ = LIFT_UP_POWER;
                       }
                    }
                 }
             }
         }

         motorLift_.setPower(power_lift_);

        if( gamepad1.right_bumper ) servo_lift_pin_pos_ = LIFT_PIN_PULL;
        servo_lift_pin_.setPosition( servo_lift_pin_pos_ );
      }

      if( USE_SWEEPER ) {
         // use left bummper to control sweeper
         if( (single_driver_ && y1_cnt_%4==1) || y2_cnt_%4==1 ) {
            power_sweeper_ = SWEEP_IN_POWER; 
         } else if( (single_driver_ && y1_cnt_%4==3) || y2_cnt_%4==3 ) {
            power_sweeper_ = SWEEP_OUT_POWER; 
         } else {
            power_sweeper_ = 0 ; 
         }
         if( power_sweeper_==0 && !crater_trip_ && USE_SWEEPER_FOR_DUMPING && is_dumping ) {
            power_sweeper_ = SWEEP_OUT_POWER/3 ;   // slowly sweep out to help dumping
         }
         power_sweeper_ = Range.clip(power_sweeper_,-1,1);
         motor_sweeper_.setPower(power_sweeper_); 
      }

      if( USE_STAB_WHEELS ) {
         if( arm_raised || gamepad1.right_trigger>0.5 ) {
            servo_stab_wheel_.setPosition( STAB_WHEEL_RELEASE ); 
         }
      }

      if( ALLOW_FLIP_ROBOT ) {
      /// Flip robot by gamepad1 left jobstick button 
      boolean flip_robot = false;  
      if( lsb1_cnt_%2 == 1 ) flip_robot = true;     
      if( flip_robot ) {
         if( !drive_sidewalk ) {
            double p = power_lf; 
            power_lf = power_rf; 
            power_rf = p; 
            power_lb = power_lf; 
            power_rb = power_rf; 
         } else {  // sidewalk
            power_lf *= -1; 
            power_rf *= -1; 
            power_lb *= -1; 
            power_rb *= -1; 
         }
      }
      }

      /// Set power values for all motors after clipping
      power_lf = Range.clip(power_lf, -1, 1);
      power_lb = Range.clip(power_lb, -1, 1);
      power_rf = Range.clip(power_rf, -1, 1);
      power_rb = Range.clip(power_rb, -1, 1); 

      motorRF_.setPower(power_rf);
      motorRB_.setPower(power_rb);
      motorLF_.setPower(power_lf);
      motorLB_.setPower(power_lb);

      //////////////////// END OF DRIVING


      // Send telemetry data back to driver station for debugging
      boolean show_msg = false;
      boolean show_loop_cnt = false;
      boolean show_wheel_power = false;
      boolean show_lift_pos = true;
      boolean show_sweeper_pos = false;
      boolean show_heading = false;
      boolean show_voltage = false;
      boolean show_mr_range = false;
      boolean show_rev_range = false;
      boolean show_crane = true;
      boolean show_sweeper = false;

      if(show_msg) {
         telemetry.addData("GiftGears(Crane)", "Team:"+(isRedTeam()?"RED":"BLUE")+", Time:"+String.format("%.2f",curr_time_));

         if( show_crane ) {
            if(USE_CRANE) telemetry.addData("CRANE: Power=", String.valueOf(motor_crane_.getPower())+", Encoder="+String.valueOf(motor_crane_.getCurrentPosition())+", tg_enc="+String.valueOf(motor_crane_.getTargetPosition()));
            if(USE_CRANE_DOUBLE) telemetry.addData("CRANE2: Power=", String.valueOf(motor_crane2_.getPower())+", Encoder="+String.valueOf(motor_crane2_.getCurrentPosition())+", tg_enc2="+String.valueOf(motor_crane2_.getTargetPosition()));
            if(USE_CRANE_ARM) telemetry.addData("CraneArmServo:", String.valueOf(servo_crane_arm_.getPosition()));
            if(USE_CRANE_WINCH) telemetry.addData("CraneWinchServo:", String.valueOf(servo_crane_winch_.getPosition()));
         }
         if(USE_SWEEPER && show_sweeper) telemetry.addData("Sweeper: Power=", String.valueOf(motor_sweeper_.getPower())+", Encoder="+String.valueOf(motor_sweeper_.getCurrentPosition()));

         if( show_loop_cnt ) telemetry.addData("LoopCounter", String.format("#loop=%d, time=%.2f, #loop/sec=%.2f",loop_cnt_,curr_time_,loop_cnt_/curr_time_)); 

         if( show_wheel_power )  telemetry.addData("WheelPower", "Factor="+String.format("%.2f",drive_power_f)+"LF/RF/LB/RB: " + String.format("%.2f", motorLF_.getPower()) + "/" + String.format("%.2f", motorRF_.getPower()) + "/" + String.format("%.2f", motorLB_.getPower()) + "/" + String.format("%.2f", motorRB_.getPower())); 

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
            telemetry.addData("Lift EncPose", ": "+String.valueOf(motorLift_.getCurrentPosition())+", Power="+String.valueOf(motorLift_.getPower())); 
         } 
      } 

   }


   // Code to run when the op mode is first disabled goes here
   @Override public void stop () {   }


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

   /// Low sensitivity drive mode 
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
      //double[] scaleArray = {0.0, 0.35, 0.4, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.78, 0.80, 0.85, 0.90, 0.93, 0.97, 1.0};   // Y17, with encoder for Mecanum 6
      //double[] scaleArray = {0.0, 0.35, 0.4, 0.45, 0.60, 0.70, 0.72, 0.74, 0.76, 0.78, 0.82, 0.86, 0.90, 0.93, 0.96, 0.98, 1.0};   // Y17, with encoder for Mecanum 6

      // 0.14 = deadzone
      double[] scaleArray = {0.0, 0.5, 0.51, 0.52, 0.53, 0.54, 0.56, 0.58, 0.60, 0.62, 0.64, 0.68, 0.70, 0.75, 0.80, 0.90, 1.0};   // Y17, with encoder for Mecanum 6


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
