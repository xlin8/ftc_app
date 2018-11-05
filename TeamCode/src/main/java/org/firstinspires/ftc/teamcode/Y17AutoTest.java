/*
 * This is Program for Autonomous Run for FTC 2017-2018 Relic Recovery season
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

///  Autonomous Run for League Meet0
@TeleOp(name="Y17AutoTest", group="GG")
//@Autonomous(name="Y17AutoTest", group="GG")
@Disabled
public class Y17AutoTest extends Y17LM0Common
{
   /// Drive modes
   static final int DRIVE_STOP = 0;                        // stop
   static final int DRIVE_RESET_ENC_DONE = 1;              // wait till encoder reset

   static final int DRIVE_FORWARD = 2;                     // go forward till timeout
   static final int DRIVE_FORWARD_ENC = 3;                 // go forward for a given distance
   static final int DRIVE_FORWARD_ENC_SLOW = 4;           // slowly go forward for a given distance

   static final int DRIVE_BACKWARD = 5;                    // go backward till timeout
   static final int DRIVE_BACKWARD_ENC = 6;               // go backward for a given distance
   static final int DRIVE_BACKWARD_ENC_SLOW = 7;          // slowly go backward for a given distance


   static final int DRIVE_TURN_LEFT = 8;                   // turn left till timeout
   static final int DRIVE_TURN_TO_LEFT = 9;               // turn left to a specified heading based on gyro
   static final int DRIVE_TURN_LEFT_ENC = 10;              // turn left for a given degree based on encoders

   static final int DRIVE_TURN_RIGHT = 11;                  // turn right till timeout
   static final int DRIVE_TURN_TO_RIGHT = 12;              // turn right to a specified heading based on gyro
   static final int DRIVE_TURN_RIGHT_ENC = 13;             // turn right for a given degree based on encoders


   static final int DRIVE_FORWARD_ENC_TO_WALL = 14;        // go forward to approach to the wall
   static final int DRIVE_BACKWARD_ENC_TO_WALL = 15;       // go backward to approach to the wall
   static final int DRIVE_CHECK_DISTANCE = 16;

   static final int DRIVE_WAIT_TILL = 17;                  // wait till a time is reached
   
   static final int DRIVE_DROP_JEWEL_ARM= 18;
   static final int DRIVE_KNOCK_JEWEL = 19;
   static final int DRIVE_PICTOGRAPH = 20;
   static final int DRIVE_KEY_COLUMN = 21;
   static final int DRIVE_KEY_COLUMN2 = 22;

   static final int DRIVE_OPEN_CLAW = 23;
   static final int DRIVE_CLOSE_CLAW = 24;
   static final int DRIVE_SHIFT_CLAW_LEFT = 25;
   static final int DRIVE_SHIFT_CLAW_RIGHT = 26;

   static final int DRIVE_STATE_JUMP = 27;
   static final int DRIVE_SHIFT_GEAR = 28;                 // shift gears to make robot move faster/slower by changing drive_power_factor_
   static final int DRIVE_KEY_COLUMN3 = 29;                // Pos1 for LQT

   static final int DRIVE_FORWARD_ENC_GRAB = 30;           // go forward for a given distance to grab glyphs
   static final int DRIVE_KEY_COLUMN4 = 31;                // Pos2 for SQ

   static final int DRIVE_SWITCH_INTAKE = 32;              // change intake system state
   static final int DRIVE_KEY_COLUMN5 = 33;                // Pos1 for OR
   static final int DRIVE_KEY_COLUMN6 = 34;                // Pos2 for OR

   static final int DRIVE_LIFT_CLAWS= 35;                  // lift grabber
   static final int DRIVE_OPEN_CLAW_ONLY = 36; 

   static final int DRIVE_KEY_COLUMN7 = 37;                // Pos1 for SR
   static final int DRIVE_KEY_COLUMN8 = 38;                // Pos2 for SR

   static final int DRIVE_MODE_NUM = 39;
//*****************************************************************************************************************

   static final double AUTO_RESET_ENC_TIME = 1.00;         // period for reseting encoders when entering DRIVE_RESET_ENC_DONE mode

   static final boolean FLIP_ROBOT_FOR_BLUE = false;       // flip robot front/end for BLUE

   static final boolean USE_IMU_FOR_TURN = true;           // use IMU sensor to turn
   static final boolean USE_ENC_FOR_DRIVE = true;          // use encoder for accurate movement

   static final double DRIVE_ENC_WHEEL_POWER = 0.40;       // default driving power using encoder; ~29sec for 6*3m, for 2017
   static final double DRIVE_ENC_SLOW_WHEEL_POWER = 0.15;  // power for encoder based slow drive for STOP_WHITE
   static final double DRIVE_ENC_TURN_WHEEL_POWER = 0.12;  // default turning power using encoder, 2017/09/08
   static final double DRIVE_ENC_POWER_RATIO = DRIVE_ENC_SLOW_WHEEL_POWER/DRIVE_ENC_WHEEL_POWER ;  // ratio between low and high
   static final double DRIVE_WHEEL_POWER = 0.20;           // default driving power
   static final double TURN_WHEEL_POWER = 0.20;            // default turning power
   static final double TURN_SLOW_WHEEL_POWER = 0.15;       // slow turning power
   static final double SLOW_TURN_DEGREE = 10.0;            // degree to enable slow turn; 10.0 degree by dflt; 0, no slow turn 

   /// Encoder scales for TestBot, 2017/09/06 
   final static double ENC_DIST_SCALE = 2000.0/1.00;       // 2000 ticks <=> 1.00 meters, 2017/09/06
   //final static double ENC_DEG_SCALE = 2000.0/253;         // 2000 ticks <=> 360-107=253 degree on mat, 2017/09/08 
   final static double ENC_DEG_SCALE = 2000.0/150;         // 2000 ticks <=> ~150 for MW6 on mat, 2017/10/15

   static final double AUTO_RUN_TIME = 60.0;               // 60 sec for testing/debugging

   static final boolean START_AS_RED = true;               // true for RED team; false for BLUE team
   static final boolean FLIP_COLOR_BY_X1 = true;           // allow user to press Pad1/X to flip team color within 1sec if true for debugging
   static final boolean SHIFT_START_POS_BY_Y1 = true;      // allow user to press Pad1/Y to indicate not the default starting position is used

   static final double  AUTO_CORRECT_HEADING_GAIN = 0.012;  // previously 0.025; power percentage to be adjusted for each degree of heading error; 1-degree error => 5% power diff // dflt for 0.40, 2017/09/08
   static final double  AUTO_CORRECT_MAX_HEADING_ERROR = 40;  // power percentage to be adjusted for each degree of heading error; 1-degree error => 5% power diff
   static final boolean AUTO_CORRECT_HEADING = true;       // automatically correct the last heading for DRIVE_FORWARD mode
   static final double  MAX_HEADING_CORRECTION = 0.95;     // max heading correction; 0, not used

   /// Autonomous specific hardware
   VuforiaLocalizer vuforia;
   VuforiaTrackables relicTrackables; 
   VuforiaTrackable relicTemplate;
   RelicRecoveryVuMark pictograph_;


   LynxI2cColorRangeSensor rev_rgb_range_;         // REV color/range sensor2
   static final boolean USE_DOUBLE_RGB_FOR_JEWEL = true;   // true for double jewel sensors
   LynxI2cColorRangeSensor rev_rgb_range2_;         // 2nd color/range sensor for jewel 

   /// Autonomous Specific Variables
   double servo_jewel_pos = SERVO_JEWEL_INIT;
   double servo_knock_pos = SERVO_KNOCK_INIT;
   static final double MIN_JEWEL_ALPHA = 10;           // min alpha for jewel color
   static final double MIN_JEWEL_COLOR_RATIO = 1.2;    // min ratio to determine the jewel color
   //static final double MIN_JEWEL_COLOR_RATIO = 1.1;    // min ratio to determine the jewel color, 2018/03/09
   boolean jewel_knocked_ = false;
   boolean red_jewel_ = false;
   boolean blue_jewel_ = false;

   static final boolean SCAN_JEWEL_WHEN_DROP_ARM = true;   // scan the jewel when dropping the arm to improve the reliability of color detection
   //static final double MIN_NUM_DIFF_JEWEL = 3;           // min alpha for jewel color
   static final double MIN_NUM_DIFF_JEWEL = 2;            // min diff for times for a jewel to be seen, lowered for double sensors, 2018/02/21
   int num_red_seen_ = 0;                                 // number of time red jewel is seen
   int num_blue_seen_ = 0;                                // number of time blue jewel is seen
   double jewel_distance_ = 0.0;                          // distance for jewel
   double sum_distance_ = 0.0;                            // sum of distance for jewel

   int num2_red_seen_ = 0;                                // number of time red jewel is seen by 2nd RGB sensor
   int num2_blue_seen_ = 0;                               // number of time blue jewel is seen by 2nd RGB sensor
   double jewel2_distance_ = 0.0;                         // distance for jewel by 2nd sensor
   double sum2_distance_ = 0.0;                           // sum of distance detected by 2nd sensor
   
   static final double  BOX_WALL_DISTANCE = 12;             // try to stop in the mid of the first column 

   static final double  WALL_DISTANCE_MAX = 17.5;          // max distance to wall; distance higher than this will trigger re-alignment
   static final double  MAX_RANGE  = 99.9;                 // any distance larger than this value will be considered as invalid reading
   static final double  MIN_WALL_DIS_RED = 20.0;           // wall distance to stop for RED/ENC_TO_WALL
   static final double  MIN_WALL_DIS_BLUE = 25.0;          // wall distance to stop for BLUE/ENC_TO_WALL
   static final double  MAX_WALL_DIS_RED1 = 100.0;         // wall distance to stop for RED position1
   static final double  MAX_WALL_DIS_BLUE1 = 100.0;        // wall distance to stop for BLUE position1
   static final double  WALL_PRE_BRAKE_DIST = 0.0;        // pre-brake robot by switch to low power when robot is close to wall if >0.0
   
   static final boolean SMART_DRIVE_MOVE = false;          // make move smart; lower power for better heading correction; lower power when approaching the wall
   static final double  SMART_DRIVE_MOVE_SCALE = 1.4;      // 40% higher/lower power to start the turn
   static final double  SMART_DRIVE_MOVE_ENC_RATIO = 0.3;  // for the first 30% of trip
   static final boolean SMART_DRIVE_TURN = true;          // make turn smart: higher power to start the turning
   static final double  SMART_DRIVE_TURN_SCALE = 1.3;      // 30% higher power to start the turn
   static final double  SMART_DRIVE_TURN_ENC_RATIO = 0.3;  // for the first 30% of trip
   boolean smart_drive_on_ = false;                        // smart drive is ON at this time
   boolean pre_brake_on_ = false;                          // pre-brake is ON if set

   boolean is_red_last_step_ = false;                      // set true for last step for RED

   /// Variables for Autonomous Mode 
   double init_wait_time_ = 0.01;                           // additional wait time; default 1 sec for debugging
   boolean dflt_start_pos_ = true;                         // default starting position if true
   int     start_pos_id_ = 1;                              // starting position ID; dflt 1,
   boolean color_flipped_ = false;                         // false for RED team, true for BLUE team

   int test_trip_id_ = -1;                                 // test trip ID for debugging

   int curr_state_id_ = -1;                                // current state ID
   double curr_state_value_ = 0.0;                         // current state value for distance, timeout, etc.
   int curr_state_drive_mode_ = -1;                        // current state drive mode
   double curr_state_start_t_ = 0.0;                       // current state start time
   double curr_state_start_h_ = 0.0;                       // current state start heading
   double curr_state_enc_cnt_ = 0.0;                       // current state targeted encoder count

   double heading_ = 0.0;                                  // current heading
   double target_heading_ = 0.0;                           // target heading for current state
   double heading_error_ = 0.0;                            // current heading error
   double adjusted_heading_error_ = 0.0;                   // adjusted heading error
   double heading_correction_ = 0.0;                       // heading correction
   double last_turn_to_heading_ = -1.0;                    // last TURN_TO heading

   boolean turn_slow_ = false;                             // turn slow if true
   
   int num_dist_ok_ = 0;                                   // numer of times for which distance is OK
   int num_dist_far_ = 0;                                  // numer of times for which distance is too far
   
   String trip_name_ = "Default";                          // trip name

   int lf_encoder_max_ = 0;                                  // LF motor encoder max position
   int lr_encoder_max_ = 0; 
   int rf_encoder_max_ = 0; 
   int rr_encoder_max_ = 0; 

   int target_lift_enc_ = -1;                               // targeted lift encoder position
   static final boolean USE_PUSHER_AR = true;                 // use pusher when OPEN_CLAW
   double last_open_claw_end_t_ = 0.0;                     // last OPEN_CLAW ending time
   double drive_power_factor_ = 1.0;                       // factor for drive power
   

   // Constructor
   public Y17AutoTest() { 
   }

   /// Autonomous mode specific initialization
   @Override public void init() {
      super.init();
      
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
      parameters.vuforiaLicenseKey = "ATbVhA//////AAAAGQpUcoBny0Xdi+FFWntcC3w9C63+hv3ccdXKcUsUhNYtbt8IbpT9SQ+VsthWIyix0rrzYP8KYaSYY5na+nufoGmLQo8vE8CPWmUj8eZcdlM9k4mi8ge0T2uzuoKZmcllal8cM3hRxo1JBFVtavCrgulnZxQ8hMsbzZuA+dZDGQTOEOCCH8ZHuh6wrIUygVerHfrXXlpeIAQvXzBiYrVPetr3zu3ROn6rno75mQ0KCM8Qp87BGS4Orx+GwxL8FlO+EXA3aSBvDh7+a57co5212MkGIRUceXxAd+BfoFjiWg3SbJpVbDM7TcDApVR88jlqeEDmbc/ODajLjEKziycgihi1rpq1lOBys2oJ68qdVrtO";
      parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
      this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
      relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
      relicTemplate = relicTrackables.get(0);
      relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
      relicTrackables.activate(); 
      pictograph_ = RelicRecoveryVuMark.UNKNOWN;

      rev_rgb_range_ = hardwareMap.get(LynxI2cColorRangeSensor.class, "rev_rgb2");
      if( USE_DOUBLE_RGB_FOR_JEWEL ) {
         rev_rgb_range2_ = hardwareMap.get(LynxI2cColorRangeSensor.class, "rev_jewel_rgb2");
      }

      if( USE_ENC_FOR_DRIVE ) {
         reset_drive_encoders();
      }
      target_lift_enc_ = -1; 
      if( USE_LIFT ) { motorLift_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER); }
      last_open_claw_end_t_ = 0.0;
      
      heading_ = 0.0; 
      target_heading_ = 0.0;
      smart_drive_on_ = false;
      is_red_last_step_ = false;
      pre_brake_on_ = false;

      num_dist_ok_ = 0; 
      num_dist_far_ = 0;

      num_red_seen_ = 0; 
      num_blue_seen_ = 0; 

   }

    @Override
    public void start() {
    }

   @Override public void init_loop() {
      super.init_loop(); 

      curr_time_ = 0.0; 

      if(USE_LIFT) motorLift_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER );
   }

   @Override public void loop() {
      double time_t = timer_.time();
      curr_time_ = time_t; 
      heading_error_ = 0.0; 
      adjusted_heading_error_ = 0.0; 
      heading_correction_ = 0.0; 
      smart_drive_on_ = false;
      pre_brake_on_ = false;
      
      servo_jewel_pos = SERVO_JEWEL_INIT;
      servo_knock_pos = SERVO_KNOCK_INIT;

      servo_left_jaw_pos_ = CR_SERVO_STOP;
      servo_right_jaw_pos_ = CR_SERVO_STOP;

      servo_left_jaw2_pos_ = CR_SERVO_STOP;
      servo_right_jaw2_pos_ = CR_SERVO_STOP;

      servo_pusher_pos_ = CR_SERVO_STOP;

      power_lift = 0.0; 

/*
      RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
      if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
         telemetry.addData("VuMark", "%s visible", vuMark);
      }  else {
         telemetry.addData("VuMark", "not visible");
      }
*/
      if( false && USE_IMU_FOR_TURN && imu_!=null ) { 
          imu_angles_  = imu_.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);  // acquiring angles are relatively expensive, don't want to do it too often
          //imu_gravity_  = imu_.getGravity();
      }

      if( time_t<=1.0 ) {
         /// Allow user to flip team color for easy testing
         if( FLIP_COLOR_BY_X1 && gamepad1.x ) {
            color_flipped_ = true; 
         }

         /// Allow user to switch to alternate starting position 
         if( SHIFT_START_POS_BY_Y1 && gamepad1.y ) {
            dflt_start_pos_ = false; 
            start_pos_id_ = 2; 
         } else if( SHIFT_START_POS_BY_Y1 && gamepad1.left_bumper ) {
            dflt_start_pos_ = false; 
            start_pos_id_ = 3; 
         } else if( SHIFT_START_POS_BY_Y1 && gamepad1.right_bumper ) {
            dflt_start_pos_ = false; 
            start_pos_id_ = 4; 
         }

         /// Choose different test trip for debugging, only for RED dflt starting position
         if( gamepad1.a && !gamepad1.b ) {
            test_trip_id_ = 2;    // encoder based driving including heading correction
         } else if( !gamepad1.a && gamepad1.b ) {
            //test_trip_id_ = 7;    // 10sec shooting 
            test_trip_id_ = 3;    // 10sec shooting 
         } else if( gamepad1.a && gamepad1.b ) {
            test_trip_id_ = 1;    // drive distance calibration
         } else if( gamepad2.a && !gamepad2.b ) {
            test_trip_id_ = 5;   // STOP_WHITE
         } else if( !gamepad2.a && gamepad2.b ) {
            test_trip_id_ = 9;    // drive to wall
         } else if( gamepad2.x ) {
            test_trip_id_ = 6;    // beacon pushing
         }
      }

      int drive_mode = DRIVE_STOP; 
      if( (START_AS_RED && !color_flipped_) || (!START_AS_RED && color_flipped_) ) {
         if( dflt_start_pos_ ) {
            drive_mode = getDriveModeRed(time_t);
         }
      } else {
         if( dflt_start_pos_ ) {
            drive_mode = getDriveModeBlue(time_t);
         }
      }
      
      double power_lf = 0, power_rf = 0, power_lb = 0, power_rb = 0;   // power for wheels


      if( time_t>=AUTO_RUN_TIME ) {
         // timeout, stop robot and sweeper
      } else {

         power_lf = getLeftFrontPower( drive_mode );
         power_rf = getRightFrontPower( drive_mode );

         if( smart_drive_on_ ) {
            /// Try to dynamically adjust the drive/turn power 
            if( drive_mode==DRIVE_TURN_LEFT_ENC || drive_mode==DRIVE_TURN_RIGHT_ENC ) {
               // boost power to start the turn
               power_lf *= SMART_DRIVE_TURN_SCALE; 
               power_rf *= SMART_DRIVE_TURN_SCALE; 
            } else if( drive_mode==DRIVE_FORWARD_ENC || drive_mode==DRIVE_BACKWARD_ENC ) { 
               // slow it down for better heading correction
               power_lf /= SMART_DRIVE_MOVE_SCALE; 
               power_rf /= SMART_DRIVE_MOVE_SCALE; 
            } else if( drive_mode==DRIVE_FORWARD_ENC_TO_WALL || drive_mode==DRIVE_BACKWARD_ENC_TO_WALL ) { 
               // faster first, then slow for accurate position 
               power_lf *= SMART_DRIVE_MOVE_SCALE; 
               power_rf *= SMART_DRIVE_MOVE_SCALE; 
            } 
         }

         if( pre_brake_on_ ) {
            if( drive_mode==DRIVE_FORWARD_ENC_TO_WALL || drive_mode==DRIVE_BACKWARD_ENC_TO_WALL ) {
               // switch from high power to lower power when wall is close enough
               if( Math.abs(power_lf) == DRIVE_ENC_WHEEL_POWER ) {
                  power_lf *= DRIVE_ENC_POWER_RATIO; 
                  power_rf *= DRIVE_ENC_POWER_RATIO; 
               }
            }
         }

         double h_err = 0.0;    // heading error
         if( AUTO_CORRECT_HEADING && 
            ( drive_mode==DRIVE_FORWARD_ENC_TO_WALL || drive_mode==DRIVE_BACKWARD_ENC_TO_WALL || 
              drive_mode==DRIVE_FORWARD_ENC || drive_mode==DRIVE_BACKWARD_ENC || 
              drive_mode==DRIVE_FORWARD_ENC_GRAB ||
              drive_mode==DRIVE_FORWARD_ENC_SLOW || drive_mode==DRIVE_BACKWARD_ENC_SLOW ) ) {
            // Get the heading error from IMU
            h_err = getHeadingError();  // expensive heading reading
            if( Math.abs(h_err)>AUTO_CORRECT_MAX_HEADING_ERROR ) {  // prevent incorrect heading error causing robot to spin
               h_err=0.0; 
            }

         }

         /// Adjust wheel power to correct heading or follow wall
         if( Math.abs(h_err)>0.0 ) {
            if( FLIP_ROBOT_FOR_BLUE && !isRedTeam() ) {          
               // left/right flipped when swapping the front/back
               h_err *= -1;
            }
            if( drive_mode==DRIVE_BACKWARD_ENC ||
                drive_mode==DRIVE_BACKWARD_ENC_SLOW ||
                drive_mode==DRIVE_BACKWARD_ENC_TO_WALL ) {
               // flip heading error for backward drive
               h_err *= -1;
            }

            adjusted_heading_error_ = h_err; 

            double corr = h_err*AUTO_CORRECT_HEADING_GAIN; 
            if( MAX_HEADING_CORRECTION>0.0 ) {             // clip the correction to ensure that motor is not reversed to avoid big swing
               corr = Range.clip(corr, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);    
            }
            heading_correction_ = corr; 
            power_lf *= ( 1 + corr );  // h_err>0, bias to left, turn right by increasing power for left wheels
            power_rf *= ( 1 - corr ); 
         }


         if( FLIP_ROBOT_FOR_BLUE && !isRedTeam() ) {
            // flip power for all driving modes
            if( drive_mode==DRIVE_FORWARD || drive_mode==DRIVE_BACKWARD || 
                drive_mode==DRIVE_FORWARD_ENC || drive_mode==DRIVE_BACKWARD_ENC || 
                drive_mode==DRIVE_FORWARD_ENC_GRAB || 
                drive_mode==DRIVE_FORWARD_ENC_SLOW || drive_mode==DRIVE_BACKWARD_ENC_SLOW || 
                drive_mode==DRIVE_FORWARD_ENC_TO_WALL|| drive_mode==DRIVE_BACKWARD_ENC_TO_WALL )
            {
               power_lf *= -1;
               power_rf *= -1;
            }
         }

         power_lb = power_lf; 
         power_rb = power_rf; 
      }

      /// Set power values for all wheel motors
      if( drive_power_factor_!=1.0 ) {
         power_lf *= drive_power_factor_; 
         power_rf *= drive_power_factor_; 
         power_lb *= drive_power_factor_; 
         power_rb *= drive_power_factor_; 
      }

      power_lf = Range.clip(power_lf, -1, 1);
      power_rf = Range.clip(power_rf, -1, 1); 
      power_lb = Range.clip(power_lb, -1, 1);
      power_rb = Range.clip(power_rb, -1, 1);

      motorLF_.setPower(power_lf);
      motorRF_.setPower(power_rf);
      motorLB_.setPower(power_lb); 
      motorRB_.setPower(power_rb);

      if( USE_GLYPH_WHEELS ) {
         double power_gl = 0.0; 
         if( drive_mode==DRIVE_FORWARD_ENC_GRAB ) {
            power_gl = GLYPH_LEFT_PUSH_POWER; 
         }
         if( intake_curr_state_ == INTAKE_PUSH ) {
            power_gl = GLYPH_LEFT_PUSH_POWER; 
         } else if( intake_curr_state_ == INTAKE_PULL ) {
            power_gl = -GLYPH_LEFT_PUSH_POWER; 
         }
         power_gl = Range.clip(power_gl, -1, 1);
         motor_glyph_left_.setPower(power_gl);
         motor_glyph_right_.setPower(-power_gl);
      }
      if( USE_INTAKE_SERVO_CR ) {
         double power_gl = 0.0; 

         if( curr_time_<5.0 ) {
            servo_left_intake_pos_ = SERVO_LEFT_INTAKE_SHELF_CR; 
            servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_SHELF_CR; 
            intake_curr_state_ = INTAKE_SHELF;
         } else if( intake_curr_state_ == INTAKE_PUSH ) {
            // pushing
            servo_left_intake_pos_ = CR_SERVO_STOP; 
            servo_right_intake_pos_ = CR_SERVO_STOP;
            if( USE_INTAKE_MAG_SWITCH ) {
               if( mag_intake_left_.getState() ) { servo_left_intake_pos_ = SERVO_LEFT_INTAKE_INIT_CR; }
               if( mag_intake_right_.getState() ) { servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_INIT_CR; }
            } 
            if( USE_INTAKE_TOUCH ) {
               if( !touch_intake_left_.getState() ) { servo_left_intake_pos_ = SERVO_LEFT_INTAKE_SHELF_CR; }
               if( !touch_intake_right_.getState() ) { servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_SHELF_CR; } 
            }
            power_gl = GLYPH_LEFT_PUSH_POWER; 
         } else if( intake_curr_state_ == INTAKE_PULL ) {
            // pushing
            servo_left_intake_pos_ = CR_SERVO_STOP; 
            servo_right_intake_pos_ = CR_SERVO_STOP;
            if( USE_INTAKE_MAG_SWITCH ) {
               if( mag_intake_left_.getState() ) { servo_left_intake_pos_ = SERVO_LEFT_INTAKE_INIT_CR; }
               if( mag_intake_right_.getState() ) { servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_INIT_CR; }
            } 
            if( USE_INTAKE_TOUCH ) {
               if( !touch_intake_left_.getState() ) { servo_left_intake_pos_ = SERVO_LEFT_INTAKE_SHELF_CR; }
               if( !touch_intake_right_.getState() ) { servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_SHELF_CR; } 
            }
            power_gl = -GLYPH_LEFT_PUSH_POWER; 
         } else if( intake_curr_state_ == INTAKE_INIT ) {
            // fold arms in
            servo_left_intake_pos_ = CR_SERVO_STOP; 
            servo_right_intake_pos_ = CR_SERVO_STOP;
            if( USE_INTAKE_TOUCH ) {
               if( touch_intake_left_.getState() ) { servo_left_intake_pos_ = SERVO_LEFT_INTAKE_INIT_CR; }
               if( touch_intake_right_.getState() ) { servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_INIT_CR; } 
            }
            power_gl = -GLYPH_LEFT_PUSH_POWER; 
         } else if( intake_curr_state_ == INTAKE_SHELF ) {
            // fold arms back
            servo_left_intake_pos_ = SERVO_LEFT_INTAKE_SHELF_CR; 
            servo_right_intake_pos_ = SERVO_RIGHT_INTAKE_SHELF_CR;
         } else {
            servo_left_intake_pos_ = CR_SERVO_STOP; 
            servo_right_intake_pos_ = CR_SERVO_STOP;
         }
         servo_left_intake_.setPosition( servo_left_intake_pos_ ); 
         servo_right_intake_.setPosition( servo_right_intake_pos_ ); 

         if( curr_time_>3.0 && curr_time_<5.0 ) {  // open bottom claws for glyph to drop
            servo_left_jaw_pos_ = LEFT_JAW_OPEN;
            servo_right_jaw_pos_ = RIGHT_JAW_OPEN;
         }

         power_gl = Range.clip(power_gl, -1, 1);
         motor_glyph_left_.setPower(power_gl);
         motor_glyph_right_.setPower(-power_gl);
      }

      if( USE_LIFT ) {
         int max_lift_enc_diff = 50; 
         if( target_lift_enc_>=0 ) {
            int lift_enc = motorLift_.getCurrentPosition(); 
            int lift_enc_diff = lift_enc-target_lift_enc_;
            if( lift_enc_diff<0 ) { // go up
               if( Math.abs(lift_enc_diff)<max_lift_enc_diff ) {
                  power_lift = MIN_LIFT_HOLD_POWER; 
               } else {
                  power_lift = LIFT_UP_POWER; 
               }
            } else if( lift_enc_diff>0 ) {  // go down
               if( Math.abs(lift_enc_diff)<max_lift_enc_diff ) {
                  power_lift = 0;  // let it drip
               } else {
                  power_lift = LIFT_DOWN_POWER; 
               }
            }
         }
         power_lift = Range.clip(power_lift, -1, 1);
         motorLift_.setPower( power_lift );

         //boolean hold_claw = false; 
         boolean hold_claw = true; 
         if( hold_claw && target_lift_enc_>0 ) {
            // keep the claw to close if the lift is rised to avoid dropping the glyph
            if( servo_left_jaw_pos_==CR_SERVO_STOP ) {
               servo_left_jaw_pos_ = LEFT_JAW_CLOSE_HOLD; 
            }
            if( servo_right_jaw_pos_==CR_SERVO_STOP ) {
               servo_right_jaw_pos_ = RIGHT_JAW_CLOSE_HOLD; 
            }
         }
      }

      if( USE_PUSHER_AR && servo_pusher_pos_==CR_SERVO_STOP && last_open_claw_end_t_>0.0 && curr_time_<last_open_claw_end_t_+2.0 ) { // retract the pusher for 2sec
         servo_pusher_pos_ = PUSHER_PULL_HOLD; 
      }

      /// Set power values for all servos
      servo_jewel_pos = Range.clip(servo_jewel_pos, 0, 1);
      servo_knock_pos = Range.clip(servo_knock_pos, 0, 1);

      servo_jewel_.setPosition(servo_jewel_pos);
      servo_knock_.setPosition(servo_knock_pos);

      servo_left_jaw_pos_ = Range.clip(servo_left_jaw_pos_, 0, 1);
      servo_right_jaw_pos_ = Range.clip(servo_right_jaw_pos_, 0, 1);
      servo_left_jaw2_pos_ = Range.clip(servo_left_jaw2_pos_, 0, 1);
      servo_right_jaw2_pos_ = Range.clip(servo_right_jaw2_pos_, 0, 1);

      servo_left_jaw_.setPosition(servo_left_jaw_pos_);
      servo_right_jaw_.setPosition(servo_right_jaw_pos_);
      servo_left_jaw2_.setPosition(servo_left_jaw2_pos_);
      servo_right_jaw2_.setPosition(servo_right_jaw2_pos_);

      servo_pusher_pos_ = Range.clip(servo_pusher_pos_, 0, 1);
      servo_pusher_.setPosition(servo_pusher_pos_); 


      boolean debug_enc_drive = false;     // wheel motor power and encoder position
      boolean debug_enc_lift = false;     // lift motor encoder position
      boolean debug_imu = false;          // expensive, big over-turning
      boolean debug_drive_motors = false;  // print out motor encoder position
      boolean show_mr_range = false;

      telemetry.addData("GiftGears", "Trip:"+String.valueOf(trip_name_)+"Team:"+(isRedTeam()?"RED":"BLUE")+", DfltStart:"+String.valueOf(dflt_start_pos_)+", TimeLeft:"+String.format("%.2f",AUTO_RUN_TIME-time_t));
      telemetry.addData("CurrState", "Id:" + String.valueOf(curr_state_id_)+", Mode:"+String.valueOf(curr_state_drive_mode_)+", StartTime:"+String.format("%.2f", curr_state_start_t_)+", Heading:"+ String.format("Start=%.2f/Curr=%.2f/Target=%.2f; Error=%.2f/AHE=%.2f/Corr=%.2f; DriveFactor=%.2f",curr_state_start_h_,heading_,target_heading_,heading_error_,adjusted_heading_error_,heading_correction_,drive_power_factor_));

      if(debug_enc_drive) {
         telemetry.addData("Wheel Power",  ": LF="+String.valueOf(motorLF_.getPower())+", RF="+String.valueOf(motorRF_.getPower())+", LB="+String.valueOf(motorLB_.getPower())+",RB="+String.valueOf(motorRB_.getPower()));
         if(USE_ENC_FOR_DRIVE) telemetry.addData("EncPos",  ": LF="+String.valueOf(motorLF_.getCurrentPosition())+", RF="+String.valueOf(motorRF_.getCurrentPosition())+", LB="+String.valueOf(motorLB_.getCurrentPosition())+",RB="+String.valueOf(motorRB_.getCurrentPosition()));
         //if(USE_ENC_FOR_DRIVE) RobotLog.i("EncPos",  ": LF="+String.valueOf(motorLF_.getCurrentPosition())+", RF="+String.valueOf(motorRF_.getCurrentPosition())+", LB="+String.valueOf(motorLB_.getCurrentPosition())+",RB="+String.valueOf(motorRB_.getCurrentPosition()));
      }
      if(debug_enc_lift && USE_LIFT) { 
         telemetry.addData("Lift",  ": power="+String.valueOf(motorLift_.getPower())+", pos="+String.valueOf(motorLift_.getCurrentPosition())); 
      }

      if( show_mr_range && mr_range_!=null ) {
         telemetry.addData("MRRangeSensor", String.format("ultra/opt/dist=%4d/%.2f/%.2f",mr_range_.rawUltrasonic(),mr_range_.cmOptical(),mr_range_.getDistance(DistanceUnit.CM)));
      }

      telemetry.addData("Jewel Module", String.format("servo_jewel_pos=%.2f/%.2f, servo_knock_pos=%.2f/%.2f; #red/#blue=%d/%d, color=%s; dist=%.2f, AvgDist=%.2f",servo_jewel_pos,servo_jewel_.getPosition(),servo_knock_pos,servo_knock_.getPosition(),num_red_seen_,num_blue_seen_,(red_jewel_?"red":(blue_jewel_?"blue":"unknown")),jewel_distance_,sum_distance_/(num_red_seen_+num_blue_seen_)));
      if( USE_DOUBLE_RGB_FOR_JEWEL ) telemetry.addData("Jewel RGB2", String.format("#red2/#blue2=%d/%d, color=%s; dist2=%.2f, AvgDist2=%.2f",num2_red_seen_,num2_blue_seen_,(red_jewel_?"red":(blue_jewel_?"blue":"unknown")),jewel2_distance_,sum2_distance_/(num2_red_seen_+num2_blue_seen_)));

      telemetry.addData("VuMark", "%s visible", pictograph_);
      telemetry.addData("TestTrip:", "%d", test_trip_id_);


      if(debug_drive_motors && USE_ENC_FOR_DRIVE) {
         int lf_enc = motorLF_.getCurrentPosition(); 
         int lr_enc = motorLB_.getCurrentPosition(); 
         int rf_enc = motorRF_.getCurrentPosition(); 
         int rr_enc = motorRB_.getCurrentPosition(); 
         telemetry.addData("Current EncPos", ": LF="+String.valueOf(lf_enc)+", LR="+String.valueOf(lr_enc)+"; RF="+String.valueOf(rf_enc)+", RR="+String.valueOf(rr_enc));
         
         if(Math.abs(lf_enc)>Math.abs(lf_encoder_max_))  lf_encoder_max_=lf_enc; 
         if(Math.abs(lr_enc)>Math.abs(lr_encoder_max_))  lr_encoder_max_=lr_enc; 
         if(Math.abs(rf_enc)>Math.abs(rf_encoder_max_))  rf_encoder_max_=rf_enc; 
         if(Math.abs(rr_enc)>Math.abs(rr_encoder_max_))  rr_encoder_max_=rr_enc; 
         telemetry.addData("Max EncPos", ": LF="+String.valueOf(lf_encoder_max_)+", LR="+String.valueOf(lr_encoder_max_)+"; RF="+String.valueOf(rf_encoder_max_)+", RR="+String.valueOf(rr_encoder_max_));

         telemetry.addData("IMU", "heading: " + String.format("%.2f", getHeading())); 
         //if(USE_IMU_FOR_TURN && imu_!=null) telemetry.addData("IMU", "status="+String.valueOf(imu_.getSystemStatus())+", calib="+String.valueOf(imu_.getCalibrationStatus())+", heading="+formatAngle(imu_angles_.angleUnit,imu_angles_.firstAngle)+", roll="+formatAngle(imu_angles_.angleUnit,imu_angles_.secondAngle)+", pitch="+formatAngle(imu_angles_.angleUnit,imu_angles_.thirdAngle)); 
      }

      if(debug_imu) {
         if(USE_IMU_FOR_TURN && imu_!=null) telemetry.addData("IMU", "status="+String.valueOf(imu_.getSystemStatus())+", calib="+String.valueOf(imu_.getCalibrationStatus())+", heading="+formatAngle(imu_angles_.angleUnit,imu_angles_.firstAngle)+", roll="+formatAngle(imu_angles_.angleUnit,imu_angles_.secondAngle)+", pitch="+formatAngle(imu_angles_.angleUnit,imu_angles_.thirdAngle)); 
      }
   }

   // Code to run when the op mode is first disabled goes here
   @Override public void stop() { 
   }

   /// Return true if it's RED team
   boolean isRedTeam() {
      return ( START_AS_RED ? (!color_flipped_) : color_flipped_ );
   }

   /// Return current robot heading based on gyro/IMU reading


   /// Return degree which robot has turned. Must be very careful with the boundary and the range of gyro reading.
   //  Assume: heading_ and curr_state_start_h_ are valid. 
   //  For MR I2C Gyro, heading range (-inf, int). No special boundary handling is needed.
   double getDegreeTurned() { 
      return Math.abs(heading_ - curr_state_start_h_); 
   }

   /// Return heading error and update heading_ & heading_error_
   double getHeadingError() {
      heading_error_ = 0;
      if( AUTO_CORRECT_HEADING ) {
         heading_error_ = getHeading()-target_heading_;
         if( heading_error_>(360-AUTO_CORRECT_MAX_HEADING_ERROR) && heading_error_<(360+AUTO_CORRECT_MAX_HEADING_ERROR) ) {
            heading_error_ = heading_error_ - 360;
         } else if( heading_error_>(-360-AUTO_CORRECT_MAX_HEADING_ERROR) && heading_error_<(-360+AUTO_CORRECT_MAX_HEADING_ERROR) ) {
            heading_error_ = heading_error_ + 360;
         }
      }
      return heading_error_;
   }

   /// Determine the power for left front wheel based on the driving mode
   double getLeftFrontPower(int mode) {
      double pwr = 0; 
      switch( mode ) {
         case DRIVE_STOP:
         case DRIVE_DROP_JEWEL_ARM:
         case DRIVE_KNOCK_JEWEL:
            pwr = 0.0;
            break;
         case DRIVE_FORWARD:   
            pwr = DRIVE_WHEEL_POWER; 
            break;
         case DRIVE_BACKWARD:    
            pwr = -1.0*DRIVE_WHEEL_POWER; 
            break;
         case DRIVE_TURN_LEFT:
         case DRIVE_TURN_TO_LEFT:
            if( turn_slow_ ) {
               pwr = -1.0*TURN_SLOW_WHEEL_POWER; 
            } else {
               pwr = -1.0*TURN_WHEEL_POWER; 
            }
            break; 
         case DRIVE_TURN_RIGHT:
         case DRIVE_TURN_TO_RIGHT:
            if( turn_slow_ ) {
               pwr = TURN_SLOW_WHEEL_POWER; 
            } else {
               pwr = TURN_WHEEL_POWER; 
            }
            break;
         case DRIVE_RESET_ENC_DONE:       
            pwr = 0.0;
            break; 
         case DRIVE_FORWARD_ENC:
         case DRIVE_FORWARD_ENC_GRAB:
            pwr = DRIVE_ENC_WHEEL_POWER; 
            break; 
         case DRIVE_FORWARD_ENC_SLOW:
         case DRIVE_FORWARD_ENC_TO_WALL:
            pwr = DRIVE_ENC_SLOW_WHEEL_POWER;
            break; 
         case DRIVE_BACKWARD_ENC:
            pwr = -1.0*DRIVE_ENC_WHEEL_POWER; 
            break; 
         case DRIVE_BACKWARD_ENC_SLOW:
         case DRIVE_BACKWARD_ENC_TO_WALL:
            pwr = -1.0*DRIVE_ENC_SLOW_WHEEL_POWER;
            break; 
         case DRIVE_TURN_LEFT_ENC:   
            pwr = -1.0*DRIVE_ENC_TURN_WHEEL_POWER; 
            break; 
         case DRIVE_TURN_RIGHT_ENC:   
            pwr = DRIVE_ENC_TURN_WHEEL_POWER; 
            break;
         case DRIVE_CHECK_DISTANCE:
            pwr = 0.0;
            break;
         default: 
            pwr = 0.0; 
            break; 
      }

      return pwr; 
   }

   /// Determine the power for right front wheel based on the driving mode
   double getRightFrontPower(int mode) {
      double pwr = 0; 
      switch( mode ) {
         case DRIVE_STOP:
         case DRIVE_DROP_JEWEL_ARM:
         case DRIVE_KNOCK_JEWEL:
            pwr = 0.0; 
            break; 
         case DRIVE_FORWARD:   
            pwr = -1.0*DRIVE_WHEEL_POWER; 
            break; 
         case DRIVE_BACKWARD:    
            pwr = DRIVE_WHEEL_POWER; 
            break; 
         case DRIVE_TURN_LEFT:
         case DRIVE_TURN_TO_LEFT:
            if( turn_slow_ ) {
               pwr = -1.0*TURN_SLOW_WHEEL_POWER; 
            } else {
               pwr = TURN_WHEEL_POWER; 
            }
            break;
         case DRIVE_TURN_RIGHT:
         case DRIVE_TURN_TO_RIGHT:
            if( turn_slow_ ) {
               pwr = TURN_SLOW_WHEEL_POWER; 
            } else {
               pwr = TURN_WHEEL_POWER; 
            }
            break;
         case DRIVE_RESET_ENC_DONE:       
            pwr = 0.0; 
            break; 
         case DRIVE_FORWARD_ENC:
         case DRIVE_FORWARD_ENC_GRAB:
            pwr = -1.0*DRIVE_ENC_WHEEL_POWER; 
            break; 
         case DRIVE_FORWARD_ENC_SLOW:
         case DRIVE_FORWARD_ENC_TO_WALL:
            pwr = -1.0*DRIVE_ENC_SLOW_WHEEL_POWER;
            break; 
         case DRIVE_BACKWARD_ENC:
            pwr = DRIVE_ENC_WHEEL_POWER; 
            break; 
         case DRIVE_BACKWARD_ENC_SLOW:
         case DRIVE_BACKWARD_ENC_TO_WALL:
            pwr = DRIVE_ENC_SLOW_WHEEL_POWER;
            break; 
         case DRIVE_TURN_LEFT_ENC:   
            pwr = -1.0*DRIVE_ENC_TURN_WHEEL_POWER; 
            break; 
         case DRIVE_TURN_RIGHT_ENC:   
            pwr = DRIVE_ENC_TURN_WHEEL_POWER; 
            break;
         case DRIVE_CHECK_DISTANCE:
            pwr = 0.0;
            break;
         default: 
            pwr = 0.0; 
            break; 
      }

      return pwr; 
   }

   /// Generalized function to determine the robot drive mode based on time
   int getDriveMode(double[] states, double time) {
      int tot_state = states.length/2; 
      return getDriveModeGyroEncTime(tot_state, states, time);
   } 

   /// Go to next state, and update starting conditions
   void gotoNextState(
         int tot_state,            // total number of states
         double[] states,          // states
         double time,              // current time
         boolean reset_encoders)   // reset encoders if true
   {
      curr_state_id_ ++; 
      curr_state_start_t_ = time; 

      curr_state_start_h_ = heading_; 
      curr_state_enc_cnt_ = 0.0; 
      int mode = DRIVE_STOP; 
      if( curr_state_id_<tot_state ) {
         mode = (int)( states[ curr_state_id_*2 +1 ] ); 
         curr_state_value_ = states[ curr_state_id_*2 ]; 
      }
      curr_state_drive_mode_ = mode; 
      if( AUTO_CORRECT_HEADING && curr_state_id_<tot_state ) { 
         double deg = states[ curr_state_id_*2 ]; 
         if( mode==DRIVE_TURN_TO_LEFT || mode==DRIVE_TURN_TO_RIGHT ) {
            target_heading_ = deg;      // a specific heading
         } else if( mode==DRIVE_TURN_LEFT_ENC ) {
            target_heading_ += deg;     // left, heading increase
         } else if( mode==DRIVE_TURN_RIGHT_ENC ) {
            target_heading_ -= deg;     // turn right, heading decrease
         }
      }

      if( mode==DRIVE_CHECK_DISTANCE ) { // reset counters
         num_dist_ok_ = 0; 
         num_dist_far_ = 0; 
      }
      if( mode==DRIVE_DROP_JEWEL_ARM ) { // reset counters
         num_red_seen_ = 0; 
         num_blue_seen_ = 0; 
         sum_distance_ = 0.0; 

         num2_red_seen_ = 0; 
         num2_blue_seen_ = 0; 
         sum2_distance_ = 0.0; 
      }

      if( reset_encoders && USE_ENC_FOR_DRIVE ) {  // reset encoders for other states
         reset_drive_encoders();     
      }

      if( isRedTeam() && curr_state_id_>=states.length-3 && mode==DRIVE_FORWARD_ENC) {
         is_red_last_step_ = true;
      }
   }

   /// Generalized function to determine the robot drive mode based on time
   int getDriveModeGyroEncTime(int num_state, double[] states, double time) {
      int mode = DRIVE_STOP;
      if( curr_state_id_ >= num_state ) {
         // Should not happen, DRIVE_STOP
         if( USE_ENC_FOR_DRIVE ) {  
            run_using_encoders() ;   // use encoders for time based drive. Required for SDK 2.2.  
         }
      } else if( curr_state_id_>=0 && curr_state_id_<num_state ) {
         int m = (int)(states[ curr_state_id_*2 +1 ]); 

         if( m==DRIVE_WAIT_TILL ) {
            if( USE_ENC_FOR_DRIVE ) {  
               run_using_encoders() ;   // use encoders for time based drive. Required for SDK 2.2.  
            }
            double t =  states[ curr_state_id_*2 ] ;   // wait till this time 
            if( t<0.0 ) { t = AUTO_RUN_TIME+t; }       // negative means minus N seconds
            if( time>=t ) {  // time reached
               gotoNextState( num_state, states, time, true );
            } else {   // keep waiting
               mode = m; 
            } 
         } else if ( m==DRIVE_TURN_TO_LEFT || m==DRIVE_TURN_TO_RIGHT ) {
            if( USE_ENC_FOR_DRIVE ) {  // disable encoders for gryo based turning
               run_using_encoders() ;   // use encoders for time based drive. Required for SDK 2.2.  
            }
            getHeading(); 

            if( states[ curr_state_id_*2 ] == -1.0 ) {   // use "-1, DRIVE_TURN_TO_LEFT" to set last_turn_to_heading_ to current heading
               last_turn_to_heading_ = heading_; 
               gotoNextState( num_state, states, time, true );
            } else {
               //double tg_deg = Math.abs(states[ curr_state_id_*2 ]);
               double tg_deg = states[ curr_state_id_*2 ];
               if( tg_deg != last_turn_to_heading_ ) last_turn_to_heading_ = tg_deg ; 
               double to_turn_deg = Math.abs( heading_-tg_deg );
               if( (m == DRIVE_TURN_TO_LEFT && heading_ >= tg_deg ) ||    // MR Gryo, turn left, heading increase
                     (m == DRIVE_TURN_TO_RIGHT && heading_ <= tg_deg) )     // MR Gyro, turn right, heading decrease
               {
                  gotoNextState( num_state, states, time, true );
               } else {
                  if(SLOW_TURN_DEGREE>0.0 && to_turn_deg<SLOW_TURN_DEGREE) {
                     turn_slow_ = true;
                  }
                  mode = m;
               }
            }
         } else if ( USE_ENC_FOR_DRIVE && (
                  (m==DRIVE_RESET_ENC_DONE) || 
                  (m==DRIVE_TURN_LEFT_ENC) || (m==DRIVE_TURN_RIGHT_ENC) || 
                  (m==DRIVE_FORWARD_ENC) || (m==DRIVE_BACKWARD_ENC) ||
                  (m==DRIVE_FORWARD_ENC_GRAB) || 
                  (m==DRIVE_FORWARD_ENC_SLOW) || (m==DRIVE_BACKWARD_ENC_SLOW) || 
                  (m==DRIVE_FORWARD_ENC_TO_WALL) || (m==DRIVE_BACKWARD_ENC_TO_WALL)
                  ) ) {
            // Use encoder to control driving 
            if( m==DRIVE_RESET_ENC_DONE ) {   // wait till encoder is reset
               if( have_drive_encoders_reset() ) {  // reset done, go to next state
                  gotoNextState( num_state, states, time, false ); 
               } else { // stay in RESET_ENC state, wait for reseting to complete
                  if( AUTO_RESET_ENC_TIME>0 && time<(curr_state_start_t_+AUTO_RESET_ENC_TIME) ) {  // auto-reset encoders
                     reset_drive_encoders();     
                  } 
                  mode = m ;
               }
            } else {  // driving using encoders
               double tg_enc_cnt = Math.abs(states[ curr_state_id_*2 ]); 
               if( tg_enc_cnt==0.0 ) {
                  gotoNextState( num_state, states, time, /*reset_encoders*/false ); 
               }
               if( (m==DRIVE_BACKWARD_ENC) || (m==DRIVE_FORWARD_ENC ) || (m==DRIVE_FORWARD_ENC_GRAB) ||
                     (m==DRIVE_BACKWARD_ENC_SLOW) || (m==DRIVE_FORWARD_ENC_SLOW) ||
                     (m==DRIVE_BACKWARD_ENC_TO_WALL) || (m==DRIVE_FORWARD_ENC_TO_WALL)) {
                  if( tg_enc_cnt<10.0 ) {   // treat it as meters, convert it to encoder count
                     tg_enc_cnt = tg_enc_cnt * ENC_DIST_SCALE; 
                  }
               } else if( (m==DRIVE_TURN_LEFT_ENC) || (m==DRIVE_TURN_RIGHT_ENC) ) {
                  if( tg_enc_cnt<360.0 ) {   // treat it as degrees
                     tg_enc_cnt = tg_enc_cnt * ENC_DEG_SCALE ;
                  }
               }
               run_using_encoders();

               if( SMART_DRIVE_TURN || SMART_DRIVE_MOVE ) {
                  int lf_enc_pos = Math.abs(getLFEncoderPos()); 
                  if( SMART_DRIVE_TURN && 
                        (m==DRIVE_TURN_LEFT_ENC || m==DRIVE_TURN_RIGHT_ENC) && 
                        lf_enc_pos<tg_enc_cnt*SMART_DRIVE_TURN_ENC_RATIO ) {
                     smart_drive_on_ = true;
                  } else if( SMART_DRIVE_TURN && 
                        (m==DRIVE_FORWARD_ENC_TO_WALL || m==DRIVE_BACKWARD_ENC_TO_WALL ) && 
                        lf_enc_pos<tg_enc_cnt*SMART_DRIVE_MOVE_ENC_RATIO ) {
                     smart_drive_on_ = true;
                  } else if( SMART_DRIVE_MOVE && m!=DRIVE_RESET_ENC_DONE && 
                        lf_enc_pos<tg_enc_cnt*SMART_DRIVE_TURN_ENC_RATIO ) {
                     smart_drive_on_ = true;
                  }
               }

               double dis = 0.0;
               boolean wall_seen = false;
               if( (m==DRIVE_BACKWARD_ENC_TO_WALL || m==DRIVE_FORWARD_ENC_TO_WALL) ) { 
                  if( rev_rgb_range_!=null ) {
                     servo_jewel_pos = SERVO_JEWEL_DOWN_POS;    // keep jewel arm down
                     jewel_distance_ = rev_rgb_range_.getDistance(DistanceUnit.CM);
                     dis = jewel_distance_;
                     if( Double.isNaN(dis) ) {
                        wall_seen = true;
                     } else if( isRedTeam() && dis<BOX_WALL_DISTANCE ) {
                        wall_seen = true;
                     } else if( !isRedTeam() && dis>BOX_WALL_DISTANCE ) {
                        wall_seen = true;
                     }
                  } else if( mr_range_!=null ) {
                  dis = mr_range_.getDistance(DistanceUnit.CM);
                  if(dis>0.0) {
                     if(isRedTeam() && dis>MAX_WALL_DIS_RED1) {
                        wall_seen = true;
                     } else if( !isRedTeam() && dis>MAX_WALL_DIS_BLUE1 ) {
                        wall_seen = true;
                     } 
                     if( !wall_seen && WALL_PRE_BRAKE_DIST>0.0 ) {
                        if( isRedTeam() && dis<MIN_WALL_DIS_RED+WALL_PRE_BRAKE_DIST ) {
                           pre_brake_on_ = true;
                        } else if( !isRedTeam() && dis<MIN_WALL_DIS_RED+WALL_PRE_BRAKE_DIST ) {
                           pre_brake_on_ = true;
                        }
                     }
                  }
                  }
               }
               if( wall_seen ) {
                  reset_drive_encoders();
                  gotoNextState(num_state, states, time, false);
               } else if( have_drive_encoders_reached( tg_enc_cnt, tg_enc_cnt ) ) {  // reset encoders and go to next state
                  //RobotLog.i("DBG: EncPos: Mode=%d, Tgt=%d, LF=%d, LB=%d, RF=%d, RB=%d", m,(int)(tg_enc_cnt),motorLF_.getCurrentPosition(),motorLB_.getCurrentPosition(),motorRF_.getCurrentPosition(),motorRB_.getCurrentPosition());  // debug motor encoder
                  reset_drive_encoders();     
                  gotoNextState( num_state, states, time, false ); 
               } else { // keep driving
                  curr_state_enc_cnt_ = tg_enc_cnt; 
                  mode = m; 
               }
            } 
         } else if( m==DRIVE_CHECK_DISTANCE ) {
            // use range sensor for distance checking if available
            double dist = mr_range_.getDistance(DistanceUnit.CM);
            if( dist>MAX_RANGE ) { 
               // invalid reading, ignored
            } else if( dist>WALL_DISTANCE_MAX ) {         
               //too_far_ = true; 
               num_dist_far_ ++; 
            } else {
               //too_far_ = false;
               num_dist_ok_ ++; 
            } 
            double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
            if( time>=(curr_state_start_t_+period) ) {  // timeout
               if( num_dist_ok_>=num_dist_far_ ) {   // distance is OK, skip re-alignment
                  curr_state_id_ += 8;
               }
               gotoNextState( num_state, states, time, true );
            } else { // keep checking
               mode = m;
            }
         } else if (m == DRIVE_DROP_JEWEL_ARM) {
            servo_jewel_pos = SERVO_JEWEL_DOWN_POS;
            //servo_jewel_pos = 0.35;

            if( SCAN_JEWEL_WHEN_DROP_ARM ) {
               double distance = rev_rgb_range_.getDistance(DistanceUnit.CM);
               jewel_distance_ = distance; 
               double red = rev_rgb_range_.red();
               double blue = rev_rgb_range_.blue();
               double alpha = rev_rgb_range_.alpha();
               if( !Double.isNaN(distance) && (alpha > MIN_JEWEL_ALPHA) ) {
                  if (red > MIN_JEWEL_COLOR_RATIO * blue) {
                     num_red_seen_ ++; 
                     sum_distance_ += distance; 
                  } else if (blue > MIN_JEWEL_COLOR_RATIO * red) {
                     num_blue_seen_ ++; 
                     sum_distance_ += distance; 
                  }
               }

               if( USE_DOUBLE_RGB_FOR_JEWEL && rev_rgb_range2_!=null ) {
                  double distance2 = rev_rgb_range2_.getDistance(DistanceUnit.CM);
                  jewel2_distance_ = distance2; 
                  double red2 = rev_rgb_range2_.red();
                  double blue2 = rev_rgb_range2_.blue();
                  double alpha2 = rev_rgb_range2_.alpha();
                  if( !Double.isNaN(distance2) && (alpha2 > MIN_JEWEL_ALPHA) ) {
                     if (red2 > MIN_JEWEL_COLOR_RATIO * blue2) {
                        num2_red_seen_ ++; 
                        sum2_distance_ += distance2; 
                     } else if (blue2 > MIN_JEWEL_COLOR_RATIO * red2) {
                        num2_blue_seen_ ++; 
                        sum2_distance_ += distance2; 
                     }
                  }
               }
            }

            double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
            if( time>=(curr_state_start_t_+period) ) {  // timeout
               gotoNextState( num_state, states, time, true );
            } else { // keep checking
               mode = m;
            }

         } else if (m == DRIVE_KNOCK_JEWEL) {

            if (!red_jewel_ && !blue_jewel_) {
               if( Math.abs(num_red_seen_-num_blue_seen_)>=MIN_NUM_DIFF_JEWEL ) {
                  // enough evidence to determine the jewel color based on the scanning data when dropping arm
                  if( num_red_seen_>num_blue_seen_ ) {
                     red_jewel_ = true;
                     blue_jewel_ = false;
                  } else {
                     blue_jewel_ = true;
                     red_jewel_ = false;
                  }
               } else if( USE_DOUBLE_RGB_FOR_JEWEL && Math.abs(num2_red_seen_-num2_blue_seen_)>=MIN_NUM_DIFF_JEWEL ) {
                  // enough evidence to determine the jewel color based on the scanning data when dropping arm from 2nd RGB sensor
                  if( num2_red_seen_ < num2_blue_seen_ ) {
                     // reversed check because two RGB sensors are mounted back-to-back
                     red_jewel_ = true;
                     blue_jewel_ = false;
                  } else {
                     blue_jewel_ = true; 
                     red_jewel_ = false;
                  } 
               } else if( USE_DOUBLE_RGB_FOR_JEWEL && Math.abs((num_blue_seen_+num2_red_seen_)-(num_red_seen_+num2_blue_seen_)) >= MIN_NUM_DIFF_JEWEL ) {
                  // combine scaning results from both sensors
                  if( (num_red_seen_+num2_blue_seen_) > (num_blue_seen_+num2_blue_seen_) ) {
                     // reversed check because two RGB sensors are mounted back-to-back
                     red_jewel_ = true;
                     blue_jewel_ = false;
                  } else {
                     blue_jewel_ = true; 
                     red_jewel_ = false;
                  } 
               } else {
                  // jewel scanning data is not sufficient, re-check the color  
                  double distance = rev_rgb_range_.getDistance(DistanceUnit.CM);
                  double red = rev_rgb_range_.red();
                  double blue = rev_rgb_range_.blue();
                  double alpha = rev_rgb_range_.alpha();

                  if (!Double.isNaN(distance) && (alpha > MIN_JEWEL_ALPHA)) {
                     if (red > MIN_JEWEL_COLOR_RATIO * blue) {
                        red_jewel_ = true;
                        blue_jewel_ = false;
                     } else if (blue > MIN_JEWEL_COLOR_RATIO * red) {
                        blue_jewel_ = true;
                        red_jewel_ = false;
                     }
                  }

                  if( !red_jewel_ && !blue_jewel_ && USE_DOUBLE_RGB_FOR_JEWEL && rev_rgb_range2_!=null ) {
                     // re-check using 2nd RGB if re-check by first RGB failed
                     double distance2 = rev_rgb_range2_.getDistance(DistanceUnit.CM);
                     double red2 = rev_rgb_range2_.red();
                     double blue2 = rev_rgb_range2_.blue();
                     double alpha2 = rev_rgb_range2_.alpha();

                     if (!Double.isNaN(distance2) && (alpha2 > MIN_JEWEL_ALPHA)) {
                        if (red2 < MIN_JEWEL_COLOR_RATIO * blue2) {
                           // reversed check because two RGB sensors are mounted back-to-back
                           red_jewel_ = true;
                           blue_jewel_ = false;
                        } else if (blue2 < MIN_JEWEL_COLOR_RATIO * red2) {
                           blue_jewel_ = true;
                           red_jewel_ = false;
                        }
                     }
                  }
               }
            }

            if( red_jewel_ || blue_jewel_ ) {
               if( isRedTeam() ) {
                  servo_knock_pos = red_jewel_ ? SERVO_KNOCK_BACK_POS : SERVO_KNOCK_FRONT_POS ; 
                  servo_jewel_pos = SERVO_JEWEL_DOWN_POS;
               } else {
                  servo_knock_pos = blue_jewel_ ? SERVO_KNOCK_BACK_POS : SERVO_KNOCK_FRONT_POS ; 
                  servo_jewel_pos = SERVO_JEWEL_DOWN_POS;
               } 
               jewel_knocked_ = true;
            }

            double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
            if( time>=(curr_state_start_t_+period) ) {  // timeout
               gotoNextState( num_state, states, time, true );
            } else { // keep checking
               mode = m;
            }
         }
         else if (m == DRIVE_PICTOGRAPH){
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
               pictograph_ = vuMark;
               //gotoNextState(num_state, states, time, true);
            }
            if(USE_LIFT) {
               if( USE_INTAKE_SERVO_CR ) {
                  target_lift_enc_ = 0; 
                  servo_left_jaw2_pos_ = LEFT_JAW_CLOSE;
                  servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE;
               } else {
                  target_lift_enc_ = (int)(LIFT_ENC_COUNT_PER_GLYPH/2);
                  servo_left_jaw_pos_ = LEFT_JAW_CLOSE;
                  servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
               }
            }
            double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
            if( time>=(curr_state_start_t_+period) ) {  // timeout
               gotoNextState( num_state, states, time, true );
            } else { // keep checking
               mode = m;
            }
         }
         else if (m == DRIVE_KEY_COLUMN ) {
            // flow control for position 1
            if( pictograph_==RelicRecoveryVuMark.CENTER ) {
               if( isRedTeam() ) {   
                  curr_state_id_ += 7; //skip code for putting it in right column
               } else {
                  curr_state_id_ += 7; //skip code for putting it in left column
               }
            } else if (pictograph_ == RelicRecoveryVuMark.RIGHT){
               if( isRedTeam() ) {
               } else {
                  curr_state_id_ += 12;
               }
            } else if (pictograph_ == RelicRecoveryVuMark.LEFT){
               if( isRedTeam() ) {   
                  curr_state_id_ += 12;
               } else {
               }
            } else { // RelicRecoveryVuMark.UNKNOWN
               if( isRedTeam() ) {   
                  curr_state_id_ += 12;  // LEFT for RED1
               } else {
                  curr_state_id_ += 12;  // RIGHT for BLUE1
               }
            }
            gotoNextState( num_state, states, time, true );
         }    
         else if (m == DRIVE_KEY_COLUMN3 ) {
            // flow control for position 1 for QT 
            if( pictograph_==RelicRecoveryVuMark.CENTER ) {
               if( isRedTeam() ) {   
                  curr_state_id_ += 13; //skip code for putting it in right column
               } else {
                  curr_state_id_ += 13; //skip code for putting it in left column
               }
            } else if (pictograph_ == RelicRecoveryVuMark.RIGHT){
               if( isRedTeam() ) {
               } else {
                  curr_state_id_ += 22;
               }
            } else if (pictograph_ == RelicRecoveryVuMark.LEFT){
               if( isRedTeam() ) {   
                  curr_state_id_ += 22;
               } else {
               }
            } else { // RelicRecoveryVuMark.UNKNOWN
               if( isRedTeam() ) {   
                  curr_state_id_ += 22;  // LEFT for RED1
               } else {
                  curr_state_id_ += 22;  // RIGHT for BLUE1
               }
            }
            gotoNextState( num_state, states, time, true );
         }    
         else if (m == DRIVE_KEY_COLUMN2) {
            // flow control for position 2
            if( pictograph_==RelicRecoveryVuMark.CENTER ){
               if( isRedTeam() ) {   
                  curr_state_id_ += 4; //skip code for putting it in right column
               } else {
                  curr_state_id_ += 4; //skip code for putting it in left column
               }
            } else if (pictograph_ == RelicRecoveryVuMark.RIGHT){
               if( isRedTeam() ) {
               } else {
                  curr_state_id_ += 8;
               }
            } else if (pictograph_ == RelicRecoveryVuMark.LEFT){
               if( isRedTeam() ) {   
                  curr_state_id_ += 8;
               } else {
               }
            } else { // RelicRecoveryVuMark.UNKNOWN
               if( isRedTeam() ) {   
                  curr_state_id_ += 8;  // LEFT for RED2
               } else {
                  curr_state_id_ += 8;  // RIGHT for BLUE2
               }
            }
            gotoNextState( num_state, states, time, true );
         }
         else if (m == DRIVE_KEY_COLUMN4) {
            // flow control for position 2 for SQ
            if( pictograph_==RelicRecoveryVuMark.CENTER ){
               if( isRedTeam() ) {   
                  curr_state_id_ += 8; //skip code for putting it in right column
               } else {
                  curr_state_id_ += 8; //skip code for putting it in left column
               }
            } else if (pictograph_ == RelicRecoveryVuMark.RIGHT){
               if( isRedTeam() ) {
               } else {
                  curr_state_id_ += 12;
               }
            } else if (pictograph_ == RelicRecoveryVuMark.LEFT){
               if( isRedTeam() ) {   
                  curr_state_id_ += 12;
               } else {
               }
            } else { // RelicRecoveryVuMark.UNKNOWN
               if( isRedTeam() ) {   
                  curr_state_id_ += 12;  // LEFT for RED2
               } else {
                  curr_state_id_ += 12;  // RIGHT for BLUE2
               }
            }
            gotoNextState( num_state, states, time, true );
         }
         else if (m == DRIVE_KEY_COLUMN5 ) {
            // flow control for position 1 for OR
            if( pictograph_==RelicRecoveryVuMark.CENTER ) {
               if( isRedTeam() ) {   
                  curr_state_id_ += 17; //skip code for putting it in right column
               } else {
                  curr_state_id_ += 13; //skip code for putting it in left column
               }
            } else if (pictograph_ == RelicRecoveryVuMark.RIGHT){
               if( isRedTeam() ) {
               } else {
                  //curr_state_id_ += 34; // 13 for LEFT,  21 for CENTER
                  curr_state_id_ += 38; // 13 for LEFT,  25 for CENTER 
               }
            } else if (pictograph_ == RelicRecoveryVuMark.LEFT){
               if( isRedTeam() ) {   
                  //curr_state_id_ += 26;  // 17 for RIGHT , 9 for CENTER
                  curr_state_id_ += 30;  // 17 for RIGHT , 13 for CENTER, 4 steps to drop and pick up 
               } else {
               }
            } else { // RelicRecoveryVuMark.UNKNOWN
               if( isRedTeam() ) {   
                  //curr_state_id_ += 26;  // LEFT for RED1
                  curr_state_id_ += 30;  // 17 for RIGHT , 13 for CENTER, 4 steps to drop and pick up 
               } else {
                  //curr_state_id_ += 22;  // RIGHT for BLUE1
                  curr_state_id_ += 0;  // LEFT for BLUE1
               }
            }
            gotoNextState( num_state, states, time, true );
         }    
         else if (m == DRIVE_KEY_COLUMN7 ) {
            // flow control for position 1 for OR
            if( pictograph_==RelicRecoveryVuMark.CENTER ) {
               if( isRedTeam() ) {   
                  //curr_state_id_ += 17; //skip code for putting it in right column
                  curr_state_id_ += 19; //skip code for putting it in right column, 2018/03/02  
               } else {
                  //curr_state_id_ += 13; //skip code for putting it in left column
                  curr_state_id_ += 15; //skip code for putting it in left column, 2018/03/02 
               }
            } else if (pictograph_ == RelicRecoveryVuMark.RIGHT){
               if( isRedTeam() ) {
               } else {
                  //curr_state_id_ += 40; // 13 for LEFT,  27 for CENTER; backup and then forward, 2018/02/28 
                  curr_state_id_ += 15 + 27 ; // 15 for LEFT,  27 for CENTER; backup and then forward, 2018/03/02
               }
            } else if (pictograph_ == RelicRecoveryVuMark.LEFT){
               if( isRedTeam() ) {   
                  //curr_state_id_ += 41;  // 17 for RIGHT, 24 for CENTER
                  curr_state_id_ += 19 + 24;  // 19 for RIGHT, 24 for CENTER. 2018/03/02 
               } else {
               }
            } else { // RelicRecoveryVuMark.UNKNOWN
               if( isRedTeam() ) {   
                  curr_state_id_ += 19 + 24;  // LEFT for RED1;  19 for RIGHT, 24 for CENTER, 2018/03/02 
               } else {
                  curr_state_id_ += 0;  // LEFT for BLUE1
               }
            }
            gotoNextState( num_state, states, time, true );
         }    
         else if (m == DRIVE_KEY_COLUMN6) {
            // flow control for position 2 for OR
            if( pictograph_==RelicRecoveryVuMark.CENTER ){
               if( isRedTeam() ) {   
                  curr_state_id_ += 21; // RIGHT has 21 steps
               } else {
                  curr_state_id_ += 21;  // LEFT has 21 steps
               }
            } else if (pictograph_ == RelicRecoveryVuMark.RIGHT){
               if( isRedTeam() ) {
               } else {
                  //curr_state_id_ += 21 + 27;   // LEFT 21 steps, CENTER 27 steps
                  curr_state_id_ += 21 + 31;   // LEFT 21 steps, CENTER 27 steps (+4)
               }
            } else if (pictograph_ == RelicRecoveryVuMark.LEFT){
               if( isRedTeam() ) {   
                  //curr_state_id_ += 21 + 25;
                  curr_state_id_ += 21 + 29;  // 4 steps for drop/pick up glyph
               } else {
               }
            } else { // RelicRecoveryVuMark.UNKNOWN
               if( isRedTeam() ) {   
                  // RIGHT is preferred for RED2
               } else {
                  // LEFT is preferred for BLUE2
               }
            }
            gotoNextState( num_state, states, time, true );
         }
         else if (m == DRIVE_KEY_COLUMN8) {
            // flow control for position 2 for SR
            if( pictograph_==RelicRecoveryVuMark.CENTER ){
               if( isRedTeam() ) {   
                  //curr_state_id_ += 21; // RIGHT has 21 steps
                  curr_state_id_ += 23; // RIGHT has 21+2 steps, 2018/03/02 
               } else {
                  //curr_state_id_ += 21;  // LEFT has 21 steps
                  curr_state_id_ += 23;  // LEFT has 23 steps, 2018/03/02 
               }
            } else if (pictograph_ == RelicRecoveryVuMark.RIGHT){
               if( isRedTeam() ) {
               } else {
                  //curr_state_id_ += 21 + 35;   // LEFT 21 steps, CENTER 27+8 steps
                  curr_state_id_ += 23 + 35;   // LEFT 21+2 steps, CENTER 27+8 steps, 2018/03/02 
               }
            } else if (pictograph_ == RelicRecoveryVuMark.LEFT){
               if( isRedTeam() ) {   
                  //curr_state_id_ += 21 + 26;  // new trip for CENTER with 26 steps
                  curr_state_id_ += 23 + 26;  // new trip for CENTER with 26 steps
               } else {
               }
            } else { // RelicRecoveryVuMark.UNKNOWN
               if( isRedTeam() ) {   
                  // RIGHT is preferred for RED2
               } else {
                  // LEFT is preferred for BLUE2
               }
            }
            gotoNextState( num_state, states, time, true );
         }
         else if (m == DRIVE_OPEN_CLAW ) {
            servo_left_jaw_pos_ = LEFT_JAW_OPEN;
            servo_right_jaw_pos_ = RIGHT_JAW_OPEN;
            servo_left_jaw2_pos_ = LEFT_JAW_OPEN;
            servo_right_jaw2_pos_ = RIGHT_JAW_OPEN;

            if( USE_PUSHER_AR ) servo_pusher_pos_ = PUSHER_PUSH; 
            if( USE_LIFT ) {  // lower the claw
               target_lift_enc_ = 0;
/*
               int lift_enc = motorLift_.getCurrentPosition(); 
               if( lift_enc>MIN_LIFT_ENC_COUNT ) {
                  power_lift = LIFT_DOWN_POWER; 
                  target_lift_enc_ = 0;
               } 
*/
            }


            double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
            if( time>=(curr_state_start_t_+period) ) {  // timeout
               gotoNextState( num_state, states, time, true );
               last_open_claw_end_t_ = time; 
            } else { // keep checking
               mode = m;
            }
         }
         else if (m == DRIVE_OPEN_CLAW_ONLY ) {
            servo_left_jaw_pos_ = LEFT_JAW_OPEN;
            servo_right_jaw_pos_ = RIGHT_JAW_OPEN;
            servo_left_jaw2_pos_ = LEFT_JAW_OPEN;
            servo_right_jaw2_pos_ = RIGHT_JAW_OPEN;

            if( USE_LIFT ) {  // lower the claw
               target_lift_enc_ = 0;
            }

            double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
            if( time>=(curr_state_start_t_+period) ) {  // timeout
               gotoNextState( num_state, states, time, true );
               last_open_claw_end_t_ = time;    // retract claw
            } else { // keep checking
               mode = m;
            }
         }
         else if (m == DRIVE_CLOSE_CLAW ){
            servo_left_jaw_pos_ = LEFT_JAW_CLOSE;
            servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
            servo_left_jaw2_pos_ = LEFT_JAW_CLOSE;
            servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE;

            double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
            if( time>=(curr_state_start_t_+period) ) {  // timeout
               gotoNextState( num_state, states, time, true );
            } else { // keep checking
               mode = m;
            }
         }
         else if (m == DRIVE_SHIFT_CLAW_LEFT ){
            servo_left_jaw_pos_ = LEFT_JAW_OPEN_SLOW;
            servo_right_jaw_pos_ = RIGHT_JAW_CLOSE;
            servo_left_jaw2_pos_ = LEFT_JAW_OPEN_SLOW;
            servo_right_jaw2_pos_ = RIGHT_JAW_CLOSE;

            double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
            boolean timeout = (time>=(curr_state_start_t_+period)); 
            if( USE_INTAKE_SERVO_CR ) { timeout = true; }    // glyph hold by top claw
            if( timeout ) {  // timeout
               target_lift_enc_ = 0;     // lower the glyph after shifting
               gotoNextState( num_state, states, time, true );
            } else { // keep checking
               mode = m;
            }
         }
         else if (m == DRIVE_SHIFT_CLAW_RIGHT ){
            servo_left_jaw_pos_ = LEFT_JAW_CLOSE;
            servo_right_jaw_pos_ = RIGHT_JAW_OPEN_SLOW;
            servo_left_jaw2_pos_ = LEFT_JAW_CLOSE;
            servo_right_jaw2_pos_ = RIGHT_JAW_OPEN_SLOW;

            double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
            boolean timeout = (time>=(curr_state_start_t_+period)); 
            if( USE_INTAKE_SERVO_CR ) { timeout = true; }    // glyph hold by top claw
            if( timeout ) {
               target_lift_enc_ = 0;     // lower the glyph after shifting
               gotoNextState( num_state, states, time, true );
            } else { // keep checking
               mode = m;
            }
         }
         else if (m == DRIVE_STATE_JUMP ){
            int jump = (int) ( states[ curr_state_id_*2 ] );   // jump
            if( jump==1 ) {
               // automatically goto NEXT state
            } else if( jump>1 ) {
               curr_state_id_ += jump-1 ; 
            } else if ( jump<-1000 ) {
               // -1001, last step; -1002 second last
               jump += 1000; 
               curr_state_id_ = num_state + jump - 1 ;
            } else if ( jump<0 ) {
               // jump back
               curr_state_id_ += jump-1 ; 
            }
            gotoNextState( num_state, states, time, true );
         }
         else if (m == DRIVE_SHIFT_GEAR ){
            double f = states[ curr_state_id_*2 ]; 
            if( f>0.33 && f<=3.0 ) {
               drive_power_factor_ = f; 
            }
            gotoNextState( num_state, states, time, true );
         }
         else if (m == DRIVE_SWITCH_INTAKE ) {
            int s = (int)(states[ curr_state_id_*2 ]); 
            if( s>0 && s<=3 ) {
               if( s!=intake_curr_state_ ) { intake_prev_state_ = intake_curr_state_; }
               intake_curr_state_ = s; 
            }
            gotoNextState( num_state, states, time, true );
         }
         else if (m == DRIVE_LIFT_CLAWS ){
            int lift_enc = (int)states[ curr_state_id_*2 ]; 
            if( lift_enc>=0 && lift_enc<LIFT_ENC_COUNT_PER_GLYPH ) {
               target_lift_enc_ = lift_enc; 
            }
            gotoNextState( num_state, states, time, true );
         }
         else {
            // Time based drive modes: STOP/FORWARD/BACKWARD/TURN_LEFT/TURN_RIGHT
            if( USE_ENC_FOR_DRIVE ) {  
               run_using_encoders() ;   // use encoders for time based drive. Required for SDK 2.2.  
            }
            double period = Math.abs( states[ curr_state_id_*2 ] );   // time limit
            if( time>=(curr_state_start_t_+period) ) {  // timeout
               gotoNextState( num_state, states, time, true );
            } else { // keep going
               mode = m;
            }
         }
      } else {  
         // curr_state_id_=-1; hanlde initial wait time
         if( USE_ENC_FOR_DRIVE ) {  
            run_using_encoders() ;   // use encoders for time based drive. Required for SDK 2.2.  
         }

         if( time<init_wait_time_ ) {
            mode = DRIVE_STOP;      // delayed start
         } else {   // initial wait time passes, go to first state
            double period = states[0]; 
            mode = (int)(states[1]); 
            gotoNextState( num_state, states, time, true );    // go to first state, curr_state_id_: -1 => 0
         }
      }

      return mode; 
   } 



  ///////TRIPS///////***************************************************************************************************************************************
   /// Define red trip
   int getDriveModeRed(double t) {

      double[] testTrip1 = { 
      // Test/calibrate distance for encoder based drive 
         1.0, DRIVE_STOP,                    
         2.9, DRIVE_FORWARD_ENC,      // 1.0m
         10.0, DRIVE_STOP,                    
         3.0, DRIVE_BACKWARD_ENC,     // 1.0m
         10.0, DRIVE_STOP,     
         } ; 

      double[] testTrip1A = { 
      // Test/calibrate distance for encoder based drive 
         1.0, DRIVE_STOP,                    
         3.0, DRIVE_FORWARD_ENC,      // 3.0m
         1.0, DRIVE_STOP,                    
         3.0, DRIVE_BACKWARD_ENC,     // 3.0m
         3.0, DRIVE_STOP,                    
         3.0, DRIVE_FORWARD_ENC,      // 3.0m
         1.0, DRIVE_STOP,                    
         3.0, DRIVE_BACKWARD_ENC,     // 3.0m
         3.0, DRIVE_STOP,                    
         3.0, DRIVE_FORWARD_ENC,      // 3.0m
         1.0, DRIVE_STOP,                    
         3.0, DRIVE_BACKWARD_ENC,     // 3.0m
         10.0, DRIVE_STOP,     
         } ; 

      double[] testTrip1B = { 
      // Test/calibrate distance for encoder based drive 
         1.0, DRIVE_STOP,                    
         0.6, DRIVE_FORWARD_ENC,      // 3.0m
         1.0, DRIVE_STOP,                    
         3.0, DRIVE_BACKWARD_ENC,     // 3.0m
         3.0, DRIVE_STOP,                    
         3.0, DRIVE_FORWARD_ENC,      // 3.0m
         1.0, DRIVE_STOP,                    
         3.0, DRIVE_BACKWARD_ENC,     // 3.0m
         3.0, DRIVE_STOP,                    
         3.0, DRIVE_FORWARD_ENC,      // 3.0m
         1.0, DRIVE_STOP,                    
         3.0, DRIVE_BACKWARD_ENC,     // 3.0m
         10.0, DRIVE_STOP,     
         } ; 

      double[] testTrip2 = {
      // Test encoder base drive and turn
         1.0, DRIVE_STOP,                    

         0.1, DRIVE_RESET_ENC_DONE,  
         2000, DRIVE_FORWARD_ENC,   
         3.0, DRIVE_STOP, 
         0.1, DRIVE_RESET_ENC_DONE,               
         2000, DRIVE_BACKWARD_ENC,           
         3.0, DRIVE_STOP,                  

         0.1, DRIVE_RESET_ENC_DONE,       
         2000, DRIVE_TURN_LEFT_ENC,      
         3.0, DRIVE_STOP,              
         0.1, DRIVE_RESET_ENC_DONE,       
         2000, DRIVE_TURN_RIGHT_ENC,      
         3.0, DRIVE_STOP,              

         0.1, DRIVE_RESET_ENC_DONE,  
         1.0, DRIVE_FORWARD_ENC,      // 1.0m
         3.0, DRIVE_STOP, 
         0.1, DRIVE_RESET_ENC_DONE,               
         1.0, DRIVE_BACKWARD_ENC,     // 1.0m
         3.0, DRIVE_STOP,                  

         0.1, DRIVE_RESET_ENC_DONE,   
         90,  DRIVE_TURN_RIGHT_ENC, 
         3.0, DRIVE_STOP,              
         0.1, DRIVE_RESET_ENC_DONE, 
         90,  DRIVE_TURN_LEFT_ENC,      
         3.0, DRIVE_STOP,              
         0.1, DRIVE_RESET_ENC_DONE,   
         180, DRIVE_TURN_RIGHT_ENC, 
         1.0, DRIVE_STOP,         
         9.0, DRIVE_STOP          
      }; 
      
      double[] testTrip3A = { 
      // Test simple trip
         1.0, DRIVE_STOP,                    
         0.1, DRIVE_RESET_ENC_DONE,  
         0.6, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,  
         45,  DRIVE_TURN_LEFT_ENC,      
         0.1, DRIVE_RESET_ENC_DONE,  
         1.7, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,  
         45,  DRIVE_TURN_LEFT_ENC,      
         0.1, DRIVE_RESET_ENC_DONE,  
         1.2, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,  
         90,  DRIVE_TURN_RIGHT_ENC,      
         0.1, DRIVE_RESET_ENC_DONE,  
         1.8, DRIVE_BACKWARD_ENC, 
         10.0, DRIVE_STOP,                    
      } ;

      double[] testTrip8 = {
         // Test CHECK_DISTANCE using range sensor
         1.0, DRIVE_STOP,                       //
         0.1, DRIVE_CHECK_DISTANCE,             // check if robot is too far
         0.01,DRIVE_RESET_ENC_DONE,             // adjust robot:  turn right, back up 10cm, turn left, stop at white
         45,  DRIVE_TURN_RIGHT_ENC,
         0.01,DRIVE_RESET_ENC_DONE,
         0.10,DRIVE_BACKWARD_ENC,               // input backup distance?
         0.01,DRIVE_RESET_ENC_DONE,
         45,  DRIVE_TURN_LEFT_ENC,
         0.01,DRIVE_RESET_ENC_DONE,
         10.0, DRIVE_STOP
      }; 

      double[] testTrip9 = {
         // Test driving to wall using range sensor
         1.1, DRIVE_STOP,//
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_FORWARD_ENC_TO_WALL,        // stop at wall with heading correction
         0.1, DRIVE_RESET_ENC_DONE,
         0.5, DRIVE_BACKWARD_ENC, 
         0.1, DRIVE_RESET_ENC_DONE,
         90,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_BACKWARD_ENC_TO_WALL,        // stop at wall with heading correction
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_STOP,                    
         10.0, DRIVE_STOP
      };


      double[] testTrip11 = {
         // Test heading correction on IMU heading boundary 180/-180
         1.0, DRIVE_STOP,
         0.1, DRIVE_RESET_ENC_DONE,
         145, DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         3.0, DRIVE_STOP,
         45,  DRIVE_TURN_LEFT_ENC,     // target 190, IMU -170
         0.1, DRIVE_RESET_ENC_DONE,
         3.0, DRIVE_STOP,
         10,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         3.0, DRIVE_STOP,
         1.1, DRIVE_FORWARD_ENC,       // check heading correction
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_STOP,
         10.0, DRIVE_STOP
      };

      double [] testTrip12 = {
         // Test jewel mission
         1.0, DRIVE_STOP, 
         //1.0, DRIVE_DROP_JEWEL_ARM, 
         //1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,  
         1.0, DRIVE_FORWARD_ENC,
         //1.0, DRIVE_STOP, 

         0.1, DRIVE_RESET_ENC_DONE,  
         0.15, DRIVE_FORWARD_ENC,   
         //1.0, DRIVE_STOP, 

         0.1, DRIVE_RESET_ENC_DONE,  
         0.15, DRIVE_FORWARD_ENC,   
         1.0, DRIVE_STOP, 
         10.0, DRIVE_STOP 
      }; 

      double [] redTrip0 = {
         // Red position2: Jewel and key column
         0.1, DRIVE_STOP,
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_STOP,

         10.0, DRIVE_STOP
      };

      double [] redTrip1 = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // new trip with auto encoder resetting and drop-push, 2017/11/10 
         0.1, DRIVE_STOP,                                   
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         //1.0, DRIVE_FORWARD_ENC_TO_WALL,      // not reliable
         0.75, DRIVE_FORWARD_ENC_SLOW,     
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN,    // pick key column 

         //RIGHT COLUMN
         //75,  DRIVE_TURN_RIGHT_ENC,
         //0.30, DRIVE_FORWARD_ENC_SLOW,  
         0.40, DRIVE_FORWARD_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,
         120,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.3, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_LEFT,
         -1011, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         //55,  DRIVE_TURN_RIGHT_ENC,
         50,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,
         -1011, DRIVE_STATE_JUMP,           

         //LEFT COLUMN
         0.20, DRIVE_FORWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         //55,  DRIVE_TURN_RIGHT_ENC,
         50,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,    
         0.35, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,

         // drop glyph
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,

         // push the glyph
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_CLOSE_CLAW,
         0.2, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.1, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_OPEN_CLAW,

         60.0, DRIVE_STOP             
      };

/*
      double [] redTrip1QT = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // new trip with auto encoder resetting and drop-push, 2017/11/10 
         // LQT trip, jewel, key column, and fetching extra glyph, 2017/08/16
         0.1, DRIVE_STOP,                                   
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         0.75, DRIVE_FORWARD_ENC_SLOW,     
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN3,    // pick key column 

         //RIGHT COLUMN
         0.40, DRIVE_FORWARD_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,
         120,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.3, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT,
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to glyph pit
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         140, DRIVE_TURN_RIGHT_ENC,
         -1006, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         50,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.25, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,

         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_BACKWARD_ENC, 
         30, DRIVE_STOP,   // stop here
         -1006, DRIVE_STATE_JUMP,           

         //LEFT COLUMN
         0.20, DRIVE_FORWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         50,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,    
         0.25, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to pit 
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         140, DRIVE_TURN_LEFT_ENC,

         /// Try to grab some glyph
         0.1, DRIVE_RESET_ENC_DONE,
         //0.7, DRIVE_FORWARD_ENC,
         //0.7, DRIVE_FORWARD_ENC_GRAB,
         0.4, DRIVE_FORWARD_ENC_GRAB,
         1.0, DRIVE_CLOSE_CLAW,

         // Back to safe zone
         0.1, DRIVE_RESET_ENC_DONE,
         //0.8, DRIVE_BACKWARD_ENC,
         0.5, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };
*/

      double [] redTrip1QT = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // new trip with auto encoder resetting and drop-push, 2017/11/10 
         // LQT trip, jewel, key column, and fetching extra glyph, 2017/08/16
         // New with relic slide, 2018/02/10
         0.1, DRIVE_STOP,                                   
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         0.75, DRIVE_FORWARD_ENC_SLOW,     
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN3,    // pick key column 

         //RIGHT COLUMN
         0.40, DRIVE_FORWARD_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,
         120,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.3, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT,
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to glyph pit
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         140, DRIVE_TURN_RIGHT_ENC,
         -1006, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         //50,  DRIVE_TURN_RIGHT_ENC,  // WM9
         53,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.25, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,

         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_BACKWARD_ENC, 
         30, DRIVE_STOP,   // stop here
         -1006, DRIVE_STATE_JUMP,           

         //LEFT COLUMN
         0.20, DRIVE_FORWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         //50,  DRIVE_TURN_RIGHT_ENC,  // WM9
         53,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,    
         0.25, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to pit 
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         140, DRIVE_TURN_LEFT_ENC,

         /// Try to grab some glyph
         0.1, DRIVE_RESET_ENC_DONE,
         //0.7, DRIVE_FORWARD_ENC,
         //0.7, DRIVE_FORWARD_ENC_GRAB,
         0.4, DRIVE_FORWARD_ENC_GRAB,
         1.0, DRIVE_CLOSE_CLAW,

         // Back to safe zone
         0.1, DRIVE_RESET_ENC_DONE,
         //0.8, DRIVE_BACKWARD_ENC,
         0.5, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };

      double [] redTrip1ORTest = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // new trip with auto encoder resetting and drop-push, 2017/11/10 
         // LQT trip, jewel, key column, and fetching extra glyph, 2017/08/16
         // New with relic slide, 2018/02/10
         // Try to get more glyphs by new intake system, 2018/02/18
         0.1, DRIVE_STOP,                                   
         0.2, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         0.7, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         //0.75, DRIVE_FORWARD_ENC_SLOW,     
         0.70, DRIVE_FORWARD_ENC,     
         0.1, DRIVE_RESET_ENC_DONE,
         0.8, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_FORWARD_ENC_TO_WALL, 
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN3,    // pick key column 

         //RIGHT COLUMN
         0.35, DRIVE_FORWARD_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,
         120,  DRIVE_TURN_RIGHT_ENC,
         //0.15, DRIVE_BACKWARD_ENC,    /// not reliable
         //0.1, DRIVE_RESET_ENC_DONE,
         //65,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.3, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT,
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to glyph pit
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         140, DRIVE_TURN_RIGHT_ENC,
         -1006, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         //53,  DRIVE_TURN_RIGHT_ENC,  // WM9
         60,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.25, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,

         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_BACKWARD_ENC, 
         30, DRIVE_STOP,   // stop here
         -1006, DRIVE_STATE_JUMP,           

         //LEFT COLUMN
         //0.20, DRIVE_FORWARD_ENC_SLOW,  
         0.10, DRIVE_FORWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         53,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,    
         0.25, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to pit 
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         140, DRIVE_TURN_LEFT_ENC,

         /// Try to grab some glyph
         0.1, DRIVE_RESET_ENC_DONE,
         //0.7, DRIVE_FORWARD_ENC_GRAB,
         0.4, DRIVE_FORWARD_ENC_GRAB,
         1.0, DRIVE_CLOSE_CLAW,

         // Back to safe zone
         0.1, DRIVE_RESET_ENC_DONE,
         //0.8, DRIVE_BACKWARD_ENC,
         0.5, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };

      double [] redTrip1OR = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // new trip with auto encoder resetting and drop-push, 2017/11/10 
         // LQT trip, jewel, key column, and fetching extra glyph, 2018/01/16
         // OR move glyph from CENTER to LEFT, 2018/02/20
         // Try to get more glyphs by new intake system, 2018/02/21
         0.1, DRIVE_STOP,                                   
         0.2, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         0.7, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         //0.70, DRIVE_FORWARD_ENC,       // not very reliable
         1.5, DRIVE_SHIFT_GEAR, 
         0.75, DRIVE_FORWARD_ENC_SLOW,     
         1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,
         0.8, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_FORWARD_ENC_TO_WALL, 
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN3,    // pick key column 

         //RIGHT COLUMN
         //0.35, DRIVE_FORWARD_ENC,   // old long trip, less reliable
         //0.20, DRIVE_FORWARD_ENC,  
         //0.1, DRIVE_RESET_ENC_DONE,
         //120,  DRIVE_TURN_RIGHT_ENC,

         0.15, DRIVE_BACKWARD_ENC,    // new short trip with better reliability
         0.1, DRIVE_RESET_ENC_DONE,
         65,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         //0.3, DRIVE_FORWARD_ENC_SLOW,
         0.20, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT,
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to glyph pit
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         140, DRIVE_TURN_RIGHT_ENC,
         -1006, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         //60,  DRIVE_TURN_RIGHT_ENC,  // WM9
         //0.1, DRIVE_RESET_ENC_DONE,   
         //0.25, DRIVE_FORWARD_ENC_SLOW,
         //0.5, DRIVE_SHIFT_CLAW_RIGHT,

         //0.1, DRIVE_RESET_ENC_DONE,    
         //0.5, DRIVE_OPEN_CLAW,
         //0.1, DRIVE_BACKWARD_ENC, 
         //30, DRIVE_STOP,   // stop here
         //-1006, DRIVE_STATE_JUMP,           

         // MIDDLE COLUMN, break CENTER, then move the glyph to LEFT 
         55,  DRIVE_TURN_RIGHT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,   
         0.30, DRIVE_FORWARD_ENC_SLOW,  // break CENTER

         0.1, DRIVE_RESET_ENC_DONE,   
         0.25, DRIVE_BACKWARD_ENC_SLOW, // back off
         0.1, DRIVE_RESET_ENC_DONE,   
         55,  DRIVE_TURN_LEFT_ENC,      // restore heading
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_STOP,               // ensure that it's 9 steps for CENTER to match DRIVE_KEY_COLUMN3
         // fall through to LEFT COLUMN trips 

         //LEFT COLUMN
         //0.20, DRIVE_FORWARD_ENC_SLOW,  
         //0.10, DRIVE_FORWARD_ENC_SLOW,  
         0.15, DRIVE_FORWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         //53,  DRIVE_TURN_RIGHT_ENC,  // WM9
         50,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,    
         //0.25, DRIVE_FORWARD_ENC_SLOW,
         0.30, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to pit 
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         140, DRIVE_TURN_LEFT_ENC,

         /// Try to grab some glyph
         0.1, DRIVE_RESET_ENC_DONE,
         //0.7, DRIVE_FORWARD_ENC_GRAB,
         0.4, DRIVE_FORWARD_ENC_GRAB,
         1.0, DRIVE_CLOSE_CLAW,

         // Back to safe zone
         0.1, DRIVE_RESET_ENC_DONE,
         //0.8, DRIVE_BACKWARD_ENC,
         0.5, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };

      double [] redTrip1ORIntake = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // new trip with auto encoder resetting and drop-push, 2017/11/10 
         // LQT trip, jewel, key column, and fetching extra glyph, 2018/01/16
         // OR move glyph from CENTER to LEFT, 2018/02/20
         // Try to get more glyphs by new intake system, 2018/02/21
         0.1, DRIVE_STOP,                                   
         0.2, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         0.7, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         //0.70, DRIVE_FORWARD_ENC,       // not very reliable
         1.5, DRIVE_SHIFT_GEAR, 
         0.75, DRIVE_FORWARD_ENC_SLOW,     
         1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,
         0.8, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_FORWARD_ENC_TO_WALL, 
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN5,    // pick key column 

         //RIGHT COLUMN
         //0.15, DRIVE_BACKWARD_ENC,    // new short trip with better reliability
         0.1, DRIVE_BACKWARD_ENC_SLOW,    // new short trip with better reliability
         0.1, DRIVE_RESET_ENC_DONE,
         65,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.20, DRIVE_FORWARD_ENC_SLOW,

         1.0, DRIVE_SHIFT_CLAW_LEFT,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW, // drop glyph
         0.2, DRIVE_BACKWARD_ENC, 
         2.0, DRIVE_SHIFT_GEAR,

         // turn to glyph pit
         //0.1, DRIVE_RESET_ENC_DONE,
         //120, DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         50, DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.20, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,

         80, DRIVE_TURN_LEFT_ENC,
         //-1006, DRIVE_STATE_JUMP,           
         //-1001, DRIVE_STATE_JUMP,       // JUST STOP     
         -1016, DRIVE_STATE_JUMP,     // NOT ready, FIXME
         /// END of RIGHT, 17 steps

         // MIDDLE COLUMN, break CENTER, then move the glyph to LEFT 
         55,  DRIVE_TURN_RIGHT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,   
         0.30, DRIVE_FORWARD_ENC_SLOW,  // break CENTER

         // Extra step to drop and pick glyph, +4
         0.5, DRIVE_OPEN_CLAW_ONLY,  // break CENTER  ++1
         0.1, DRIVE_STOP,               // ensure that it's 9 steps for CENTER to match DRIVE_KEY_COLUMN3
         0.5, DRIVE_CLOSE_CLAW,  // break CENTER  ++1
         (LIFT_ENC_COUNT_PER_GLYPH/6), DRIVE_LIFT_CLAWS,  // lift the claws up by ~2in 

         0.1, DRIVE_RESET_ENC_DONE,   
         0.25, DRIVE_BACKWARD_ENC_SLOW, // back off

         0.1, DRIVE_RESET_ENC_DONE,   
         55,  DRIVE_TURN_LEFT_ENC,      // restore heading
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_STOP,               // ensure that it's 9 steps for CENTER to match DRIVE_KEY_COLUMN3
         // fall through to LEFT COLUMN trips 
         /// END of CENTER, 9 steps

         //LEFT COLUMN
         0.20, DRIVE_FORWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         53,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,    
         //0.25, DRIVE_FORWARD_ENC_SLOW,
         0.20, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to pit 
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         140, DRIVE_TURN_LEFT_ENC,

         /// Try to grab some glyph with help of intake system
         INTAKE_PULL, DRIVE_SWITCH_INTAKE,    // SHELF -> PULL
         2.0, DRIVE_STOP,              // extra delay for intake to be ready
         0.1, DRIVE_RESET_ENC_DONE,
         INTAKE_PUSH, DRIVE_SWITCH_INTAKE,    // PULL -> PUSH
         0.4, DRIVE_FORWARD_ENC_GRAB,

         INTAKE_PULL, DRIVE_SWITCH_INTAKE,    // PUSH -> PULL 
         1.5, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         0.2, DRIVE_FORWARD_ENC_GRAB,
         1.0, DRIVE_CLOSE_CLAW, 

         (LIFT_ENC_COUNT_PER_GLYPH/4), DRIVE_LIFT_CLAWS,  // lift the claws up by ~2in 
         INTAKE_SHELF, DRIVE_SWITCH_INTAKE, 
         0.1, DRIVE_RESET_ENC_DONE,
         0.85, DRIVE_BACKWARD_ENC,      // back to safe zone
         0, DRIVE_LIFT_CLAWS,           // lower it down to ground

         0.1, DRIVE_RESET_ENC_DONE,
         0.05, DRIVE_FORWARD_ENC,      // forward to avoid touch 

         60.0, DRIVE_STOP             
      };

      /// Red position#1 for Super Regional, 2018/02/27 
      double [] redTrip1SR = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // new trip with auto encoder resetting and drop-push, 2017/11/10 
         // LQT trip, jewel, key column, and fetching extra glyph, 2018/01/16
         // OR move glyph from CENTER to LEFT, 2018/02/20
         // Try to get more glyphs by new intake system, 2018/02/21
         0.1, DRIVE_STOP,                                   
         0.2, DRIVE_PICTOGRAPH,
         //1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_DROP_JEWEL_ARM, // 2018/03/09
         0.7, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         //0.70, DRIVE_FORWARD_ENC,       // not very reliable
         1.5, DRIVE_SHIFT_GEAR, 
         0.75, DRIVE_FORWARD_ENC_SLOW,     
         1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,
         //0.8, DRIVE_DROP_JEWEL_ARM,
         //1.0, DRIVE_FORWARD_ENC_TO_WALL,  // not reliable, disable, 2018/02/28  
         //0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN7,    // pick key column 

         //RIGHT COLUMN
         0.1, DRIVE_BACKWARD_ENC_SLOW,    // new short trip with better reliability
         0.1, DRIVE_RESET_ENC_DONE,
         65,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         //0.20, DRIVE_FORWARD_ENC_SLOW,
         //0.15, DRIVE_FORWARD_ENC_SLOW,
         0.10, DRIVE_FORWARD_ENC_SLOW,

         0.5, DRIVE_SHIFT_CLAW_LEFT,
         //0.5, DRIVE_OPEN_CLAW, // drop glyph
         //0.8, DRIVE_OPEN_CLAW, // drop glyph
         0.5, DRIVE_OPEN_CLAW_ONLY, // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,    

         0.2, DRIVE_BACKWARD_ENC, 
         2.0, DRIVE_SHIFT_GEAR,

         // turn to glyph pit
         0.1, DRIVE_RESET_ENC_DONE,
         50, DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.20, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,

         //80, DRIVE_TURN_LEFT_ENC,
         100, DRIVE_TURN_LEFT_ENC,
         //-1006, DRIVE_STATE_JUMP,           
         //-1001, DRIVE_STATE_JUMP,       // JUST STOP     
         //-1016, DRIVE_STATE_JUMP,     
         -1018, DRIVE_STATE_JUMP,      // 2018/03/02 
         /// END of RIGHT, 17 steps

         /// MIDDLE COLUMN, break CENTER, then move the glyph to LEFT 
         55,  DRIVE_TURN_RIGHT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,   
         0.20, DRIVE_FORWARD_ENC_SLOW,  

         // Extra step to drop glyph, back/forth, and then pick glyph, +10
         0.5, DRIVE_OPEN_CLAW_ONLY,     // drop glyph
         0.1, DRIVE_STOP,               
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,   // push the glyph
         0.1, DRIVE_RESET_ENC_DONE,   

         0.1, DRIVE_BACKWARD_ENC_SLOW,  // back off
         0.1, DRIVE_RESET_ENC_DONE,
         60, DRIVE_STOP,




              0.1, DRIVE_FORWARD_ENC_SLOW,   // forward
         0.5, DRIVE_CLOSE_CLAW,         // pick the glyph up
         (LIFT_ENC_COUNT_PER_GLYPH/6), DRIVE_LIFT_CLAWS,  // lift the claws up by ~1in 

         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_BACKWARD_ENC_SLOW, // back off
         0.1, DRIVE_RESET_ENC_DONE,

         //35,  DRIVE_TURN_RIGHT_ENC,      
         31,  DRIVE_TURN_RIGHT_ENC,     // 2018/03/07  
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,  
         0.5, DRIVE_SHIFT_CLAW_LEFT,

         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_BACKWARD_ENC_SLOW,  
         //60, DRIVE_STOP,                // END of CENTER, 9 steps

         //LEFT COLUMN
         0.20, DRIVE_FORWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         53,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,    
         //0.25, DRIVE_FORWARD_ENC_SLOW,
         //0.20, DRIVE_FORWARD_ENC_SLOW,
         //0.10, DRIVE_FORWARD_ENC_SLOW,
         0.15, DRIVE_FORWARD_ENC_SLOW,  // 2018/03/06 
         0.5, DRIVE_SHIFT_CLAW_RIGHT,
         // drop glyph
         //0.5, DRIVE_OPEN_CLAW,
         0.5, DRIVE_OPEN_CLAW_ONLY,
         0.1, DRIVE_RESET_ENC_DONE,    
         //0.10, DRIVE_FORWARD_ENC_SLOW,
         0.13, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to pit 
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         140, DRIVE_TURN_LEFT_ENC,

         /// Try to grab some glyph with help of intake system
         INTAKE_PULL, DRIVE_SWITCH_INTAKE,    // SHELF -> PULL
         //2.0, DRIVE_STOP,              // extra delay for intake to be ready
         1.5, DRIVE_STOP,              // extra delay for intake to be ready, 2018/03/06 
         0.1, DRIVE_RESET_ENC_DONE,
         INTAKE_PUSH, DRIVE_SWITCH_INTAKE,    // PULL -> PUSH
         0.4, DRIVE_FORWARD_ENC_GRAB,

         INTAKE_PULL, DRIVE_SWITCH_INTAKE,    // PUSH -> PULL 
         1.5, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         0.2, DRIVE_FORWARD_ENC_GRAB,
         //1.0, DRIVE_CLOSE_CLAW, 
         0.8, DRIVE_CLOSE_CLAW,         // make it faster to aovid timeout 2018/03/02  

         (LIFT_ENC_COUNT_PER_GLYPH/4), DRIVE_LIFT_CLAWS,  // lift the claws up by ~2in 
         INTAKE_SHELF, DRIVE_SWITCH_INTAKE, 
         0.1, DRIVE_RESET_ENC_DONE,
         0.85, DRIVE_BACKWARD_ENC,      // back to safe zone
         0, DRIVE_LIFT_CLAWS,           // lower it down to ground

         0.1, DRIVE_RESET_ENC_DONE,
         0.01, DRIVE_FORWARD_ENC,      // forward to avoid touch 

         60.0, DRIVE_STOP             
      };

      double [] redTrip2 = {
         // Red position2: Jewel and key column
         0.1, DRIVE_STOP,
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_STOP,

         0.7, DRIVE_FORWARD_ENC_SLOW,     
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN2,    // pick key column 

         //RIGHT COLUMN
         //20,  DRIVE_TURN_LEFT_ENC,
         13,  DRIVE_TURN_LEFT_ENC,  // MW90
         0.1, DRIVE_RESET_ENC_DONE,
         0.25, DRIVE_FORWARD_ENC_SLOW,
         -1011, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         //40,  DRIVE_TURN_LEFT_ENC,
         35,  DRIVE_TURN_LEFT_ENC,  // MW9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         -1011, DRIVE_STATE_JUMP,           

         //LEFT COLUMN
         60,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         15,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,

         // drop glyph
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,

         // push the glyph
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_CLOSE_CLAW,
         0.2, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.1, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_OPEN_CLAW,

         60.0, DRIVE_STOP             
      };

      double [] redTrip2QT = {
         // Red position2: Jewel and key column
         0.1, DRIVE_STOP,
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_STOP,

         0.7, DRIVE_FORWARD_ENC_SLOW,     
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN2,    // pick key column 

         //RIGHT COLUMN, not very reliable bcz the head-in
         15,  DRIVE_TURN_LEFT_ENC,  // MW90
         0.1, DRIVE_RESET_ENC_DONE,
         0.25, DRIVE_FORWARD_ENC_SLOW,
         -1004, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         35,  DRIVE_TURN_LEFT_ENC,  // MW9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         -1004, DRIVE_STATE_JUMP,           

         //LEFT COLUMN
         60,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         //15,  DRIVE_TURN_RIGHT_ENC,
         18,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,

         // drop glyph
         1.0, DRIVE_OPEN_CLAW,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };

      double [] redTrip2SQ = {
         // Red position2: for SQ
         0.1, DRIVE_STOP,
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_STOP,

         0.7, DRIVE_FORWARD_ENC_SLOW,     
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN4,    // pick key column 

         //RIGHT COLUMN, back first to create room for 45-deg in
         90,  DRIVE_TURN_LEFT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,
         0.20, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,
         50,  DRIVE_TURN_RIGHT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,
         0.25, DRIVE_FORWARD_ENC_SLOW,
         -1006, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         35,  DRIVE_TURN_LEFT_ENC,  // MW9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         -1006, DRIVE_STATE_JUMP,           

         //LEFT COLUMN
         60,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         18,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,

         // drop glyph
         1.0, DRIVE_SHIFT_CLAW_RIGHT,
         0.5, DRIVE_STOP,
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };

      double [] redTrip2OR = {
         // Red position2 for OR
         0.1, DRIVE_STOP,
         0.2, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         0.7, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_STOP,


         //1.5, DRIVE_SHIFT_GEAR, 
         0.7, DRIVE_FORWARD_ENC_SLOW,     
         //1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN6,    // pick key column for OR

         //RIGHT COLUMN, back first to create room for 45-deg in
         90,  DRIVE_TURN_LEFT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,
         0.20, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,
         50,  DRIVE_TURN_RIGHT_ENC,  
         //55,  DRIVE_TURN_RIGHT_ENC,  

         0.1, DRIVE_RESET_ENC_DONE, 
         0.3, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT,
         0.1, DRIVE_STOP,
         0.5, DRIVE_OPEN_CLAW,

         0.1, DRIVE_RESET_ENC_DONE,    
         //2.0, DRIVE_SHIFT_GEAR, 
         1.5, DRIVE_SHIFT_GEAR, 
         //0.3, DRIVE_BACKWARD_ENC,
         0.25, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         45,  DRIVE_TURN_LEFT_ENC,  

         0.3, DRIVE_RESET_ENC_DONE,    
         0.6, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         45,  DRIVE_TURN_LEFT_ENC,  
         1.0, DRIVE_SHIFT_GEAR, 

         60.0, DRIVE_STOP,             /// RIGHT trip END, 21 steps

         //MIDDLE COLUMN
         //35,  DRIVE_TURN_LEFT_ENC,  
         //0.1, DRIVE_RESET_ENC_DONE,   
         //0.3, DRIVE_FORWARD_ENC_SLOW,
         //-1006, DRIVE_STATE_JUMP,           

         32,  DRIVE_TURN_LEFT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,

         // Extra 4 step to drop and pick glyph, +4
         0.5, DRIVE_OPEN_CLAW_ONLY,  // break CENTER  ++1
         0.1, DRIVE_STOP,               // ensure that it's 9 steps for CENTER to match DRIVE_KEY_COLUMN3
         0.5, DRIVE_CLOSE_CLAW,  // break CENTER  ++1
         (LIFT_ENC_COUNT_PER_GLYPH/6), DRIVE_LIFT_CLAWS,  // lift the claws up by ~2in 

         0.1, DRIVE_RESET_ENC_DONE,   
         0.2, DRIVE_BACKWARD_ENC_SLOW,

         0.1, DRIVE_RESET_ENC_DONE,    
         20,  DRIVE_TURN_LEFT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,   
         0.2, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,    

         23,  DRIVE_TURN_RIGHT_ENC,  
         0.25, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_SHIFT_CLAW_LEFT,
         0.1, DRIVE_STOP,
         0.5, DRIVE_OPEN_CLAW,

         0.1, DRIVE_RESET_ENC_DONE,   
         0.2, DRIVE_BACKWARD_ENC,
         1.5, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,    
         //45,  DRIVE_TURN_LEFT_ENC,  
         55,  DRIVE_TURN_LEFT_ENC,  

         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_FORWARD_ENC,
         //60,  DRIVE_TURN_LEFT_ENC,  
         50,  DRIVE_TURN_LEFT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,    

         60.0, DRIVE_STOP,             /// CENTER trip END, 25 steps

         // LEFT COLUMN
         60,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         20,  DRIVE_TURN_RIGHT_ENC,

         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT, 
         0.1, DRIVE_STOP,
         0.5, DRIVE_OPEN_CLAW,

         1.5, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,    
         0.2, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         45,  DRIVE_TURN_LEFT_ENC,  

         0.1, DRIVE_RESET_ENC_DONE,    
         0.4, DRIVE_FORWARD_ENC,
         60,  DRIVE_TURN_LEFT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,    

         0.1, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.05, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    

         60.0, DRIVE_STOP             
      };

      /// New trip for Super-Regional, 2018/02/28 
      double [] redTrip2SR = {
         // Red position2 for OR
         0.1, DRIVE_STOP,
         0.2, DRIVE_PICTOGRAPH,
         //1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_DROP_JEWEL_ARM, // 2018/03/09
         0.7, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_STOP, 

         //1.5, DRIVE_SHIFT_GEAR, 
         0.7, DRIVE_FORWARD_ENC_SLOW,     
         //1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN8,    // pick key column for SR

         //RIGHT COLUMN, back first to create room for 45-deg in
         90,  DRIVE_TURN_LEFT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,
         0.20, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,
         50,  DRIVE_TURN_RIGHT_ENC,  
         //55,  DRIVE_TURN_RIGHT_ENC,

         0.1, DRIVE_RESET_ENC_DONE, 
         0.2, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_LEFT,
         0.1, DRIVE_STOP,
         //0.5, DRIVE_OPEN_CLAW,
         0.5, DRIVE_OPEN_CLAW_ONLY,
         0.1, DRIVE_RESET_ENC_DONE, 
         0.15, DRIVE_FORWARD_ENC_SLOW,

         0.1, DRIVE_RESET_ENC_DONE,    
         //2.0, DRIVE_SHIFT_GEAR, 
         1.5, DRIVE_SHIFT_GEAR, 
         //0.3, DRIVE_BACKWARD_ENC,
         0.25, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         //60, DRIVE_STOP,

         45,  DRIVE_TURN_LEFT_ENC,  

         0.3, DRIVE_RESET_ENC_DONE,    
         0.6, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         45,  DRIVE_TURN_LEFT_ENC,  
         1.0, DRIVE_SHIFT_GEAR, 

         60.0, DRIVE_STOP,             /// RIGHT trip END, 21 steps; +2, 2018/03/02

         //MIDDLE COLUMN
         35,  DRIVE_TURN_LEFT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,    
         0.3, DRIVE_FORWARD_ENC_SLOW,

         // Extra step to drop glyph, back/forth, and then pick glyph, +8
         0.5, DRIVE_OPEN_CLAW_ONLY,     // drop glyph
         0.1, DRIVE_STOP,               
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_BACKWARD_ENC_SLOW,  // back off
         0.1, DRIVE_RESET_ENC_DONE,
         60, DRIVE_STOP,



         0.1, DRIVE_FORWARD_ENC_SLOW,   // forward
         0.5, DRIVE_CLOSE_CLAW,         // pick the glyph up
         (LIFT_ENC_COUNT_PER_GLYPH/6), DRIVE_LIFT_CLAWS,  // lift the claws up by ~1in 

         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_BACKWARD_ENC_SLOW, 

         0.1, DRIVE_RESET_ENC_DONE,    
         //40,  DRIVE_TURN_RIGHT_ENC,  
         35,  DRIVE_TURN_RIGHT_ENC,   // 2018/03/07 

         0.1, DRIVE_RESET_ENC_DONE,    
         0.01, DRIVE_FORWARD_ENC_SLOW, 
         0.1, DRIVE_SHIFT_CLAW_LEFT,
         0.1, DRIVE_STOP,
         //0.5, DRIVE_OPEN_CLAW,
         1.0, DRIVE_OPEN_CLAW,

         0.1, DRIVE_RESET_ENC_DONE,   
         0.15, DRIVE_BACKWARD_ENC,
         1.5, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,    
         120,  DRIVE_TURN_LEFT_ENC,  

         //60.0, DRIVE_STOP,             /// CENTER trip END, 26 steps

         // LEFT COLUMN
         60,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         20,  DRIVE_TURN_RIGHT_ENC,

         0.1, DRIVE_RESET_ENC_DONE,   
         0.15, DRIVE_FORWARD_ENC_SLOW,

         // Extra step to drop glyph, back/forth, and then pick glyph, +8
         0.5, DRIVE_OPEN_CLAW_ONLY,     // drop glyph
         0.1, DRIVE_STOP,               
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_BACKWARD_ENC_SLOW,  // back off
         0.1, DRIVE_RESET_ENC_DONE,
         60, DRIVE_STOP,




              0.1, DRIVE_FORWARD_ENC_SLOW,   // forward
         0.5, DRIVE_CLOSE_CLAW,         // pick the glyph up
         (LIFT_ENC_COUNT_PER_GLYPH/6), DRIVE_LIFT_CLAWS,  // lift the claws up by ~1in 

         0.1, DRIVE_RESET_ENC_DONE,   
         0.2, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,
         55,  DRIVE_TURN_RIGHT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,    
         0.10, DRIVE_FORWARD_ENC_SLOW, 

         0.5, DRIVE_SHIFT_CLAW_RIGHT, 
         0.1, DRIVE_STOP,
         1.0, DRIVE_OPEN_CLAW,
         //0.5, DRIVE_OPEN_CLAW_ONLY,
         //0.1, DRIVE_RESET_ENC_DONE,    
         //0.1, DRIVE_FORWARD_ENC_SLOW, 
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,

         //60.0, DRIVE_STOP              /// LEFT trip END
      };

      if( test_trip_id_>0 ) {
         if( test_trip_id_==2 ) {
            return getDriveMode(testTrip2, t);        // test encoder drive/turn
         } else if (test_trip_id_ == 3){
            return getDriveMode(testTrip12, t);
         } else if (test_trip_id_ == 8){
            return getDriveMode(testTrip8, t);        // test CHECK_DISTANCE using range sensor
         } else if (test_trip_id_ == 9){
            return getDriveMode(testTrip9, t);        // test wall following using range sensor
         } else if (test_trip_id_ == 11){
            return getDriveMode(testTrip11, t);
         } else if (test_trip_id_ == 12){
            return getDriveMode(testTrip12, t);
         } else {
            return getDriveMode (testTrip2, t);
         }
      } 

      if( trip_name_ == "RedPos1Diag") {
         return getDriveMode(redTrip1, t);
      } else if( trip_name_ == "RedPos1SR") {
         return getDriveMode(redTrip1SR, t);
      } else if( trip_name_ == "RedPos1ORIntake") {
         return getDriveMode(redTrip1ORIntake, t);
      } else if( trip_name_ == "RedPos1OR") {
         return getDriveMode(redTrip1OR, t);
      } else if( trip_name_ == "RedPos2SR") {
         return getDriveMode(redTrip2SR, t);
      } else if( trip_name_ == "RedPos2OR") {
         return getDriveMode(redTrip2OR, t);
      } else if( trip_name_ == "RedPos1ORTest") {
         return getDriveMode(redTrip1ORTest, t);
      } else if( trip_name_ == "RedPos1QT") {
         return getDriveMode(redTrip1QT, t);
      } else if( trip_name_ == "RedPos2Diag") {
         //return getDriveMode(redTrip2, t);
         return getDriveMode(redTrip2QT, t);
      } else if( trip_name_ == "RedPos2SQ") {
         return getDriveMode(redTrip2SQ, t);
      } else if( trip_name_ == "JewelOnly") {
         return getDriveMode(redTrip0, t);
      } else {
         return getDriveMode(redTrip1, t);
      }
   }

   /// Define blue trip
   int getDriveModeBlue(double t) {

      double [] blueTrip1 = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // new trip with auto encoder resetting and drop-push, 2017/11/10 
         0.1, DRIVE_STOP,                                   
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         //1.0, DRIVE_FORWARD_ENC_TO_WALL,     
         0.75, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN,    // pick key column 

         //LEFT COLUMN
         //85,  DRIVE_TURN_RIGHT_ENC,
         0.30, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         55,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.25, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,
         -1011, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         110,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.20, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_LEFT,
         -1011, DRIVE_STATE_JUMP,           

         //RIGHT COLUMN
         0.15, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         115,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.3, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_LEFT,
         
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_BACKWARD_ENC,

         // push the glyph
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_CLOSE_CLAW,
         0.2, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.1, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_OPEN_CLAW,

         60.0, DRIVE_STOP             
      };

/*
      /// Blue Pos1 for LQT, turn to pit
      double [] blueTrip1QT = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // new trip with auto encoder resetting and drop-push, 2017/11/10 
         0.1, DRIVE_STOP,                                   
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         0.75, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN3,    // pick key column 

         //LEFT COLUMN
         0.30, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         55,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.20, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT, 
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to glyph pit
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         135, DRIVE_TURN_LEFT_ENC,
         -1006, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         110,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.20, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT,

         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_BACKWARD_ENC, 
         30, DRIVE_STOP,   // stop here
         -1006, DRIVE_STATE_JUMP,           

         //RIGHT COLUMN
         //0.15, DRIVE_BACKWARD_ENC_SLOW,  
         0.10, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         115,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.25, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT, 
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to pit 
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         145, DRIVE_TURN_RIGHT_ENC,

         /// Try to grab some glyph
         0.1, DRIVE_RESET_ENC_DONE,
         //0.7, DRIVE_FORWARD_ENC,
         //0.7, DRIVE_FORWARD_ENC_GRAB,
         0.4, DRIVE_FORWARD_ENC_GRAB,
         1.0, DRIVE_CLOSE_CLAW,

         // Back to safe zone
         0.1, DRIVE_RESET_ENC_DONE,
         //0.8, DRIVE_BACKWARD_ENC,
         0.5, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };
*/
      /// Blue Pos1 for LQT, turn to pit, after relic module, 2018/02/10
      double [] blueTrip1QT = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // new trip with auto encoder resetting and drop-push, 2017/11/10 
         0.1, DRIVE_STOP,                                   
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         0.75, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN3,    // pick key column 

         //LEFT COLUMN
         0.30, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         52,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.20, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT, 
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to glyph pit
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         135, DRIVE_TURN_LEFT_ENC,
         -1006, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         110,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.20, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT,

         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_BACKWARD_ENC, 
         30, DRIVE_STOP,   // stop here
         -1006, DRIVE_STATE_JUMP,           

         //RIGHT COLUMN
         //0.15, DRIVE_BACKWARD_ENC_SLOW,  
         0.10, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         115,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.25, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT, 
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to pit 
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         145, DRIVE_TURN_RIGHT_ENC,

         /// Try to grab some glyph
         0.1, DRIVE_RESET_ENC_DONE,
         //0.7, DRIVE_FORWARD_ENC,
         //0.7, DRIVE_FORWARD_ENC_GRAB,
         0.4, DRIVE_FORWARD_ENC_GRAB,
         1.0, DRIVE_CLOSE_CLAW,

         // Back to safe zone
         0.1, DRIVE_RESET_ENC_DONE,
         //0.8, DRIVE_BACKWARD_ENC,
         0.5, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };

      /// Blue Pos1 for OR, move the glyph from CENTER to RIGHT
      double [] blueTrip1OR = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // New trip with auto encoder resetting and drop-push, 2017/11/10 
         // Move glyph from CENTER to RIGHT
         0.1, DRIVE_STOP,                                   
         0.2, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         0.7, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         1.5, DRIVE_SHIFT_GEAR, 
         0.75, DRIVE_BACKWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN5,    // pick key column 

         //LEFT COLUMN
         0.30, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         52,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         //0.20, DRIVE_FORWARD_ENC_SLOW,
         0.25, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT, 
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to glyph pit
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         135, DRIVE_TURN_LEFT_ENC,
         -1006, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         // Original trip
         //110, DRIVE_TURN_RIGHT_ENC,  // 
         //0.1, DRIVE_RESET_ENC_DONE,   
         //0.20, DRIVE_FORWARD_ENC_SLOW,
         //1.0, DRIVE_SHIFT_CLAW_LEFT, 
         //0.1, DRIVE_RESET_ENC_DONE,    
         //0.5, DRIVE_OPEN_CLAW,
         //0.1, DRIVE_BACKWARD_ENC, 
         //30, DRIVE_STOP,   // stop here

         // new trip to move glyph from CENTER to LEFT
         110, DRIVE_TURN_RIGHT_ENC,  // 
         0.1, DRIVE_RESET_ENC_DONE,   
         0.25, DRIVE_FORWARD_ENC_SLOW,   // go deeper
         0.1, DRIVE_RESET_ENC_DONE,    
         //0.20, DRIVE_BACKWARD_ENC_SLOW,
         //27,  DRIVE_TURN_LEFT_ENC,  // 
         //0.15, DRIVE_FORWARD_ENC_SLOW,   // go deeper
         0.40, DRIVE_BACKWARD_ENC_SLOW,  // go in with 10deg angle for better reliability
         15,  DRIVE_TURN_LEFT_ENC,       // 
         0.35, DRIVE_FORWARD_ENC_SLOW,   // go deeper
         0.1, DRIVE_RESET_ENC_DONE,    

         0.1, DRIVE_SHIFT_CLAW_LEFT, 
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         0.1, DRIVE_RESET_ENC_DONE,    

         3.0, DRIVE_SHIFT_GEAR, 
         160, DRIVE_TURN_RIGHT_ENC,     //  head to pit
         1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,    

         0.2, DRIVE_BACKWARD_ENC,      // push the glyph a little
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_FORWARD_ENC,       // go forward a little to avoid touch
         0.1, DRIVE_RESET_ENC_DONE,    

         30,  DRIVE_STOP,              // stop here

         //RIGHT COLUMN
         0.15, DRIVE_BACKWARD_ENC_SLOW,  
         //0.10, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         115, DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.25, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT, 
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to pit 
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         145, DRIVE_TURN_RIGHT_ENC,

         /// Try to grab some glyph
         0.1, DRIVE_RESET_ENC_DONE,
         //0.7, DRIVE_FORWARD_ENC,
         //0.7, DRIVE_FORWARD_ENC_GRAB,
         0.4, DRIVE_FORWARD_ENC_GRAB,
         1.0, DRIVE_CLOSE_CLAW,

         // Back to safe zone
         0.1, DRIVE_RESET_ENC_DONE,
         //0.8, DRIVE_BACKWARD_ENC,
         0.5, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };

      /// Blue Pos1 for OR, move the glyph from CENTER to RIGHT, and grabbing
      double [] blueTrip1ORIntake = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // New trip with auto encoder resetting and drop-push, 2017/11/10 
         // Move glyph from CENTER to RIGHT
         // Use intake to help grabbing 
         0.1, DRIVE_STOP,                                   
         0.2, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         0.7, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         1.5, DRIVE_SHIFT_GEAR, 
         0.75, DRIVE_BACKWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN5,    // pick key column 

         //LEFT COLUMN
         0.30, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         52,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         //0.20, DRIVE_FORWARD_ENC_SLOW,
         0.25, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT, 
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to glyph pit
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         135, DRIVE_TURN_LEFT_ENC,
         //-1006, DRIVE_STATE_JUMP,           
         -1016, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         // new trip to move glyph from CENTER to LEFT
         110, DRIVE_TURN_RIGHT_ENC,  // 
         0.1, DRIVE_RESET_ENC_DONE,   
         0.25, DRIVE_FORWARD_ENC_SLOW,   // go deeper

         // Extra 4 step to drop and pick glyph, +4
         0.5, DRIVE_OPEN_CLAW_ONLY,  // break CENTER  ++1
         0.1, DRIVE_STOP,               // ensure that it's 9 steps for CENTER to match DRIVE_KEY_COLUMN3
         0.5, DRIVE_CLOSE_CLAW,  // break CENTER  ++1
         (LIFT_ENC_COUNT_PER_GLYPH/6), DRIVE_LIFT_CLAWS,  // lift the claws up by ~2in 

         0.1, DRIVE_RESET_ENC_DONE,    
         0.40, DRIVE_BACKWARD_ENC_SLOW,  // go in with 10deg angle for better reliability
         15,  DRIVE_TURN_LEFT_ENC,       // 
         0.35, DRIVE_FORWARD_ENC_SLOW,   // go deeper
         0.1, DRIVE_RESET_ENC_DONE,    

         0.1, DRIVE_SHIFT_CLAW_LEFT, 
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         0.1, DRIVE_RESET_ENC_DONE,    

         3.0, DRIVE_SHIFT_GEAR, 
         160, DRIVE_TURN_RIGHT_ENC,     //  head to pit
         1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,    

         0.2, DRIVE_BACKWARD_ENC,      // push the glyph a little
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_FORWARD_ENC,       // go forward a little to avoid touch
         0.1, DRIVE_RESET_ENC_DONE,    

         30,  DRIVE_STOP,              // stop here

         //RIGHT COLUMN
         0.15, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         115, DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.25, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT, 
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         2.0, DRIVE_SHIFT_GEAR, 
         // turn to pit 
         0.1, DRIVE_RESET_ENC_DONE,
         145, DRIVE_TURN_RIGHT_ENC,

         /// Try to grab some glyph with help of intake system
         INTAKE_PULL, DRIVE_SWITCH_INTAKE, 
         1.5, DRIVE_STOP,              // extra delay for intake to be ready
         0.1, DRIVE_RESET_ENC_DONE,
         INTAKE_PUSH, DRIVE_SWITCH_INTAKE, 
         0.4, DRIVE_FORWARD_ENC_GRAB,

         INTAKE_PULL, DRIVE_SWITCH_INTAKE, 
         1.5, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         0.2, DRIVE_FORWARD_ENC_GRAB,
         1.0, DRIVE_CLOSE_CLAW, 

         (LIFT_ENC_COUNT_PER_GLYPH/4), DRIVE_LIFT_CLAWS,  // lift the claws up by ~2in 
         INTAKE_SHELF, DRIVE_SWITCH_INTAKE, 
         0.1, DRIVE_RESET_ENC_DONE,
         0.85, DRIVE_BACKWARD_ENC,      // Back to safe zone
         0, DRIVE_LIFT_CLAWS,           // lower it down to ground

         60.0, DRIVE_STOP             
      };

      /// Blue Pos1 for SR, move the glyph from CENTER to RIGHT, and grabbing
      double [] blueTrip1SR = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // New trip with auto encoder resetting and drop-push, 2017/11/10 
         // Move glyph from CENTER to RIGHT
         // Use intake to help grabbing 
         0.1, DRIVE_STOP,                                   
         0.2, DRIVE_PICTOGRAPH,
         //1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_DROP_JEWEL_ARM, // 2018/03/09
         0.7, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         1.5, DRIVE_SHIFT_GEAR, 
         0.75, DRIVE_BACKWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN7,    // pick key column 

         //LEFT COLUMN
         0.30, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         52,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         //0.25, DRIVE_FORWARD_ENC_SLOW,
         0.15, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT, 
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         //0.5, DRIVE_OPEN_CLAW,
         0.5, DRIVE_OPEN_CLAW_ONLY,
         0.15, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,
         0.2, DRIVE_BACKWARD_ENC, 
         // turn to glyph pit
         2.0, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         135, DRIVE_TURN_LEFT_ENC,
         //-1006, DRIVE_STATE_JUMP,           
         //-1016, DRIVE_STATE_JUMP,           
         -1018, DRIVE_STATE_JUMP,             // 2018/03/06, 2 steps to avoid contact

         //MIDDLE COLUMN
         // new trip to move glyph from CENTER to LEFT
         110, DRIVE_TURN_RIGHT_ENC,  // 
         0.1, DRIVE_RESET_ENC_DONE,   
         0.15, DRIVE_FORWARD_ENC_SLOW,   // go deeper

         // Extra step to drop glyph, back/forth, and then pick glyph, +8+2
         0.5, DRIVE_OPEN_CLAW_ONLY,     // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,   // forward
         0.1, DRIVE_STOP,               
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_BACKWARD_ENC_SLOW,  // back off
         0.1, DRIVE_RESET_ENC_DONE,
         60, DRIVE_STOP,



         0.1, DRIVE_FORWARD_ENC_SLOW,   // forward
         0.5, DRIVE_CLOSE_CLAW,         // pick the glyph up
         (LIFT_ENC_COUNT_PER_GLYPH/6), DRIVE_LIFT_CLAWS,  // lift the claws up by ~1in 

         0.1, DRIVE_RESET_ENC_DONE,    
         0.40, DRIVE_BACKWARD_ENC_SLOW,  // go in with 10deg angle for better reliability
         15,  DRIVE_TURN_LEFT_ENC,       // 
         0.35, DRIVE_FORWARD_ENC_SLOW,   // go deeper
         0.1, DRIVE_RESET_ENC_DONE,    

         0.1, DRIVE_SHIFT_CLAW_LEFT, 
         0.5, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC, 
         0.1, DRIVE_RESET_ENC_DONE,    
         3.0, DRIVE_SHIFT_GEAR, 

         160, DRIVE_TURN_RIGHT_ENC,     //  head to pit
         1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,    

         //30,  DRIVE_STOP,              // CENTER stop here, 27 steps

         //RIGHT COLUMN
         0.15, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         115, DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.15, DRIVE_FORWARD_ENC_SLOW,
         1.0, DRIVE_SHIFT_CLAW_LEFT, 
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         //0.5, DRIVE_OPEN_CLAW,
         0.5, DRIVE_OPEN_CLAW_ONLY,
         //0.15, DRIVE_FORWARD_ENC_SLOW,
         0.10, DRIVE_FORWARD_ENC_SLOW,  // 2018/03/07 
         0.1, DRIVE_RESET_ENC_DONE,    
         0.2, DRIVE_BACKWARD_ENC, 
         2.0, DRIVE_SHIFT_GEAR, 
         // turn to pit 
         0.1, DRIVE_RESET_ENC_DONE,
         145, DRIVE_TURN_RIGHT_ENC,

         /// Try to grab some glyph with help of intake system
         INTAKE_PULL, DRIVE_SWITCH_INTAKE, 
         //1.5, DRIVE_STOP,              // extra delay for intake to be ready
         1.3, DRIVE_STOP,              // extra delay for intake to be ready, 2018/03/06 
         0.1, DRIVE_RESET_ENC_DONE,
         INTAKE_PUSH, DRIVE_SWITCH_INTAKE, 
         0.4, DRIVE_FORWARD_ENC_GRAB,

         INTAKE_PULL, DRIVE_SWITCH_INTAKE, 
         1.5, DRIVE_SHIFT_GEAR,
         0.1, DRIVE_RESET_ENC_DONE,
         0.2, DRIVE_FORWARD_ENC_GRAB,
         //1.0, DRIVE_CLOSE_CLAW, 
         0.8, DRIVE_CLOSE_CLAW,         // 2018/03/06 

         (LIFT_ENC_COUNT_PER_GLYPH/4), DRIVE_LIFT_CLAWS,  // lift the claws up by ~2in 
         INTAKE_SHELF, DRIVE_SWITCH_INTAKE, 
         0.1, DRIVE_RESET_ENC_DONE,
         0.85, DRIVE_BACKWARD_ENC,      // Back to safe zone
         0, DRIVE_LIFT_CLAWS,           // lower it down to ground

         0.1, DRIVE_RESET_ENC_DONE,
         0.01, DRIVE_FORWARD_ENC,      // forward to avoid touch 

         60.0, DRIVE_STOP             
      };

      /// Blue Pos1 for SQ, try to grab more glyphs in AR
      double [] blueTrip1SQ = {
         // Jewel and first glyph WITH angled INSERT for key columen
         // new trip with auto encoder resetting and drop-push, 2017/11/10 
         0.1, DRIVE_STOP,                                   
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_RESET_ENC_DONE,
         //1.0, DRIVE_FORWARD_ENC_TO_WALL,     
         0.75, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN,    // pick key column 

         //LEFT COLUMN
         //85,  DRIVE_TURN_RIGHT_ENC,
         0.30, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         55,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.25, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_RIGHT,
         -1013, DRIVE_STATE_JUMP,           

         //MIDDLE COLUMN
         110,  DRIVE_TURN_RIGHT_ENC,  // WM9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.20, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_LEFT,
         -1013, DRIVE_STATE_JUMP,           

         //RIGHT COLUMN
         0.15, DRIVE_BACKWARD_ENC_SLOW,  
         0.1, DRIVE_RESET_ENC_DONE,
         115,  DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.3, DRIVE_FORWARD_ENC_SLOW,
         0.5, DRIVE_SHIFT_CLAW_LEFT,
         
         // drop glyph
         0.1, DRIVE_RESET_ENC_DONE,    
         1.0, DRIVE_OPEN_CLAW,
         //0.1, DRIVE_BACKWARD_ENC,
         0.2, DRIVE_BACKWARD_ENC,

         // turn to glyph pit
         0.1, DRIVE_RESET_ENC_DONE,
         //90,  DRIVE_TURN_TO_RIGHT,  
         145, DRIVE_TURN_RIGHT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         1.2, DRIVE_FORWARD_ENC,
         1.0, DRIVE_CLOSE_CLAW,

         // turn to box
         0.1, DRIVE_RESET_ENC_DONE,
         //-90, DRIVE_TURN_TO_RIGHT,  
         180, DRIVE_TURN_RIGHT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,
         1.5, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         1.0, DRIVE_OPEN_CLAW,
         0.2, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };
      double [] blueTrip2 = {
         // Blue position2: Jewel and key column
         0.1, DRIVE_STOP,
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_STOP,

         0.8, DRIVE_BACKWARD_ENC_SLOW,     
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN2,    // pick key column 

         //LEFT COLUMN
         //160, DRIVE_TURN_LEFT_ENC,
         148, DRIVE_TURN_LEFT_ENC,  // MW9
         0.1, DRIVE_RESET_ENC_DONE,
         0.15, DRIVE_FORWARD_ENC_SLOW,
         -1011, DRIVE_STATE_JUMP,           

         //CENTER COLUMN
         //140, DRIVE_TURN_LEFT_ENC,
         120, DRIVE_TURN_LEFT_ENC, // MW9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         -1011, DRIVE_STATE_JUMP,           

         //RIGHT  COLUMN
         //120, DRIVE_TURN_LEFT_ENC,
         90, DRIVE_TURN_LEFT_ENC, // MW9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         //15,  DRIVE_TURN_LEFT_ENC,
         45,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,

         // drop glyph
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,

         // push the glyph
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_CLOSE_CLAW,
         0.2, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         0.1, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,
         1.0, DRIVE_OPEN_CLAW,

         60.0, DRIVE_STOP             
      };

      double [] blueTrip2QT = {
         // Blue position2: Jewel and key column
         0.1, DRIVE_STOP,
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_STOP,

         0.8, DRIVE_BACKWARD_ENC_SLOW,     
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN2,    // pick key column 

         //LEFT COLUMN
         //160, DRIVE_TURN_LEFT_ENC,
         148, DRIVE_TURN_LEFT_ENC,  // MW9
         0.1, DRIVE_RESET_ENC_DONE,
         0.15, DRIVE_FORWARD_ENC_SLOW,
         -1004, DRIVE_STATE_JUMP,           

         //CENTER COLUMN
         //140, DRIVE_TURN_LEFT_ENC,
         120, DRIVE_TURN_LEFT_ENC, // MW9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         -1004, DRIVE_STATE_JUMP,           

         //RIGHT  COLUMN
         90, DRIVE_TURN_LEFT_ENC, // MW9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.3, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         45,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,

         // drop glyph
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };

      double [] blueTrip2SQ = {
         // Blue position2: Jewel and key column
         0.1, DRIVE_STOP,
         1.0, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_STOP,

         0.8, DRIVE_BACKWARD_ENC_SLOW,     
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN4,    // pick key column  for SQ

         //LEFT COLUMN
         90, DRIVE_TURN_LEFT_ENC,  // MW9
         0.1, DRIVE_RESET_ENC_DONE,
         0.10, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,
         40, DRIVE_TURN_LEFT_ENC,  // MW9
         0.1, DRIVE_RESET_ENC_DONE,
         0.15, DRIVE_FORWARD_ENC_SLOW,
         -1006, DRIVE_STATE_JUMP,           

         //CENTER COLUMN
         120, DRIVE_TURN_LEFT_ENC, // MW9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.15, DRIVE_FORWARD_ENC_SLOW,
         -1006, DRIVE_STATE_JUMP,           

         //RIGHT  COLUMN
         90, DRIVE_TURN_LEFT_ENC, // MW9
         0.1, DRIVE_RESET_ENC_DONE,   
         0.30, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         45,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,

         // drop glyph
         1.0, DRIVE_SHIFT_CLAW_LEFT,
         0.5, DRIVE_STOP,
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };

      double [] blueTrip2OR = {
         // Blue position2: Jewel and key column
         // Move glyph to LEFT and face pit, 2018/02/22 
         0.1, DRIVE_STOP,
         0.2, DRIVE_PICTOGRAPH,
         1.0, DRIVE_DROP_JEWEL_ARM,
         0.7, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_STOP,

         1.5, DRIVE_SHIFT_GEAR, 
         0.8, DRIVE_BACKWARD_ENC_SLOW,     
         1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN6,    // pick key column for OR

         //LEFT COLUMN
         90, DRIVE_TURN_LEFT_ENC, 
         0.1, DRIVE_RESET_ENC_DONE,
         0.10, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,
         40, DRIVE_TURN_LEFT_ENC,  

         0.1, DRIVE_RESET_ENC_DONE,
         0.15, DRIVE_FORWARD_ENC_SLOW, 
         1.0, DRIVE_SHIFT_CLAW_LEFT,
         0.1, DRIVE_STOP,
         0.5, DRIVE_OPEN_CLAW,

         0.1, DRIVE_RESET_ENC_DONE,    
         1.5, DRIVE_SHIFT_GEAR, 
         0.2, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         45,  DRIVE_TURN_RIGHT_ENC,  

         0.3, DRIVE_RESET_ENC_DONE,    
         //0.6, DRIVE_FORWARD_ENC,
         0.55, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         45,  DRIVE_TURN_RIGHT_ENC,  
         1.0, DRIVE_SHIFT_GEAR, 

         60.0, DRIVE_STOP,             /// LEFT trip END, 21 steps

         // CENTER COLUMN 
         90, DRIVE_TURN_LEFT_ENC, 
         0.1, DRIVE_RESET_ENC_DONE,
         0.10, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,
         //45, DRIVE_TURN_LEFT_ENC,  
         42, DRIVE_TURN_LEFT_ENC,  

         0.1, DRIVE_RESET_ENC_DONE,
         0.15, DRIVE_FORWARD_ENC_SLOW, 

         // Extra 4 step to drop and pick glyph, +4
         0.5, DRIVE_OPEN_CLAW_ONLY,  // break CENTER  ++1
         0.1, DRIVE_STOP,               // ensure that it's 9 steps for CENTER to match DRIVE_KEY_COLUMN3
         0.5, DRIVE_CLOSE_CLAW,  // break CENTER  ++1
         (LIFT_ENC_COUNT_PER_GLYPH/6), DRIVE_LIFT_CLAWS,  // lift the claws up by ~2in 

         0.1, DRIVE_RESET_ENC_DONE,
         0.30, DRIVE_BACKWARD_ENC_SLOW, 
         0.1, DRIVE_RESET_ENC_DONE,

         22, DRIVE_TURN_LEFT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,
         0.20, DRIVE_FORWARD_ENC_SLOW, 
         0.1, DRIVE_SHIFT_CLAW_LEFT,
         0.5, DRIVE_OPEN_CLAW,

         2.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         //45,  DRIVE_TURN_RIGHT_ENC,  
         70,  DRIVE_TURN_RIGHT_ENC,  

         0.1, DRIVE_RESET_ENC_DONE,    
         0.3, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         //45,  DRIVE_TURN_RIGHT_ENC,  
         30,  DRIVE_TURN_RIGHT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,    

         1.0, DRIVE_SHIFT_GEAR, 
         60.0, DRIVE_STOP,             /// CENTER trip END, 27 steps

         //RIGHT  COLUMN
         90, DRIVE_TURN_LEFT_ENC, 
         0.1, DRIVE_RESET_ENC_DONE,   
         0.30, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         //40,  DRIVE_TURN_LEFT_ENC,
         45,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,

         // Extra 4 step to drop and pick glyph, +4
         0.5, DRIVE_OPEN_CLAW_ONLY,  // break CENTER  ++1
         0.1, DRIVE_STOP,               // ensure that it's 9 steps for CENTER to match DRIVE_KEY_COLUMN3
         0.5, DRIVE_CLOSE_CLAW,  // break CENTER  ++1
         (LIFT_ENC_COUNT_PER_GLYPH/6), DRIVE_LIFT_CLAWS,  // lift the claws up by ~2in 

         0.1, DRIVE_RESET_ENC_DONE,    
         0.15, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         //50,  DRIVE_TURN_LEFT_ENC,
         55,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.10, DRIVE_FORWARD_ENC_SLOW,

         0.1, DRIVE_SHIFT_CLAW_LEFT,
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP             
      };

      /// New trip for super-regional 
      double [] blueTrip2SR = {
         // Blue position2: Jewel and key column
         // Move glyph to LEFT and face pit, 2018/02/22 
         0.1, DRIVE_STOP,
         0.2, DRIVE_PICTOGRAPH,
         //1.0, DRIVE_DROP_JEWEL_ARM,
         1.0, DRIVE_DROP_JEWEL_ARM, // 2018/03/09
         0.7, DRIVE_KNOCK_JEWEL,
         0.1, DRIVE_STOP,

         1.5, DRIVE_SHIFT_GEAR, 
         0.8, DRIVE_BACKWARD_ENC_SLOW,     
         1.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,

         0.1, DRIVE_KEY_COLUMN8,    // pick key column for OR

         //LEFT COLUMN
         90, DRIVE_TURN_LEFT_ENC, 
         0.1, DRIVE_RESET_ENC_DONE,
         0.10, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,
         40, DRIVE_TURN_LEFT_ENC,  

         0.1, DRIVE_RESET_ENC_DONE,
         0.10, DRIVE_FORWARD_ENC_SLOW, 
         1.0, DRIVE_SHIFT_CLAW_LEFT,
         0.1, DRIVE_STOP,
         //0.5, DRIVE_OPEN_CLAW,
         0.5, DRIVE_OPEN_CLAW_ONLY,
         0.1, DRIVE_RESET_ENC_DONE,
         0.10, DRIVE_FORWARD_ENC_SLOW, 

         0.1, DRIVE_RESET_ENC_DONE,    
         1.5, DRIVE_SHIFT_GEAR, 
         0.2, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         45,  DRIVE_TURN_RIGHT_ENC,  

         0.3, DRIVE_RESET_ENC_DONE,    
         //0.6, DRIVE_FORWARD_ENC,
         0.55, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         45,  DRIVE_TURN_RIGHT_ENC,  
         1.0, DRIVE_SHIFT_GEAR, 

         60.0, DRIVE_STOP,             /// LEFT trip END, 21+2 steps

         // CENTER COLUMN 
         90, DRIVE_TURN_LEFT_ENC, 
         0.1, DRIVE_RESET_ENC_DONE,
         0.10, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,
         //45, DRIVE_TURN_LEFT_ENC,  
         42, DRIVE_TURN_LEFT_ENC,  

         0.1, DRIVE_RESET_ENC_DONE,
         //0.10, DRIVE_FORWARD_ENC_SLOW, 
         0.15, DRIVE_FORWARD_ENC_SLOW,  // 2018/03/07 

         // Extra step to drop glyph, back/forth, and then pick glyph, +8
         0.5, DRIVE_OPEN_CLAW_ONLY,     // drop glyph
         0.1, DRIVE_STOP,               
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_BACKWARD_ENC_SLOW,  // back off
         0.1, DRIVE_RESET_ENC_DONE,
         60, DRIVE_STOP,

         0.1, DRIVE_FORWARD_ENC_SLOW,   // forward
         0.5, DRIVE_CLOSE_CLAW,         // pick the glyph up
         (LIFT_ENC_COUNT_PER_GLYPH/6), DRIVE_LIFT_CLAWS,  // lift the claws up by ~1in 

         0.1, DRIVE_RESET_ENC_DONE,
         //0.30, DRIVE_BACKWARD_ENC_SLOW, 
         0.10, DRIVE_BACKWARD_ENC_SLOW, 
         0.1, DRIVE_RESET_ENC_DONE,

         //22, DRIVE_TURN_LEFT_ENC,  
         //45, DRIVE_TURN_LEFT_ENC,  
         //40, DRIVE_TURN_LEFT_ENC,  
         35, DRIVE_TURN_LEFT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,
         //0.20, DRIVE_FORWARD_ENC_SLOW, 
         0.10, DRIVE_FORWARD_ENC_SLOW, 
         0.1, DRIVE_SHIFT_CLAW_LEFT,
         1.0, DRIVE_OPEN_CLAW,

         2.0, DRIVE_SHIFT_GEAR, 
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         //70,  DRIVE_TURN_RIGHT_ENC,  
         80,  DRIVE_TURN_RIGHT_ENC,  

         0.1, DRIVE_RESET_ENC_DONE,    
         0.2, DRIVE_FORWARD_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         30,  DRIVE_TURN_RIGHT_ENC,  
         0.1, DRIVE_RESET_ENC_DONE,    

         1.0, DRIVE_SHIFT_GEAR, 
         //60.0, DRIVE_STOP,             /// CENTER trip END, 35 steps

         //RIGHT  COLUMN
         90, DRIVE_TURN_LEFT_ENC, 
         0.1, DRIVE_RESET_ENC_DONE,   
         0.30, DRIVE_FORWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         40,  DRIVE_TURN_LEFT_ENC,
         //45,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_FORWARD_ENC_SLOW,

         // Extra step to drop glyph, back/forth, and then pick glyph, +8
         0.5, DRIVE_OPEN_CLAW_ONLY,     // drop glyph
         0.1, DRIVE_STOP,               
         0.1, DRIVE_RESET_ENC_DONE,   
         0.1, DRIVE_BACKWARD_ENC_SLOW,  // back off
         0.1, DRIVE_RESET_ENC_DONE,
         60, DRIVE_STOP,


         0.1, DRIVE_FORWARD_ENC_SLOW,   // forward
         0.5, DRIVE_CLOSE_CLAW,         // pick the glyph up
         (LIFT_ENC_COUNT_PER_GLYPH/6), DRIVE_LIFT_CLAWS,  // lift the claws up by ~1in 

         0.1, DRIVE_RESET_ENC_DONE,    
         0.15, DRIVE_BACKWARD_ENC_SLOW,
         0.1, DRIVE_RESET_ENC_DONE,   
         50,  DRIVE_TURN_LEFT_ENC,
         //55,  DRIVE_TURN_LEFT_ENC,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.10, DRIVE_FORWARD_ENC_SLOW,

         0.1, DRIVE_SHIFT_CLAW_LEFT,
         0.5, DRIVE_OPEN_CLAW,
         0.1, DRIVE_RESET_ENC_DONE,    
         0.1, DRIVE_BACKWARD_ENC,

         60.0, DRIVE_STOP
      };

      if( trip_name_ == "BluePos1") {
         return getDriveMode(blueTrip1, t);
      } else if( trip_name_ == "BluePos1SR") {
         return getDriveMode(blueTrip1SR, t);
      } else if( trip_name_ == "BluePos1OR") {
         return getDriveMode(blueTrip1OR, t);
      } else if( trip_name_ == "BluePos1ORIntake") {
         return getDriveMode(blueTrip1ORIntake, t);
      } else if( trip_name_ == "BluePos2SR") {
         return getDriveMode(blueTrip2SR, t);
      } else if( trip_name_ == "BluePos2OR") {
         return getDriveMode(blueTrip2OR, t);
      } else if( trip_name_ == "BluePos1QT") {
         return getDriveMode(blueTrip1QT, t);
      } else if( trip_name_ == "BluePos2") {
         //return getDriveMode(blueTrip2, t);
         return getDriveMode(blueTrip2QT, t);
      } else if( trip_name_ == "BluePos2SQ") {
         return getDriveMode(blueTrip2SQ, t);
      } else {
         return getDriveMode(blueTrip1, t);
      }
   }


/**************************************************************************** 
 * Begin of Encoder related codes
 ****************************************************************************/ 

   /// Access the left encoder's count.
   int getLFEncoderPos() {
      int l_return = 0;
      if( motorLF_ != null ) {
         l_return = motorLF_.getCurrentPosition();
      }
      return l_return;
   }
   int getLBEncoderPos() {
      int l_return = 0;
      if( motorLB_ != null ) {
         l_return = motorLB_.getCurrentPosition();
      }
      return l_return;
   }

   /// Access the right encoder's count.
   int getRFEncoderPos() {
      int l_return = 0;
      if( motorRF_ != null ) {
         l_return = motorRF_.getCurrentPosition();
      }
      return l_return;
   }
   int getRBEncoderPos() {
      int l_return = 0;
      if( motorRB_ != null ) {
         l_return = motorRB_.getCurrentPosition();
      }
      return l_return;
   }

   /// Indicate whether the left drive encoder has been completely reset.
   boolean isleftEncodersReset() {
      boolean l_return = false;
      if( getLFEncoderPos()==0 && getLBEncoderPos()==0 ) { l_return = true; }
      return l_return;
   }

   /// Indicate whether the left drive encoder has been completely reset.
   boolean isRightEncodersReset () {
      boolean l_return = false;
      if( getRFEncoderPos()==0 && getRBEncoderPos()==0 ) { l_return = true; }
      return l_return;
   }

   /// Indicate whether the encoders have been completely reset.
   boolean have_drive_encoders_reset () {
      boolean l_return = false;
      if( isleftEncodersReset() && isRightEncodersReset() ) {
         l_return = true;
      }
      return l_return;
   }

   /// Indicate whether the left front or left rear drive motor's encoder has reached a value.
   boolean isLeftEncodersReached (double p_count) {
      boolean l_return = false;
      if( motorLF_ != null ) {
         if (Math.abs (motorLF_.getCurrentPosition ()) >= p_count) {
            l_return = true;
         }
      }
      if( !l_return && motorLB_!=null) {
         if( Math.abs( motorLB_.getCurrentPosition() ) >= p_count ) {
            l_return = true;
         }
      }
      return l_return;
   }

   /// Indicate whether the right front or right rear drive motor's encoder has reached a value.
   boolean isRightEncodersReached (double p_count) {
      boolean l_return = false;
      if (motorRF_ != null) {
         if (Math.abs (motorRF_.getCurrentPosition ()) >= p_count) {
            l_return = true;
         }
      }
      if( !l_return && motorRB_!=null) {
         if( Math.abs( motorRB_.getCurrentPosition() ) >= p_count ) {
            l_return = true;
         }
      }
      return l_return;
   }

   /// Indicate whether the drive motors' encoders have reached a value.
   boolean have_drive_encoders_reached(double p_left_count, double p_right_count) {
      boolean l_return = false;
      if( isLeftEncodersReached(p_left_count) && isRightEncodersReached(p_right_count) ) {
         l_return = true;
      }
      return l_return;
   }

   /// Reset the left drive wheel encoder.
   public void resetLeftEncoders () {
      if (motorLF_ != null) {
         motorLF_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }
      if (motorLB_ != null) {
         motorLB_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }
   }

   /// Reset the right drive wheel encoder.
   public void resetRightEncoders () {
      if (motorRF_ != null) {
         motorRF_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }
      if (motorRB_ != null) {
         motorRB_.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }
   }

   /// Reset both drive wheel encoders.
   public void reset_drive_encoders () {
      resetLeftEncoders ();
      resetRightEncoders ();
   }

   /// Set the left drive wheel encoder to run, if the mode is appropriate.
   public void useLeftEncoders () {
      if (motorLF_ != null) {
         motorLF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
      }
      if (motorLB_ != null) {
         motorLB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
      }
   }

   /// Set the right drive wheel encoder to run, if the mode is appropriate.
   public void useRightEncoders () {
      if (motorRF_ != null) {
         motorRF_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
      }
      if (motorRB_ != null) {
         motorRB_.setMode ( DcMotor.RunMode.RUN_USING_ENCODER);
      }
   }

   /// Set both drive wheel encoders to run, if the mode is appropriate.
   public void run_using_encoders () {
      useLeftEncoders ();
      useRightEncoders ();
   }

   /// Set the left drive wheel encoder to run, if the mode is appropriate.
   public void useNoLeftEncoders () {
      if (motorLF_ != null) {
         if (motorLF_.getMode () == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            motorLF_.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         }
      }
      if (motorLB_ != null) {
         if (motorLB_.getMode () == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            motorLB_.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         }
      }
   }

   /// Set the right drive wheel encoder to run, if the mode is appropriate.
   public void useNoRightEncoders () {
      if (motorRF_ != null) {
         if (motorRF_.getMode () == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            motorRF_.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         }
      }
      if (motorRB_ != null) {
         if (motorRB_.getMode () == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            motorRB_.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         }
      }
   }

   /// Set all drive wheel encoders to run, if the mode is appropriate.
   //  Motors must be put in DcMotor.RunMode.RUN_WITHOUT_ENCODERS first, then safely disable encoders
   //  WARNING: RUN_WITHOUT_ENCODERS is deprecated. NOT working!
   public void run_without_drive_encoders() {
      useNoLeftEncoders ();
      useNoRightEncoders ();
   } 

/**************************************************************************** 
 * End of Encoder related codes
 ****************************************************************************/ 

   String formatAngle(AngleUnit angleUnit, double angle) {
      return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
   }               

   String formatDegrees(double degrees){
      return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
   }

   // Return values for a given Adafruit RGB connected to a specific port
   String getMuxRgbValues(int port){
      String s = "INVALID";
      return s;
   }


   /// Progressively scale up the heading error for a better heading correction for a given drive mode
   double scaleUpHeadingError(double err, boolean slow_drive) {
      double[] scaleUpperBound  = {0.0, 3.0, 5.0, 7.0, 10.0};    // heading error in degree

      // Original factor for gear ratio (2:3), 2016/12/01
      double[] scaleFactor      = {3.0, 2.5, 2.0, 1.5,  1.0};    // scale up factor; 0~3, 3.0X; 3~5, 2.5X; 5~7, 2.0X; 7~10, 1.5X; >10, 1.0X 
      double[] scaleFactorSlow  = {2.0, 1.7, 1.4, 1.2,  1.0};    // smaller scale up factors for slow drive mode if needed

      // New factors for high gear ratio (3:2), 2017/02/20; lower factors for high speed
      //double[] scaleFactor      = {2.0, 1.7, 1.4, 1.2,  1.0};    // lower scale up factors for fast drive mode 
      //double[] scaleFactorSlow  = {2.0, 1.7, 1.4, 1.2,  1.0};    // smaller scale up factors for slow drive mode if needed

      // Progressive scaling for smooth heading correction
      //  E.g. err=2 => adj_err=2*3.0=6; 3X
      //  E.g. err=4 => adj_err=1*2.5+3*3.0=11.5; 2.88X
      //  E.g. err=6 => adj_err=1*2.0+2*2.5+3*3.0=16.0; 2.67X
      //  E.g. err=8 => adj_err=1*1.5+2*2.0+2*2.5+3*3.0=19.5; 2.44X
      //  E.g. err=11 => adj_err=1*1.0+3*1.5+2*2.0+2*2.5+3*3.0=23.5; 2.13X 
      double abs_err = Math.abs(err); 
      double new_err = 0.0; 
      for( int i=scaleUpperBound.length-1; i>=0; --i ) {
         double ub = scaleUpperBound[i];
         double f = scaleFactor[i]; 
         if( slow_drive ) {
            f = scaleFactorSlow[i]; 
         }
         if( abs_err>ub ) {
            new_err += (abs_err-ub)*f;
            abs_err = ub; 
         }
      } 

      if( err<0.0 ) new_err *= -1.0; 
      return new_err; 
   }

}
