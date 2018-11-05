/**
 * Put all common hardware configuration here
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//import com.qualcomm.robotcore.util.Range;


/**
 * Common codes 
 */
public class Y17TestCommon extends OpMode {

   /// Main timer
   ElapsedTime timer_ = new ElapsedTime();

   /// Motors
   DcMotor motorRF_;
   DcMotor motorRB_;
   DcMotor motorLF_;
   DcMotor motorLB_;

   //static final boolean USE_SWEEPER = true;
   static final boolean USE_SWEEPER = false;
   DcMotor motorSweeper_;

   //static final boolean USE_LIFT = true;
   static final boolean USE_LIFT = false;
   DcMotor motorLift_;

   /// Servos
   Servo servo_left_jaw_;                                       // servo aligned with Adafruit RGB
   Servo servo_right_jaw_;                                        // servo aligned with MR RGB
   double servo_left_jaw_pos_;                                  // Ada servo position
   double servo_right_jaw_pos_;                                   // MR servo position

   /// Servo Variables
   static final double  CR_SERVO_STOP = 0.5;
   static final double  LEFT_JAW_OPEN = 1.0;
   static final double  LEFT_JAW_OPEN_SLOW = 0.7;
   static final double  LEFT_JAW_CLOSE = 0.0;
   static final double  LEFT_JAW_CLOSE_HOLD = 0.25;
   static final double  RIGHT_JAW_OPEN = 1.0;
   static final double  RIGHT_JAW_OPEN_SLOW = 0.7;
   static final double  RIGHT_JAW_CLOSE = 0.0;
   static final double  RIGHT_JAW_CLOSE_HOLD = 0.25;


   /// IMU
   static final boolean USE_IMU  = true;           // use IMU sensor to turn 
   //static final boolean USE_IMU  = false;           // use IMU sensor to turn 
   BNO055IMU imu_;                         // Adafruit or RevHub IMU 
   BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
   Orientation imu_angles_;
   //Acceleration imu_gravity_;
   double heading_ = 0.0;                                  // current heading 

   /// Joy sticks
   static final float JOYSTICK_DEAD_ZONE = 0.1f; 
   int a1_cnt_;                 // number of times pad1/A is pressed
   int b1_cnt_;                 // number of times pad1/B is pressed
   int x1_cnt_;                 // number of times pad1/X is pressed
   int y1_cnt_;                 // number of times pad1/Y is pressed
   int lb1_cnt_;                // number of times pad1/left_bumper is pressed
   int rb1_cnt_;                // number of times pad1/right_bumper is pressed

   int a2_cnt_;                 // number of times pad2/A is pressed
   int b2_cnt_;                 // number of times pad2/B is pressed
   int x2_cnt_;                 // number of times pad2/X is pressed
   int y2_cnt_;                 // number of times pad2/Y is pressed
   int lb2_cnt_;                // number of times pad2/left_bumper is pressed
   int rb2_cnt_;                // number of times pad2/right_bumper is pressed 

   double last_button_time_;
   double last_button_time2_;

   /// Other variables
   double curr_time_; 

   /// Battery voltage
   double battery_voltage_=0.0;     // battery voltage 


   ///  Constructor
   public Y17TestCommon() {
   }


   ///  Code to run when the op mode is initialized goes here
   @Override public void init() {
      /// Use the hardwareMap to get the dc motors and servos by name.



      motorLF_ = hardwareMap.dcMotor.get("motor1");
      motorLB_ = hardwareMap.dcMotor.get("motor2");
      motorRF_ = hardwareMap.dcMotor.get("motor3");
      motorRB_ = hardwareMap.dcMotor.get("motor4");

      // Reverse motor, required for RevHub 
      motorLF_.setDirection(DcMotor.Direction.REVERSE);
      motorLB_.setDirection(DcMotor.Direction.REVERSE);
      motorRF_.setDirection(DcMotor.Direction.REVERSE);
      motorRB_.setDirection(DcMotor.Direction.REVERSE);

      if(USE_SWEEPER) motorSweeper_ = hardwareMap.dcMotor.get("motor5");
      if(USE_LIFT) motorLift_ = hardwareMap.dcMotor.get("motor6");

      servo_left_jaw_ = hardwareMap.servo.get("servo_left_jaw");
      servo_right_jaw_ = hardwareMap.servo.get("servo_right_jaw");
      servo_left_jaw_pos_ = CR_SERVO_STOP;
      servo_right_jaw_pos_ = CR_SERVO_STOP;
      servo_left_jaw_.setPosition(servo_left_jaw_pos_);
      servo_right_jaw_.setPosition(servo_right_jaw_pos_);


      if( USE_IMU ) {
         parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
         parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
         parameters.loggingEnabled      = true;
         parameters.loggingTag          = "IMU";
         parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

         imu_ = hardwareMap.get(BNO055IMU.class, "imu");
         imu_.initialize(parameters);
         //while( !imu_.isSystemCalibrated() ) { }    // Error: cause OpMode stuck in init(), and robot to stop
         imu_.startAccelerationIntegration(new Position(), new Velocity(), 1000);
      }

      /// Set joystick deadzone, any value below this threshold value will be considered as 0; moved from init() to init_loop() to aovid crash
      if(gamepad1!=null) gamepad1.setJoystickDeadzone( JOYSTICK_DEAD_ZONE ); 
      if(gamepad2!=null) gamepad2.setJoystickDeadzone( JOYSTICK_DEAD_ZONE ); 

      curr_time_ = 0.0; 
      last_button_time_ = 0.0;
      a1_cnt_ = 0;
      b1_cnt_ = 0;
      x1_cnt_ = 0;
      y1_cnt_ = 0;
      lb1_cnt_ = 0;
      rb1_cnt_ = 0;

      last_button_time2_ = 0.0;
      a2_cnt_ = 0;
      b2_cnt_ = 0;
      x2_cnt_ = 0;
      y2_cnt_ = 0;
      lb2_cnt_ = 0;
      rb2_cnt_ = 0;

   }


   /// This method will be called once before entering loop()
   @Override public void init_loop() {
      curr_time_ = 0.0;
   }

   /// This method will be called repeatedly in a loop
   @Override public void loop () { 
   }


   /// Code to run when the op mode is first disabled goes here
   @Override public void stop () { 
   } 

   /// Return true if it's RED team
   boolean isRedTeam() {
      //return ( START_AS_RED ? (!color_flipped_) : color_flipped_ );
      return true;
   }

   /// Return current robot heading based on gyro/IMU reading
   double getHeading() {
       heading_ = 0;
       if( USE_IMU && imu_!=null ) {
         imu_angles_  = imu_.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);  // acquiring angles are expensive, keep it minimal
         heading_ = AngleUnit.DEGREES.normalize(imu_angles_.firstAngle);
      }
      return heading_;
   }


   /// Computes the current battery voltage
   // Copy from /robotcontroller/external/samples/ConceptTelemetry.java
   double getBatteryVoltage(boolean first_reading) {
      double result = Double.POSITIVE_INFINITY;
      for (VoltageSensor sensor : hardwareMap.voltageSensor) {
         double voltage = sensor.getVoltage();
         if (voltage > 0) {
            result = Math.min(result, voltage);
            if(first_reading) break;    // return the first valid reading (>0.0) to minimize overhead
         }
      }
      return result;
   }

   int numVoltageSensors() {
      int num = 0; 
      for( VoltageSensor sensor : hardwareMap.voltageSensor ) {
         ++num;
      }
      return num;
   }
} 
