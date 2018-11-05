/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Demonstrates empty OpMode
 */
//@Autonomous(name = "Concept: NullOp", group = "Concept")
@TeleOp(name="ServoTest", group="GG")
//@Disabled
public class Y17ServoTest extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  double last_button_time_ ;

  boolean TEST_JEWEL_SERVOS = false;
  /// aUTONOMOUS SERVOS
  Servo servo_jewel_;                           // servo for rising/lowering long stick
  Servo servo_knock_;                           // servo for knocking the jewel
  Servo servo_pusher_;                           // servo for knocking the jewel
  double servo_jewel_pos_;
  double servo_knock_pos_;
  double servo_pusher_pos_;
  static final double  JEWEL_INIT = 0.5;
  static final double  KNOCK_INIT = 0.5;
  static final double  PUSHER_INIT = 0.5;




  boolean TEST_RAMPCLAMP_SERVOS = true;
  Servo servo_front_clamp_;                              // front ramp clamp servo
  Servo servo_back_clamp_;                               // back ramp clamp servo
  double servo_front_slide_pos;
  double servo_back_slide_pos;
  static final double FRONT_SLIDE_INIT = 0.0;
  static final double BACK_SLIDE_INIT = 0.0;
  static final double FRONT_SLIDE_HOLD = 0.0;
  static final double BACK_SLIDE_HOLD = 0.0;



  @Override
  public void init() {
    if(TEST_JEWEL_SERVOS) {
      servo_jewel_ = hardwareMap.servo.get("servo_jewel");
      servo_knock_ = hardwareMap.servo.get("servo_knock");
      servo_jewel_pos_ = JEWEL_INIT;
      servo_knock_pos_ = KNOCK_INIT;
    }
    //servo_pusher_pos_ = PUSHER_INIT;
    last_button_time_ = 0.0;

    if(TEST_RAMPCLAMP_SERVOS){
      servo_front_clamp_ = hardwareMap.servo.get("servo_front_clamp");
      servo_back_clamp_ = hardwareMap.servo.get("servo_back_clamp");
      servo_back_slide_pos = 0.5;
      servo_front_slide_pos = 0.5; //0.5 is init position

      servo_back_clamp_.setPosition(servo_back_slide_pos);
      servo_front_clamp_.setPosition(servo_front_slide_pos);
    }


    telemetry.addData("Status", "Initialized");
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());


    if (TEST_RAMPCLAMP_SERVOS) {
      double t = runtime.time();
      if (t > last_button_time_ + 0.2) {

        if (gamepad1.dpad_up) {
          servo_front_slide_pos += 0.1;
          last_button_time_ = t;
        } else if (gamepad1.dpad_down) {
          servo_front_slide_pos -= 0.1;
          last_button_time_ = t;
        }
        else if (gamepad1.dpad_left) {
          servo_back_slide_pos += 0.1;
          last_button_time_ = t;
        } else if (gamepad1.dpad_right) {
          servo_back_slide_pos -= 0.1;
          last_button_time_ = t;
        }
      }
    }



    if (TEST_JEWEL_SERVOS) {
      double t = runtime.time();
      if (t > last_button_time_ + 0.2) {
        if (gamepad1.dpad_up) {
          servo_jewel_pos_ += 0.1;
          last_button_time_ = t;
        } else if (gamepad1.dpad_down) {
          servo_jewel_pos_ -= 0.1;
          last_button_time_ = t;
        } else if (gamepad1.dpad_left) {
          servo_knock_pos_ += 0.1;
          last_button_time_ = t;
        } else if (gamepad1.dpad_right) {
          servo_knock_pos_ -= 0.1;
          last_button_time_ = t;
        } else if (gamepad1.a) {
          servo_pusher_pos_ += 0.1;
          last_button_time_ = t;
        } else if (gamepad1.b) {
          servo_pusher_pos_ -= 0.1;
          last_button_time_ = t;
        }
      }
    }

    if(TEST_RAMPCLAMP_SERVOS){
      servo_front_slide_pos = Range.clip(servo_front_slide_pos, 0, 1);
      servo_back_slide_pos = Range.clip(servo_back_slide_pos, 0, 1);

      servo_front_clamp_.setPosition( servo_front_slide_pos );
      servo_back_clamp_.setPosition( servo_back_slide_pos);

      telemetry.addData("Ramp Clamps", String.format("front slide pos=%.2f, back slide pos=%.2f",
              servo_front_clamp_.getPosition(),servo_back_clamp_.getPosition()));


    }

    if(TEST_JEWEL_SERVOS){

      servo_jewel_pos_ = Range.clip(servo_jewel_pos_, 0, 1);
      servo_jewel_.setPosition( servo_jewel_pos_ );
      servo_knock_pos_ = Range.clip(servo_knock_pos_, 0, 1);
      servo_knock_.setPosition( servo_knock_pos_ );
      telemetry.addData("Jewel Module", String.format("jewel_pos=%.2f, knock_pos=%.2f",
              servo_jewel_.getPosition(),servo_knock_.getPosition()));

      servo_pusher_pos_ = Range.clip(servo_pusher_pos_, 0, 1);
      //servo_pusher_.setPosition( servo_pusher_pos_ );
      telemetry.addData("Glyph Pusher", String.format("pusher_pos=%.2f", servo_pusher_pos_));

    }
  }
}
