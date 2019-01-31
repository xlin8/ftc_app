/* Copyright (c) 2018 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name = "Y18 TensorFlow Object Detection Webcam", group = "Concept")
// @Disabled
public class Y18ConceptTensorFlowObjectDetectionWebcam extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AS0FKrL/////AAABmTcBCNs1gE8uh4tntGA7HSgXRT5npDQpV2pw5tlqbZCI6WJQRf0bKf458A218bGkQJCWkJzvJy6UtNnhziraRVDDZSnTSZGSD7s3WN9jNYqBiSoO3CVE6FU2dX1yuJNa1zfiEhcGT8ChTd+kucE/q3sXsy/nw1KqlW/7uEaEeRwaCPseqsbNrc1HZ1bi18PzwQpCaypDruqqVEyZ3dvTqDmjPg7WFBe2kStPR/qTtcLSXdE804RxxkgTGUtDMIG7TTbAdirInGVZw2p2APZKAQdYofYW2E0Ss5hZCeL55zflSuQK0QcW1sAyvaTUMd/fDse4FgqxhnfK0ip0Kc+ZqZ6XJpof/Nowwxv3IgDWZJzO";

    private static final int GOLD_MINERAL_AT_LEFT = 0;
    private static final int GOLD_MINERAL_AT_CENTER = 1;
    private static final int GOLD_MINERAL_AT_RIGHT = 2;
    private static final int GOLD_MINERAL_AT_UNKNOWN = 3;
    private static double MIN_MINERAL_HEIGHT_RATIO = 0.6;
    private static double MIN_MINERAL_HEIGHT_WIDTH_RATIO = 0.75;
    private static double MAX_MINERAL_BOTTOM_DIFF_RATIO = 0.4;

    VuforiaLocalizer vuforia_;
    WebcamName webcamName_;

    TFObjectDetector tfod_;

    // Detect gold mineral position
    int goldPosition_ = GOLD_MINERAL_AT_UNKNOWN;
    boolean isGuessedGoldPosition_ = false;

    @Override
    public void runOpMode() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();

            /** Activate Tensor Flow Object Detection. */
            if (tfod_ != null) {
                tfod_.activate();
            }
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        waitForStart();

        telemetry.addData("Start automonous", "(GoldAt="+String.valueOf(goldPosition_)+" IsGuessed="+String.valueOf(isGuessedGoldPosition_)+")");
        telemetry.update();

        if (tfod_ != null) {
            tfod_.shutdown();
        }

        while (opModeIsActive()) {
            idle();
        }
    }

    @Override
    public synchronized void waitForStart() {
        while (!isStarted()) {
            synchronized (this) {
                try {
                    int curr_gold_pos = detectMineralsByTfod();
                    if (curr_gold_pos != GOLD_MINERAL_AT_UNKNOWN) {
                        goldPosition_ = curr_gold_pos;
                        isGuessedGoldPosition_ = false;
                    }

                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }

    private void initVuforia() {
        webcamName_ = hardwareMap.get(WebcamName.class, "Webcam");

        // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName_;

        vuforia_ = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod_ = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia_);
        tfod_.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private int detectMineralsByTfod() {
        if (tfod_ == null) return GOLD_MINERAL_AT_UNKNOWN;

        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod_.getUpdatedRecognitions();
        if (updatedRecognitions != null ) {
            int num_detect_obj = updatedRecognitions.size();
            telemetry.addData("# Object Detected", num_detect_obj);

            if (num_detect_obj >= 2) {
                int [] mineral_y = {-1, -1, -1};
                int [] mineral_x = {-1, -1, -1};
                int [] mineral_height = {-1, -1, -1};
                int [] mineral_width = {-1, -1, -1};
                boolean [] is_gold_mineral_flag = {false, false, false};
                int num_bottom_minerals = 0;
                for (Recognition recognition : updatedRecognitions) {
                    int curr_x = (int) recognition.getLeft();
                    int curr_y = (int) recognition.getBottom();
                    int curr_height = (int) recognition.getHeight();
                    int curr_width = (int) recognition.getWidth();
                    boolean curr_is_gold_flag = false;
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) curr_is_gold_flag = true;

                    if (num_bottom_minerals < 3) {
                        mineral_x[num_bottom_minerals] = curr_x;
                        mineral_y[num_bottom_minerals] = curr_y;
                        mineral_height[num_bottom_minerals] = curr_height;
                        mineral_width[num_bottom_minerals] = curr_width;
                        is_gold_mineral_flag[num_bottom_minerals] = curr_is_gold_flag;
                        ++num_bottom_minerals;
                    } else {
                        int top_y = mineral_y[0];
                        int top_id = 0;
                        for (int i = 1; i < 3; ++i) {
                            if (mineral_y[i] < top_y) {
                                top_y = mineral_y[i];
                                top_id = i;
                            }
                        }

                        if (curr_y > top_y) {
                            mineral_x[top_id] = curr_x;
                            mineral_y[top_id] = curr_y;
                            mineral_height[top_id] = curr_height;
                            mineral_width[top_id] = curr_width;
                            is_gold_mineral_flag[top_id] = curr_is_gold_flag;
                        }
                    }
                }

                // Sort mineral array in dcreasing order of Y
                for (int i=0; i<num_bottom_minerals-1; ++i) {
                    for (int j=i; j<num_bottom_minerals-1; ++j) {
                        if (mineral_y[j] < mineral_y[j+1]) {
                            int tmp = mineral_x[j];
                            mineral_x[j] = mineral_x[j+1];
                            mineral_x[j+1] = tmp;
                            tmp = mineral_y[j];
                            mineral_y[j] = mineral_y[j+1];
                            mineral_y[j+1] = tmp;
                            tmp = mineral_height[j];
                            mineral_height[j] = mineral_height[j+1];
                            mineral_height[j+1] = tmp;
                            boolean tmp_bool = is_gold_mineral_flag[j];
                            is_gold_mineral_flag[j] = is_gold_mineral_flag[j+1];
                            is_gold_mineral_flag[j+1] = tmp_bool;
                        }
                    }
                }

                if (is_gold_mineral_flag[0] == false || is_gold_mineral_flag[1] == false) {
                    boolean has_two_mineral_in_bottem_flag = true;

                    if (num_bottom_minerals == 3) {
                        if (mineral_height[2] >= mineral_width[2]) {
                            if (((double)mineral_width[2] / (double)mineral_height[2]) < MIN_MINERAL_HEIGHT_WIDTH_RATIO) num_bottom_minerals = 2;
                        } else {
                            if (((double)mineral_height[2] / (double)mineral_width[2]) < MIN_MINERAL_HEIGHT_WIDTH_RATIO) num_bottom_minerals = 2;
                        }

                        if (num_bottom_minerals == 3) {
                            int diff_y = mineral_y[1] - mineral_y[2];
                            if (diff_y >= mineral_height[1]) num_bottom_minerals = 2;
                            else if (((double)diff_y / (double)mineral_height[1]) > MAX_MINERAL_BOTTOM_DIFF_RATIO) num_bottom_minerals = 2;

                            if (num_bottom_minerals == 3) {
                                if (mineral_height[2] > mineral_height[1]) {
                                    if (((double) mineral_height[1] / (double) mineral_height[2]) >= MIN_MINERAL_HEIGHT_RATIO)
                                        has_two_mineral_in_bottem_flag = false;
                                } else {
                                    if (((double) mineral_height[2] / (double) mineral_height[1]) >= MIN_MINERAL_HEIGHT_RATIO)
                                        has_two_mineral_in_bottem_flag = false;
                                }
                            }
                        }
                    }


                    if (has_two_mineral_in_bottem_flag==true) {
                        int diff_y = mineral_y[0] - mineral_y[1];
                        if (diff_y >= mineral_height[0]) has_two_mineral_in_bottem_flag = false;
                        else if (((double)diff_y / (double)mineral_height[0]) > MAX_MINERAL_BOTTOM_DIFF_RATIO) has_two_mineral_in_bottem_flag = false;

                        /*
                        if (mineral_height[0] > mineral_height[1]) {
                            if (((double)mineral_height[1] / (double)mineral_height[0]) < MIN_MINERAL_HEIGHT_RATIO)
                                has_two_mineral_in_bottem_flag = false;
                        } else {
                            if (((double)mineral_height[0] / (double)mineral_height[1]) < MIN_MINERAL_HEIGHT_RATIO)
                                has_two_mineral_in_bottem_flag = false;
                        }
                        */

                        if (has_two_mineral_in_bottem_flag == true){
                            if (is_gold_mineral_flag[0] == false && is_gold_mineral_flag[1] == false) {
                                telemetry.addData("Gold Mineral Position", "Left (%d, %d, %d) (%d, %d, %d)",
                                        mineral_x[0], mineral_y[0], mineral_height[0], mineral_x[1], mineral_y[1], mineral_height[1]);
                                return GOLD_MINERAL_AT_LEFT;
                            } else {
                                boolean is_in_center_flag = false;
                                if (is_gold_mineral_flag[0] == true) {
                                    if (mineral_x[0] < mineral_x[1]) is_in_center_flag = true;
                                } else {
                                    if (mineral_x[0] > mineral_x[1]) is_in_center_flag = true;
                                }

                                if (is_in_center_flag == true) {
                                    telemetry.addData("Gold Mineral Position", "Center (%d, %d, %d) (%d, %d, %d)",
                                            mineral_x[0], mineral_y[0], mineral_height[0], mineral_x[1], mineral_y[1], mineral_height[1]);
                                    return GOLD_MINERAL_AT_CENTER;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Right (%d, %d, %d) (%d, %d, %d)",
                                            mineral_x[0], mineral_y[0], mineral_height[0], mineral_x[1], mineral_y[1], mineral_height[1]);
                                    return GOLD_MINERAL_AT_RIGHT;
                                }
                           }
                       }
                    }
                }
            }
        } else {
            telemetry.addData("Empty detected object list", "(LastGoldAt="+String.valueOf(goldPosition_)+" IsGuessed="+String.valueOf(isGuessedGoldPosition_)+")");
        }

        telemetry.update();

        return GOLD_MINERAL_AT_UNKNOWN;
    }
}