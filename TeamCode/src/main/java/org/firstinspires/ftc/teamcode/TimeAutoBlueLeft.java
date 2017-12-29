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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Thronebot: TimeAuto(BlueLeft)", group="Thronebot")

public class TimeAutoBlueLeft extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    ColorSensor color_sensor;


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    static Boolean LEFT = false;
    static Boolean CENTER = false;
    static Boolean RIGHT = false;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        color_sensor = hardwareMap.colorSensor.get("color");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // START HERE: THIS AUTONOMOUS CASE REQUIRES AN EXTRA TURN. LEFT IS CLOSEST
        runtime.reset();

        /*  ***********************************************
        *   READ COLOR OF RELICS: ***TEST FOR VALUES OF RED AND BLUE***
        *   For x seconds
        *   ***********************************************
        */
        while(runtime.seconds() <= x) {
            telemetry.addData("TIME: ", runtime.seconds());
            telemetry.addData("Red: ", color_sensor.red());
            telemetry.addData("Blue: ", color_sensor.blue());
            telemetry.update();

            color_sensor.enableLed(true);
            // color_sensor.enableLed(false);
        } // Runtime: xs

        /*  ***********************************************
        *   LOWER ARM
        *   For x seconds
        *   ***********************************************
        */
        while(runtime.seconds() <= x) {
            telemetry.addData("TIME: ", runtime.seconds());
            telemetry.update();
            color_sensor.enableLed(true);

            // Lower robot arm

            // BLUETEAM: NUDGE RED RELIC. ASSUME ONLY SCANNING LEFT BALL
            // Robot arm swipes left or right ***TEST FOR VALUES OF RED AND BLUE***

        } // Runtime: xs

        /*  ***********************************************
        *   MOVE TOWARDS RELICS ???????
        *   For x seconds
        *   ***********************************************
        */

        /*  ***********************************************
        *   SCAN PICTOGRAPH
        *   MOVE TOWARDS RELICS FIRST???????
        *   For x seconds
        *   ***********************************************
        */
        while(runtime.seconds() <= x) {
            telemetry.addData("TIME: ", runtime.seconds());
            telemetry.update();
            color_sensor.enableLed(true);

            // Lower robot arm

            // BLUETEAM: NUDGE RED RELIC. ASSUME ONLY SCANNING LEFT BALL
            // Robot arm swipes left or right ***TEST FOR VALUES OF RED AND BLUE***

        } // Runtime: xs

        /*  ***********************************************
        *   NUDGE RELIC
        *   For x seconds
        *   ***********************************************
        */
        while() {
            // BLUETEAM: NUDGE RED RELIC. ASSUME ONLY SCANNING LEFT RELIC
            if (color_sensor.red() > 20) {
                // move entire robot back to nudge red ball
            } else {
                // move entire robot forward to nudge red ball
            }
        }

        /*  ***********************************************
        *   MOVE TO CRYPTOBOX, DEPENDING ON WHAT CAMERA READS
        *   ***********************************************
        */
        // DEPENDING ON WHAT THE CAMERA READS FROM PICTOGRAPH:
        // Move to first column, turn if necessary, place down glyph
        if (camera reads left){
            // 2 seconds
            while (runtime.seconds() <= x) {
                // print to console
                telemetry.addData("TIME: ", runtime.seconds());
                telemetry.update();

                // move to first column
            } // runtime xs

            // while to move

            // while to place glyph down
        }
        else if (camera reads center) {
            // 2.5 seconds
            // while to move to center column
            while (runtime.seconds() <= x) {
                // print to console
                telemetry.addData("TIME: ", runtime.seconds());
                telemetry.update();

                // move to center column

            } // runtime xs

            // while to move

            // while to place glyph down
        }
        else (camera reads right) {
            // 3 seconds
            // while to move to right column
            while (runtime.seconds() <= x) {
                // print to console
                telemetry.addData("TIME: ", runtime.seconds());
                telemetry.update();

                // move to last column

            } // runtime xs

            // while to move

            // while to place glyph down
        }

    }
}
