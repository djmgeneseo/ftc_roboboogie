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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.sun.tools.javac.api.ClientCodeWrapper;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Thronebot: MechanumTank", group="Thronebot")

public class ThronebotTelopMechanumTank extends OpMode{

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.HardwareThronebot robot = new org.firstinspires.ftc.teamcode.HardwareThronebot(); // use the class created to define a Pushbot's hardware

    public boolean g1xSlowSwitch = false;
    public boolean g2xSwitch = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //ColorSensor colorSensor;
        //colorSensor = hardwareMap.get(ColorSensor.class, "Sensor_Color");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Say", "We pressed INIT!");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /************************
         * CONTROLLER 1
         *  left stick(y) = left-side wheels
         *  right stick(y) = right-side wheels
         *  left trigger = strafe left
         *  right trigger = strafe right
         *  ***********************
         */

        double g1leftStick;
        double g1rightStick;
        double g1leftTrigger;
        double g1rightTrigger;

        if(!g1xSlowSwitch) {
            g1leftStick = gamepad1.left_stick_y * .20;
            g1rightStick = gamepad1.right_stick_y * .20;
            g1leftTrigger = gamepad1.left_trigger * .20;
            g1rightTrigger = gamepad1.right_trigger * .20;
        } else {
            g1leftStick = gamepad1.left_stick_y * .05;
            g1rightStick = gamepad1.right_stick_y * .05;
            g1leftTrigger = gamepad1.left_trigger * .05;
            g1rightTrigger = gamepad1.right_trigger * .05;
        }
        boolean g1x = gamepad1.x;


        /************************
         * CONTROLLER 2
         *  left stick(y) = move the lift
         *  x button = open/close glyph claws
         ************************
         */

        double g2leftStick2 = gamepad2.left_stick_y;
        boolean g2x = gamepad2.x;

        // display statements
        telemetry.addData("LEFT STICK VALUE: ",g1leftStick);
        telemetry.addData("RIGHT STICK VALUE: ",g1rightStick);
        telemetry.addData("LEFT TRIGGER POWER: ",g1leftTrigger);
        telemetry.addData("RIGHT TRIGGER POWER: ",g1rightTrigger);
        telemetry.update();

        /************************
         *  CONTROLLER 1 - LOGIC
         ************************
         */

        // SLOW-DOWN TOGGLE with 'x'
        if(g1x) {
            g1xSlowSwitch = !g1xSlowSwitch;
        }

        // TRIGGERS AND STICKS
        // If the triggers are being pushed, cannot use sticks
        if(g1leftTrigger > 0 || g1rightTrigger > 0) {
            if (g1leftTrigger > 0) {
                // strafe left
                robot.frontLeft.setPower(-g1leftTrigger);
                robot.backLeft.setPower(g1leftTrigger);
                robot.frontRight.setPower(g1leftTrigger);
                robot.backRight.setPower(-g1leftTrigger);
            } else if (g1rightTrigger > 0) {
                // strafe right
                robot.frontLeft.setPower(g1rightTrigger);
                robot.backLeft.setPower(-g1rightTrigger);
                robot.frontRight.setPower(-g1rightTrigger);
                robot.backRight.setPower(g1rightTrigger);
            }
        } else {
            // movement with sticks
            robot.frontLeft.setPower(g1leftStick);
            robot.backLeft.setPower(g1leftStick);
            robot.backRight.setPower(g1rightStick);
            robot.frontRight.setPower(g1rightStick);
        }


        /************************
         *  CONTROLLER 2 - LOGIC
         ************************
         */
        // LEFT STICK
        if (g2leftStick2 > 0 ){
            robot.lift.setPower(g2leftStick2);
        } else if (g2leftStick2 < 0){
            robot.lift.setPower(g2leftStick2);
        }
        else {
            // If left stick(y) is not being moved
            robot.lift.setPower(0);
        }

        // X BUTTON
        while(g2x) {
            if(g2xSwitch) {
                while (g2x) {
                    // is 1 open or close????
                    robot.clawRight.setPosition(1);
                    robot.clawLeft.setPosition(0);
                }
                robot.clawRight.setPosition(robot.clawRight.getPosition());
                robot.clawLeft.setPosition(robot.clawLeft.getPosition());
                g2xSwitch = !g2xSwitch;
            } else {
                while (g2x) {
                    robot.clawLeft.setPosition(0);
                    robot.clawRight.setPosition(1);
                }
                robot.clawRight.setPosition(robot.clawRight.getPosition());
                robot.clawLeft.setPosition(robot.clawLeft.getPosition());
                g2xSwitch = !g2xSwitch;
            }
        }
    } // LOOP

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Say", "It Stopped!");
    }
}