/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
        package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the Hardware class to define the devices on the robot.
 * All device access is managed through the Hardware class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a Tank Drive style Teleop for a PushBot
 * In this mode the left stick controls the direction of the left motors, and
 * the Right stick controls the right motor direction
 */

@TeleOp(name="AndyMark Teleop", group="Official")

public class OfficialTeleop_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot           = new Hardware();   // Use a hardware class

    @Override
    public void runOpMode() {
        double left = 0;
        double right = 0;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.left_bumper) {
                left = (1 - .5 * gamepad1.left_trigger) * (-gamepad1.left_stick_y + (1 - .5 * Math.abs(gamepad1.left_stick_y)) * gamepad1.right_stick_x);
                right = (1 - .5 * gamepad1.left_trigger) * (-gamepad1.left_stick_y - (1 - .5 * Math.abs(gamepad1.left_stick_y)) * gamepad1.right_stick_x);
            }
            else
            {
                left = Math.pow((1 - .206 * gamepad1.left_trigger) * (-gamepad1.left_stick_y + (1 - .5 * Math.abs(gamepad1.left_stick_y)) * gamepad1.right_stick_x), 3);
                right = Math.pow((1 - .206 * gamepad1.left_trigger) * (-gamepad1.left_stick_y - (1 - .5 * Math.abs(gamepad1.left_stick_y)) * gamepad1.right_stick_x), 3);
            }


            //left = Range.clip(left, -1, 1);
            //right = Range.clip(right, -1, 1);

            //ETHAN PLEASE EXPLAIN HOW THIS BELOW STATEMENT WORKS
            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }


            //Set Motor Power to gamepad values
            robot.lfMotor.setPower(left);
            robot.rfMotor.setPower(right);


            //buttonArm Motion (right trigger button value changes servo position)
            double servoOffSet = 1-.33*gamepad1.right_trigger;
            if (gamepad1.right_trigger>.01){
                robot.buttonArm.setPosition(servoOffSet);
            }
            else if(gamepad1.right_trigger == 0){
                robot.buttonArm.setPosition(1);
            }

            //buttonMasher Motion
            if (gamepad1.right_bumper) {
                robot.buttonMasher.setPosition(0+.33*gamepad1.right_trigger);
            } else{
                robot.buttonMasher.setPosition(.25+.33*gamepad1.right_trigger);
            }



            //if(gamepad2.dpad_up){
            //    robot.linearSlide.setPower(1);
            //} else if(gamepad2.dpad_down){
            //    robot.linearSlide.setPower(-1);
            //}else{
            //    robot.linearSlide.setPower(0);
            //}


            //if(gamepad2.a){
            //    robot.flyWheelRight.setPower(1);
            //    robot.flyWheelLeft.setPower(1);
            //}else{
            //    robot.flyWheelRight.setPower(0);
            //    robot.flyWheelLeft.setPower(0);
            //}



            //robot.pulley.setPower(-gamepad2.right_stick_y);


        }



        // Send telemetry message to signify robot running;
            /*telemetry.addData("claw",  "Offset = %.2f", clawOffset);*/


        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.update();

        // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
        robot.waitForTick(40);
    }
}

