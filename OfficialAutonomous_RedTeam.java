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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the Hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 */

@Autonomous(name="AndyMark Autonomous Red Team", group="Official")
public class OfficialAutonomous_RedTeam extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware         robot   = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: ANDYMARK Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.141592653589);
    static final double     DRIVE_SPEED             = .18;
    static final double     TURN_SPEED              = 0.2;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        robot.lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();


        robot.lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                            robot.lfMotor.getCurrentPosition(),
                            robot.rfMotor.getCurrentPosition()
                            //robot.lfMotor.getCurrentPosition(),
                            //robot.rfMotor.getCurrentPosition()
        );
        telemetry.update();


        //calibrate gyro
        robot.GyroSensor.calibrate();
        while(robot.GyroSensor.isCalibrating()){
            sleep(50);
            idle();
        }
        robot.ColorSensorA.enableLed(false);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //Use encoderDrive for driving forward. Use gyroTurn for turning. gyroTurn uses ABSOLUTE HEADING.
        //THE ANGLE USED IN gyroTurn IS RELATIVE TO THE ORIGINAL STARTING POINT WHEN YOU FIRST STARTED THE OPMODE
        //Negative angle is to the right of the original starting orientation
        //Positive angle is to the left of the original starting orientation (front of robot = 0 degrees angle)





        encoderDrive(DRIVE_SPEED, 9, 9, 30);
        sleep(200);
        gyroTurn(TURN_SPEED, 45);
        sleep(200);
        encoderDrive(DRIVE_SPEED, 46, 46, 30);
        sleep(200);
        gyroTurn(TURN_SPEED, 0);
        sleep(200);
        encoderDrive(DRIVE_SPEED, 12, 12, 30);
        sleep(200);
        pushButton();
        sleep(100);
        encoderDrive(DRIVE_SPEED, 47, 47, 30);
        sleep(200);
        pushButton();
        sleep(100);


        //pushButton is the function used for autonomous button pushing
        //see the function itself for how it works (ask Brian)(no figure it out for your self WENDELL WU)



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.lfMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rfMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.lfMotor.setTargetPosition(newLeftTarget);
            robot.rfMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.lfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.lfMotor.setPower(Math.abs(speed));
            robot.rfMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.lfMotor.isBusy() && robot.rfMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.lfMotor.getCurrentPosition(),
                                            robot.rfMotor.getCurrentPosition()
                        );
                telemetry.update();
            }

            // Stop all motion;
            robot.lfMotor.setPower(0);
            robot.rfMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

    public void pushButton() {

        int sensorDataBlue = robot.ColorSensorA.blue();
        int sensorDataRed = robot.ColorSensorA.red();
        robot.buttonArm.setPosition(1);
        if (sensorDataBlue > 3) {
            //go back 4 1/2 inches first
            encoderDrive(DRIVE_SPEED, -3.5, -3.5, 9);
            robot.buttonArm.setPosition(.7);
            sleep(100);
            robot.buttonArm.setPosition(1);
            sleep(100);
            robot.buttonArm.setPosition(.7);
            sleep(100);
            robot.buttonArm.setPosition(1);
            encoderDrive(DRIVE_SPEED, 5, 5, 9);
        }
        else if (sensorDataRed > 3){
            robot.buttonArm.setPosition(.7);
            sleep(100);
            robot.buttonArm.setPosition(1);
            sleep(100);
            robot.buttonArm.setPosition(.7);
            sleep(100);
            robot.buttonArm.setPosition(1);
        }
        telemetry.update();
    }

    public void gyroTurn(double turningSpeed, int targetHeading){

        double speed = turningSpeed;

        robot.lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean error = true;

        while(error){

            int gyroZ = robot.GyroSensor.getIntegratedZValue();

            if(targetHeading < gyroZ){
                robot.rfMotor.setPower(-speed);
                robot.lfMotor.setPower(speed);
            } else if(targetHeading > gyroZ) {
                robot.rfMotor.setPower(speed);
                robot.lfMotor.setPower(-speed);
            }else if(targetHeading == gyroZ){
                robot.rfMotor.setPower(0);
                robot.lfMotor.setPower(0);

                robot.lfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                error = false;
            }

        telemetry.addData("gyroZ measure: %7d", gyroZ);
        telemetry.update();
        }

    }
}
