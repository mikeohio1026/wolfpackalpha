package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
public class Hardware {
    /*Public OpMode Members*/
    /*Motors attached here.*/
    public DcMotor lfMotor = null;
    public DcMotor rfMotor = null;

    //public DcMotor flyWheelLeft = null;
    //public DcMotor flyWheelRight = null;
    //public DcMotor pulley = null;
    //public DcMotor linearSlide = null;

    /*For initializing Servos (not attached)*/
    public Servo buttonMasher = null;
    public Servo buttonArm = null;

    //For Initalizing Sensors
    public ColorSensor ColorSensorA = null;
    public ModernRoboticsI2cGyro GyroSensor = null;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware() {

    }




    /* Initialize standard Hardware interfaces */

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /*Define and Initialize Motors (Assign variable (usage) name to the physical Motor)
        * Remember that the string "name" has to be the same as the one on the configuration
        */
        lfMotor = hwMap.dcMotor.get("lf_motor");
        rfMotor = hwMap.dcMotor.get("rf_motor");

        //flyWheelLeft = hwMap.dcMotor.get("flyWheelLeft");
        //flyWheelRight = hwMap.dcMotor.get("flyWheelRight");
        //pulley = hwMap.dcMotor.get("pulley");
        //linearSlide = hwMap.dcMotor.get("linearSlide");


        lfMotor.setDirection(DcMotor.Direction.REVERSE); //AndyMark Settings
        rfMotor.setDirection(DcMotor.Direction.FORWARD); //AndyMark Settings

        //pulley direction is correct, not the other 3 additional motors
        //flyWheelRight.setDirection(DcMotor.Direction.FORWARD);
        //flyWheelLeft.setDirection(DcMotor.Direction.FORWARD);
        //pulley.setDirection(DcMotor.Direction.REVERSE);
        //linearSlide.setDirection(DcMotor.Direction.FORWARD);


        /* Set all motors to zero power*/
        rfMotor.setPower(0);
        lfMotor.setPower(0);

        //flyWheelRight.setPower(0);
        //flyWheelLeft.setPower(0);
        //pulley.setPower(0);
        //linearSlide.setPower(0);

        /* Set all motors to run without encoders.
         May want to use RUN_USING_ENCODERS if encoders are installed*/
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //flyWheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //flyWheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //pulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /* Define and initialize ALL installed servos.*/
        buttonMasher = hwMap.servo.get("buttonMasher");
        buttonMasher.setPosition(0);
        buttonArm = hwMap.servo.get("buttonArm");
        buttonArm.setPosition(1);

        /* Define and initialize ALL installed sensors.*/
        ColorSensorA = hwMap.colorSensor.get("Color_A");

        GyroSensor = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");


//naming convention
        //gyro sensor = gyro
        //buttonArm mechanism = buttonArm
        //button Pusher thing = buttonMasher
        //color sensor A = Color_A
        //motor left front motor = lf_motor
        //motor right front motor = rf_motor
        //motor pulley = pulley
        //motor linear slide = linearSlide
        //motor flywheels = flyWheelRight and flyWheelLeft

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

