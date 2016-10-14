package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by joeyun on 9/28/16.
 */

public class Motor extends OpMode{
    DcMotor leftMotor;
    DcMotor rightMotor;
    @Override
    public void init() {
leftMotor= hardwareMap.dcMotor.get("left_drive");
        rightMotor=hardwareMap.dcMotor.get("right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        float yValue= -gamepad1.left_stick_y;
        float xValue= -gamepad1.right_stick_x;

        float leftPower= yValue + xValue;
        float rightPower= yValue - xValue;

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        leftPower= Range.clip(leftPower, -1, 1);
        rightPower=Range.clip(rightPower, -1, 1);

    }
}
