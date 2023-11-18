package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Controller;

@TeleOp(name="Tank Drive", group="E")
@Disabled
public class Tank extends OpMode {

    private DcMotor fLeft, bLeft, fRight, bRight;
    private Controller primary;

    @Override
    public void init() {
        fLeft = hardwareMap.get(DcMotor.class, "front left");
        bLeft = hardwareMap.get(DcMotor.class, "back left");
        fRight = hardwareMap.get(DcMotor.class, "front right");
        bRight = hardwareMap.get(DcMotor.class, "back right");

        primary = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        double leftPower  = Range.clip(-primary.left_stick_y + primary.left_stick_x, -1.0, 1.0) ;
        double rightPower = Range.clip(-primary.right_stick_y + primary.right_stick_x, -1.0, 1.0) ;

        fLeft.setPower(leftPower);
        fRight.setPower(rightPower);
        bLeft.setPower(leftPower);
        bRight.setPower(rightPower);
    }
}