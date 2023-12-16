package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp
public class SlideTest extends OpMode {
    DcMotorEx left, right;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotorEx.class, "Slide Left");
        right = hardwareMap.get(DcMotorEx.class, "Slide Right");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
//        if(gamepad1.a) {
//            left.setPower(1.0);
//            right.setPower(1.0);
//        } else if(gamepad1.b) {
//            left.setPower(-1.0);
//            right.setPower(-1.0);
//        } else {
//            left.setPower(0.0);
//            right.setPower(0.0);
//        }

        left.setPower(gamepad1.left_stick_y);
        right.setPower(gamepad1.left_stick_y);
    }
}
