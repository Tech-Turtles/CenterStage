package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LiftTest extends OpMode {
    DcMotorEx lift;

    @Override
    public void init() {
        lift = hardwareMap.get(DcMotorEx.class, "Lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        lift.setPower(-gamepad1.left_stick_y);
    }
}
