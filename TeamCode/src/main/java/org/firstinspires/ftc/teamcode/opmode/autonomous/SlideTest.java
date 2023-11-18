package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.core.RobotConstants.DEADZONE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Servo;

@Config
@Autonomous(name = "Slide Test", group = "E")
public class SlideTest extends RobotHardware {
    private final Motor left = RobotConfiguration.SLIDE_LEFT.getAsMotor(),
            right = RobotConfiguration.SLIDE_RIGHT.getAsMotor();
    public static double power = 0.0, holdPower = 0.0, downRetractMultiplier = 0.7;

    public static boolean hold = false, manual = true;


    @Override
    public void loop() {
        super.loop();
        if(power < -0.1)
            power *= downRetractMultiplier;

        if(hold) {
            left.setPower(holdPower);
            right.setPower(holdPower);
        } else {
            if(manual) {
                double x = -primary.left_stick_y;
                if(x < -0.1)
                    x *= downRetractMultiplier;
                left.setPower(x);
                right.setPower(x);
            } else {
                left.setPower(power);
                right.setPower(power);
            }
        }
    }
}