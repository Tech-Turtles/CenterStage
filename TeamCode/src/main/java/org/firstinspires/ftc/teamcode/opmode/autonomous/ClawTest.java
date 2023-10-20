package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Servo;

@Config
@Autonomous(name = "Claw Test", group = "E")
public class ClawTest extends RobotHardware {
       private final Servo
               left = RobotConfiguration.CLAW_LEFT.getAsServo(),
               right = RobotConfiguration.CLAW_RIGHT.getAsServo();

       public static double l1 = 0.5, l2 = 0.5, r1 = 0.5, r2 = 0.5;
       public static int state = 1;
       public static boolean leftEnabled = true, rightEnabled = true;

    @Override
    public void loop() {
        super.loop();
        if(leftEnabled) {
            left.setPosition(state == 1 ? l1 : l2);
        }

        if(rightEnabled) {
            right.setPosition(state == 1 ? r1 : r2);
        }
    }
}