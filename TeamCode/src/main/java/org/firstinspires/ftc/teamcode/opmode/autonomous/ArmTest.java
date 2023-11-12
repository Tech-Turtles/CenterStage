package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Servo;

@Config
@Autonomous(name = "Arm Test", group = "E")
public class ArmTest extends RobotHardware {
    private final Servo
            left = RobotConfiguration.ARM_LEFT.getAsServo(),
            right = RobotConfiguration.ARM_RIGHT.getAsServo();

    public static double l1 = 0.0, l2 = 0.5, l3 = 0.85, r1 = 0.0, r2 = 0.5, r3 = 0.85;
    public static int state = 1;
    public static boolean enabled = false;

    @Override
    public void init() {
        super.init();

//        MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
//                new MotionState(0, 0, 0),
//                new MotionState(20, 0, 0),
//                5,
//                10,
//                30
//        );

//        MotionState state = profile.get(elapsedTime);
//
//        controller.setTargetPosition(state.x);
//        controller.setTargetVelocity(state.v);
//        controller.setTargetAcceleration(state.a);
//
//        double correction = controller.update(measuredPosition);
    }

    @Override
    public void loop() {
        super.loop();

        if(enabled) {
            switch (state) {
                case 1:
                    left.setPosition(l1);
                    right.setPosition(r1);
                    break;
                case 2:
                    left.setPosition(l2);
                    right.setPosition(r2);
                    break;
                default:
                    left.setPosition(l3);
                    right.setPosition(r3);
                    break;
            }
        }
    }
}