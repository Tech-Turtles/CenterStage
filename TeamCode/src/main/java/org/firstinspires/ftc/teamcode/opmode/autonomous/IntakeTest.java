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
@Autonomous(name = "Intake Test", group = "E")
public class IntakeTest extends RobotHardware {
    private final Servo ramp = RobotConfiguration.RAMP.getAsServo();
    private final Motor intake = RobotConfiguration.INTAKE.getAsMotor();

    public static double intakeSpeed = 1.0;
    public static double outtakeSpeed = -0.5;

    public static double r1 = 0.0, r2 = 0.2, r3 = 0.63, r4 = 0.55;
    public static int state = 1;
    public static boolean rampEnabled = false;

    @Override
    public void loop() {
        super.loop();

        if(primary.right_trigger > DEADZONE) {
            intake.setPower(intakeSpeed);
        } else if(primary.left_trigger > DEADZONE) {
            intake.setPower(outtakeSpeed);
        } else {
            intake.setPower(0.0);
        }

        if(rampEnabled) {
            switch (state) {
                case 1:
                    ramp.setPosition(r1);
                    break;
                case 2:
                    ramp.setPosition(r2);
                    break;
                case 3:
                    ramp.setPosition(r3);
                    break;
                default:
                    ramp.setPosition(r4);
                    break;
            }
        }
    }
}