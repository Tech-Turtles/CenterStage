package org.firstinspires.ftc.teamcode.opmode.autonomous;

import static org.firstinspires.ftc.teamcode.core.AutonomousStateContext.leftBound;
import static org.firstinspires.ftc.teamcode.core.AutonomousStateContext.rightBound;
import static org.firstinspires.ftc.teamcode.core.AutonomousStateContext.spikePosition;
import static org.firstinspires.ftc.teamcode.core.RobotConstants.WRIST_CENTER;

import org.firstinspires.ftc.teamcode.core.AutonomousStateContext;
import org.firstinspires.ftc.teamcode.core.RobotConfiguration;
import org.firstinspires.ftc.teamcode.core.RobotConstants;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.utility.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.autonomous.SpikePosition;
import org.firstinspires.ftc.teamcode.utility.autonomous.StartPosition;
import org.opencv.core.Rect;

/**
 * Base autonomous class that holds the autonomous OpModes for each starting location.
 * Responsible for initializing the state-machine with the correct starting location & color.
 * Updates the state-machine and outputs the current state for each
 * {@link org.firstinspires.ftc.teamcode.utility.autonomous.Executive.StateMachine.StateType}
 */
public class Autonomous extends RobotHardware {

    public static AllianceColor robotColor = AllianceColor.RED;
    public static StartPosition robotStartPos = StartPosition.AUDIENCE;
    private Executive.RobotStateMachineContextInterface robotStateContext;
    private Webcam webcam = RobotConfiguration.WEBCAM.getAsWebcam();

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Test", group="A")
    public static class Test extends Autonomous {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.BACK_BOARD;
            super.init();
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Left", group="B")
    public static class AutoRedAudience extends Autonomous {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.AUDIENCE;
            super.init();
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Right", group="B")
    public static class AutoRedBackBoard extends Autonomous {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.BACK_BOARD;
            super.init();
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Left", group="C")
    public static class AutoBlueBackBoard extends Autonomous {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.BACK_BOARD;
            super.init();
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Right", group="C")
    public static class AutoBlueAudience extends Autonomous {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.AUDIENCE;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();
        robotStateContext = new AutonomousStateContext(this, robotColor, robotStartPos);
        robotStateContext.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        RobotConfiguration.RAMP.getAsServo().setPosition(RobotConstants.IntakePosition.START.getPosition());
        RobotConfiguration.ARM_LEFT.getAsServo().setPosition(RobotConstants.ArmPosition.HOLD.getLeftPos());
        RobotConfiguration.ARM_RIGHT.getAsServo().setPosition(RobotConstants.ArmPosition.HOLD.getRightPos());
        RobotConfiguration.WRIST.getAsServo().setPosition(WRIST_CENTER);
        RobotConfiguration.CLAW_LEFT.getAsServo().setPosition(RobotConstants.ClawPosition.GRAB.getLeftPos());
        RobotConfiguration.CLAW_RIGHT.getAsServo().setPosition(RobotConstants.ClawPosition.GRAB.getRightPos());
        try {
            Rect rect = robotColor.equals(AllianceColor.BLUE)
                    ? webcam.getProcessor().getBlueRect()
                    : webcam.getProcessor().getRedRect();

            try {
                double x = rect.x + rect.width / 2.0;
                if (x < leftBound)
                    spikePosition = SpikePosition.LEFT;
                else if (x > rightBound)
                    spikePosition = SpikePosition.RIGHT;
                else
                    spikePosition = SpikePosition.MIDDLE;
            } catch (Exception ignore) {
            }

            telemetry.addData("Spike", spikePosition.name());
        }catch (Exception ignore) {}

    }

    @Override
    public void loop() {
        super.loop();
        robotStateContext.update();
        telemetry.addData("Pose", swerveDrive.getPose());
        telemetry.addData("State ", robotStateContext.getCurrentState());
        if(packet != null) {
            packet.put("State: ", robotStateContext.getCurrentState());
        }
    }
}