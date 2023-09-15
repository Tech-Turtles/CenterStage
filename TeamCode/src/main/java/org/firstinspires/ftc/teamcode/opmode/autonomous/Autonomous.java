package org.firstinspires.ftc.teamcode.opmode.autonomous;

import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.autonomous.StartPosition;

;


public class Autonomous extends RobotHardware {

    public static AllianceColor robotColor = AllianceColor.RED;
    public static StartPosition robotStartPos = StartPosition.AUDIENCE;
    private Executive.RobotStateMachineContextInterface robotStateContext;
    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Left", group="A")
    public static class AutoRedAudience extends Autonomous {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.AUDIENCE;
            super.init();
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Right", group="A")
    public static class AutoRedFar extends Autonomous {
        @Override public void init() {
            robotColor = AllianceColor.RED;
            robotStartPos = StartPosition.FAR;
            super.init();
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Left", group="B")
    public static class AutoBlueFar extends Autonomous {
        @Override public void init() {
            robotColor = AllianceColor.BLUE;
            robotStartPos = StartPosition.FAR;
            super.init();
        }
    }

    @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Right", group="B")
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
        robotStateContext = new RobotStateContext(this, robotColor, robotStartPos);
        robotStateContext.init();
    }

    @Override
    public void loop() {
        super.loop();
        robotStateContext.update();
        telemetry.addData("State ", robotStateContext.getCurrentState());
        if(packet != null) {
            packet.put("State: ", robotStateContext.getCurrentState());
        }
    }
}