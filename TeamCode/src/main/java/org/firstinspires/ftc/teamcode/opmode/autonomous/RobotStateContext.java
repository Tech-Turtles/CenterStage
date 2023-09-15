package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.utility.autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.utility.autonomous.Executive;
import org.firstinspires.ftc.teamcode.utility.autonomous.StartPosition;

@Config
public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    // Required class variables
    private final Autonomous autonomous;
    private final Executive.StateMachine<Autonomous> stateMachine;
    private final AllianceColor allianceColor;
    private final StartPosition startPosition;
    public RobotStateContext(Autonomous autonomous, AllianceColor allianceColor, StartPosition startPosition) {
        this.autonomous = autonomous;
        this.allianceColor = allianceColor;
        this.startPosition = startPosition;
        this.stateMachine = new Executive.StateMachine<>(autonomous);
        stateMachine.update();
    }

    @Override
    public void init() {
        stateMachine.init();
    }

    @Override
    public void update() {
        stateMachine.update();
    }

    @Override
    public String getCurrentState() {
        return stateMachine.getCurrentStateByType();
    }

    /**
     * Start State
     * State that sets the robot's position to the start position.
     * Changes the routine based on start position.
     *
     * Trajectory: none
     * Next State: 
     */
    class Start extends Executive.StateBase<Autonomous> {
        @Override
        public void init(Executive.StateMachine<Autonomous> stateMachine) {
            super.init(stateMachine);

            switch (startPosition) {
                case AUDIENCE:
                    break;
                case FAR:
                    break;
                default:
                   throw new IllegalArgumentException("Invalid start position");
            }
        }
    }
}