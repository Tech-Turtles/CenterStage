package org.firstinspires.ftc.teamcode.utility.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.utility.pathplanner.path.PathPlannerTrajectory;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * Provides Executive robot Control through class based state machine.
 */
public class Executive {

    /**
     * Interface to be used as the interface for the class which hosts the
     * state machine object as well as the concrete implementations of the required states.
     */
    public interface RobotStateMachineContextInterface {
        void init();
        void update();
        String getCurrentState();
    }

    /**
     * Robot state machine, supporting multiple named simultaneous states, expanded by adding to an enum.
     */
    static public class StateMachine <T_opmode extends RobotHardware> {

        private final Map<StateType, StateBase<T_opmode>> stateMap = new HashMap<>();
        private final T_opmode opMode;

        /**
         * State Machine will support one additional concurrent state for each possible StateType.
         * Note that StateMachine.remove(StateType.ARM) would remove an existing ARM state, for
         * example.
         */
        public enum StateType {
            DRIVE,
            INTAKE,
            SLIDES,
            ARM,
            WRIST,
            CLAW,
            CLIMB
        }

        public StateMachine(T_opmode opMode) {
            this.opMode = opMode;
        }

        public void changeState(StateType stateType, StateBase<T_opmode> state) {
            stateMap.put(stateType, state);
            state.init(this);
        }

        public void removeStateByType(StateType stateType) {
            stateMap.remove(stateType);
        }


        public void clearDeletedStates() {
            for (StateType stateType : StateType.values()) {
                if(stateMap.get(stateType) != null) {
                    if (stateMap.get(stateType).isDeleteRequested()) {
                        stateMap.remove(stateType);
                    }
                }
            }
        }

        /**
         * Allows a StateBase to access a reference to another running state, primarily for
         * reading the 'isDone' property.
         * @param stateType The StateType of the state you want a reference to
         * @return A reference to the state
         */
        public StateBase<T_opmode> getStateReferenceByType(StateType stateType) {
            return stateMap.get(stateType);
        }

        public Class<?> getCurrentStateByType(StateType stateType) {
            StateBase<T_opmode> state = stateMap.get(stateType);
            try {
                return state.getClass();
            } catch (NullPointerException ignore){
                // This method should only be used for logging or to check the current state.
                // Because of that, it does not matter what class is returned.
                return this.getClass();
            }
        }

        /**
         * A method supplies
         * @return a String containing all of the StateTypes and the States associated with them
         */
        public String getCurrentStateByType() {
            StringBuilder stateString = new StringBuilder();
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                String stateElement = getCurrentStateByType(type).getSimpleName();
                stateString.append(stateElement).append("\n");
            }
            return stateString.toString();
        }

        public void init() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase<T_opmode> state = stateMap.get(type);
                if(state != null) {
                    state.init(this);
                }
            }
        }

        /**
         * Removes states if a delete has been requested.
         * Runs the update function for any states that are not null or isDeleteRequested and has been initialized
         */
        public void update() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase<T_opmode> state = stateMap.get(type);

                if(state == null) {
                    Log.wtf(this.getClass().getSimpleName(), "State in the stateMap is null");
                    continue;
                }

                if (!state.isInitialized()) {
                    Log.wtf(this.getClass().getSimpleName(), state.getClass().getSimpleName() + " State has not been initialized");
                    continue;
                }

                if (!state.isDeleteRequested())
                    state.update();
            }
            clearDeletedStates();
        }

        /**
         * Changes 'initialized' to false, preventing it from being updated until
         */
        public void reset() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase<T_opmode> state = stateMap.get(type);
                state.reset();
            }
        }
    }

    /**
     * Base class for all states, allowing for optional implementation of its methods
     * @param <T_opmode> Reference to the OpMode within the state. The generic allows it to accept
     *                  any instance of RobotHardware, allowing the methods associated with that
     *                  specific opmode, rather than being limited to only RobotHardware.
     */
    public static abstract class StateBase<T_opmode extends RobotHardware> {

        StateMachine<T_opmode> stateMachine;    // Reference to StateMachine, for modifying states.
        protected T_opmode opMode;              // OpMode reference
        protected ElapsedTime stateTimer;       // Amount of time the state has been active
        protected ElapsedTime statePeriod;      // Amount of time since the state has been executed.
        ElapsedTime timer;
        double lastStatePeriod = -1;
        public boolean isDone = false;

        private boolean initialized = false;
        private boolean deleteRequested = false;
        protected PathPlannerTrajectory trajectory = null;

        public void init(StateMachine<T_opmode> stateMachine) {
            this.stateMachine = stateMachine;
            this.opMode = stateMachine.opMode;
            stateTimer = new ElapsedTime();
            statePeriod = new ElapsedTime();
            timer = new ElapsedTime();
            isDone = false;
            initialized = true;
            timer.reset();
            stateTimer.reset();
            statePeriod.reset();
        }

        public double getTime() {
            return stateTimer.seconds();
        }

        public PathPlannerTrajectory getTrajectory() {
            return trajectory;
        }

        public void update() {
            lastStatePeriod = statePeriod.seconds();
            statePeriod.reset();
        }

        protected void nextState(StateMachine.StateType stateType, StateBase<T_opmode> state) {
            stateMachine.changeState(stateType, state);
        }

        public void reset() {
            initialized = false;
        }

        boolean isInitialized() {
            return initialized;
        }

        public void requestDelete() {
            deleteRequested = true;
        }

        public boolean isDeleteRequested() {
            return deleteRequested;
        }
    }
}
