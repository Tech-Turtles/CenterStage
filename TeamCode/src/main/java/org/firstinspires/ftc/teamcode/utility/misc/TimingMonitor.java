package org.firstinspires.ftc.teamcode.utility.misc;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.RobotHardware;

import java.util.ArrayList;

/**
 *  Timing Monitor is a debug tool for discovering the source of slowdowns in code.
 *  This is done by keeping track of the LONGEST time interval observed between labeled checkpoints.
 *
 *  To use, first create a TimingMonitor object with a reference to the opMode
 *  e.g. TimingMonitor timingMonitor = new TimingMonitor(this)
 *  Then place the timingMonitor.loopStart() call at the beginning of the execution loop.
 *  It is important that this runs before any checkpoints.
 *  Finally, place timingMonitor.checkpoint('label_1') calls after segments you wish timed.
 *  To view the results, run timingMonitor.displayMaxTimes()
 *  The time associated with the START label is the longest time it took to get from the
 *  last checkpoint to the starting point.
 *  Place checkpoints after the code blocks that their label refers to.
 */
public class TimingMonitor {
    private final RobotHardware opMode; // Reference to opmode
    private final ElapsedTime timer;
    // All checkpoints hit once, names established and arrays populated.
    private boolean firstLoopCompleted;
    private ArrayList<String> checkpointNames;
    private ArrayList<Double> checkpointMaxTimes;
    private int checkpointIndex;
    private int totalCheckpoints;
    private boolean checkpointOrderError;
    private final int MAX_CHECKPOINTS = 100;
    private boolean enabled = true;
    private final StringBuilder builder = new StringBuilder();

    public TimingMonitor(RobotHardware opMode) {
        this.opMode = opMode;
        timer = new ElapsedTime();
        this.reset();
    }

    public void reset() {
        enabled = true;
        firstLoopCompleted = false; // Required to begin timing
        checkpointIndex = 0;
        totalCheckpoints = 0;
        checkpointOrderError = false;
        checkpointNames = new ArrayList<>();
        checkpointMaxTimes = new ArrayList<>();
        builder.setLength(0);
    }

    //Disabling the timing monitor will remove the reports and minimize the processing footprint.
    public void disable() {
        enabled = false;
    }

    public void enable() {
        enabled = true;
        this.reset();
    }

    public void loopStart() {
        if(!enabled) return; // Exit function in not enabled.

        double previousTime;
        double currentTime;
        // Determine if the first loop has been completed
        if (!firstLoopCompleted){ // Initial loop underway.
            if(checkpointIndex > 0) {
                // We've already completed the first loop, so get to work.
                firstLoopCompleted = true;
                totalCheckpoints = checkpointNames.size();
            } else {
                // This is the first time the loop start has been run.
                //checkpointIndex <= 0
                checkpointIndex = 0;
                checkpointNames.add("START");
                timer.reset();
                checkpointMaxTimes.add(0.0); // Initialize blank on first loop.

            }
        }

        if (firstLoopCompleted){
            /* Loop initialized and surveyed once.
               Begin normal operation.
               Looking for longest delay
            */
            checkpointIndex = 0; // Only place the checkpoint index is reset.
            currentTime = timer.seconds();
            timer.reset();
            previousTime = checkpointMaxTimes.get(checkpointIndex);
//            if(currentTime > previousTime) {
                // Store new time if it is longer.
                checkpointMaxTimes.set(checkpointIndex,currentTime);
//            }
        }
        // Increment the checkpoint index
        checkpointIndex++;
    }

    public void checkpoint(String checkpointName){
        if(!enabled) return; // Exit function in not enabled.

        double currentTime;
        double previousTime;
        if(checkpointIndex > 0 && checkpointIndex <= MAX_CHECKPOINTS) {
            /* If checkpointIndex wasn't incremented >0, then we haven't run loopStart().
              If loopStart() is NEVER called, then checkpoint will cause the arrays to grow
              without bound, so a limit was added
            */
            if (firstLoopCompleted) {
                currentTime = timer.seconds();
                timer.reset();
                previousTime = checkpointMaxTimes.get(checkpointIndex);
//                if (currentTime > previousTime) {
                    // Store new time if it is longer.
                    checkpointMaxTimes.set(checkpointIndex, currentTime);
//                }

                //Error check the name index
                if (!checkpointName.equals(checkpointNames.get(checkpointIndex))) {
                    // If we miss a checkpoint, our times will be out of order and invalid.
                    checkpointOrderError = true;
                }
            } else {
                /* firstLoopCompleted = false
                   Add checkpoint name to list
                */
                checkpointNames.add(checkpointName);
                // Add checkpoint time to the list.
                timer.reset();
                checkpointMaxTimes.add(0.0); // Initialize blank
            }
            // Increment the checkpoint index
            checkpointIndex++;
        } else { // If checkpointIndex wasn't incremented, then we haven't run loopStart().
            checkpointOrderError = true;
        }
    }

    @SuppressLint("DefaultLocale")
    public void displayMaxTimes() {
        builder.setLength(0);
        if(!enabled) {
            opMode.telemetry.addLine("TimingMonitor Disabled");
            return; // Exit function in not enabled.
        }

        builder.append("TimingMonitor Max Time Results").append("\n");
        if(checkpointOrderError) {
            builder.append("ERROR: Checkpoint order inconsistent.");
            builder.append("ERROR: Make sure loopStart() runs before checkpoint(), and that no calls are conditional.");
        }
        if(firstLoopCompleted) {
            int index = 0;
            for (String checkpoint : checkpointNames) {
                builder.append(checkpoint).append(": ").append(String.format("%.3f", checkpointMaxTimes.get(index))).append("\n");
                index++;
            }
        }
        opMode.telemetry.addLine(builder.toString());
    }
}
