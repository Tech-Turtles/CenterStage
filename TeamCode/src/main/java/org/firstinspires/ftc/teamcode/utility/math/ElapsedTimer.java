package org.firstinspires.ftc.teamcode.utility.math;

import static org.firstinspires.ftc.teamcode.utility.math.ListMath.average;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ElapsedTimer extends ElapsedTime {

    private final ElapsedTime period = new ElapsedTime();
    private final List<Double> pastPeriods = new ArrayList<>();
    private final double historyLength;

    public ElapsedTimer() {
        this(200);
    }

    public ElapsedTimer(double historyLength) {
        this.historyLength = historyLength;
    }

    public void updatePeriodTime(){
        pastPeriods.add(period.seconds());
        period.reset();
        if (pastPeriods.size()>= historyLength) {
            pastPeriods.remove(0);
        }
        average(pastPeriods);
    }

    public double getAveragePeriodSec() {
        return average(pastPeriods);
    }

    public double getMaxPeriodSec() {
        return Collections.max(pastPeriods);
    }

    public double getLastPeriodSec() {
        if (pastPeriods.size() == 0)
            return 0;
        return pastPeriods.get(pastPeriods.size()-1);
    }

    public List<Double> getPastPeriods() {
        return pastPeriods;
    }

    public void clearPastPeriods() {
        pastPeriods.clear();
    }

    public double getHistoryLength() {
        return historyLength;
    }
}