package frc.util;

import java.util.Random;
import edu.wpi.first.wpilibj.Timer;

public class RandomExecutionLimiter {
    private int executionCounter = 0;
    private int executionLimit = 0;
    private Random random = new Random();
    private Timer timer = new Timer();
    private int minTimesPerSecond;
    private int maxTimesPerSecond;

    public RandomExecutionLimiter(int minTimesPerSecond, int maxTimesPerSecond) {
        this.minTimesPerSecond = minTimesPerSecond;
        this.maxTimesPerSecond = maxTimesPerSecond;
        resetExecutionLimit();
        timer.start();
    }

    public RandomExecutionLimiter() {
        this(3, 7);
    }

    public boolean shouldExecute() {
        executionCounter++;
        if (executionCounter >= executionLimit || timer.hasElapsed(1.0)) {
            executionCounter = 0;
            resetExecutionLimit();
            timer.reset();
            return true;
        }
        return false;
    }

    private void resetExecutionLimit() {
        executionLimit = 50 / (minTimesPerSecond + random.nextInt(maxTimesPerSecond - minTimesPerSecond + 1));
    }
}