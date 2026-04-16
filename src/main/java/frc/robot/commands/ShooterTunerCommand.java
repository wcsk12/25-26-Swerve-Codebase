package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Runs a short open-loop measurement to compute velocity feedforward, applies a
 * conservative closed-loop config, then performs a step test and reports rise time
 * and steady-state error.
 *
 * This command is intended for bench tuning only. It prints results to stdout.
 */
public class ShooterTunerCommand extends Command {
  private final ShooterSubsystem shooter;
  private final double openLoopOutput; // 0..1
  private final double openLoopDurationSecs;
  private final double targetRPM;
  private final double closedLoopTestDurationSecs;

  private int state = 0;
  private double stateStartTime = 0;
  private final List<Double> samples = new ArrayList<>();
  private final List<Double> sampleTimes = new ArrayList<>();
  // Sweep helpers
  private List<Double> pCandidates = new ArrayList<>();
  private int sweepIndex = 0;
  private static class SweepResult {
    double p;
    double t10;
    double t90;
    double rise;
    double steady;
    double sse;
  }
  private final List<SweepResult> results = new ArrayList<>();
  private final double desiredRiseSecs = 2.0; // target rise time we want to approach
  private double thisCalcedFF = 0.0;

  public ShooterTunerCommand(ShooterSubsystem shooter, double openLoopOutput, double openLoopDurationSecs, double targetRPM, double closedLoopTestDurationSecs) {
    this.shooter = shooter;
    this.openLoopOutput = openLoopOutput;
    this.openLoopDurationSecs = openLoopDurationSecs;
    this.targetRPM = targetRPM;
    this.closedLoopTestDurationSecs = closedLoopTestDurationSecs;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    System.out.println("[ShooterTuner] init");
    state = 1; // start open-loop sampling
    stateStartTime = Timer.getFPGATimestamp();
    samples.clear();
    sampleTimes.clear();
    shooter.setShooterSpeed(openLoopOutput);
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double elapsed = now - stateStartTime;
    double rpm = shooter.getMeasuredRPM();

    if (state == 1) {
      // open-loop sampling
      samples.add(rpm);
      sampleTimes.add(now);
      if (elapsed >= openLoopDurationSecs) {
        // compute average of last ~0.5s (or as many samples as available)
        int lastCount = Math.min(samples.size(), Math.max(1, (int)(0.5 / 0.02)) );
        double sum = 0;
        for (int i = samples.size() - lastCount; i < samples.size(); i++) {
          sum += samples.get(i);
        }
        double avgRPM = sum / lastCount;
        if (avgRPM < 50) {
          System.out.println("[ShooterTuner] measured RPM too small (" + avgRPM + ") — aborting tuner");
          state = 99;
          return;
        }
        double ff = openLoopOutput / avgRPM;
        System.out.println(String.format("[ShooterTuner] open-loop avgRPM=%.1f at output=%.3f -> ff=%.8f", avgRPM, openLoopOutput, ff));

        // prepare sweep candidates around a conservative starting P
        double startP = Math.max(ff * 3.0, 1e-6);
        double[] mults = new double[] {0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0};
        pCandidates.clear();
        for (double m : mults) {
          double p = startP * m;
          if (p > 0 && !pCandidates.contains(p)) pCandidates.add(p);
        }
        System.out.println(String.format("[ShooterTuner] FF=%.8f startP=%.8f candidates=%s", ff, startP, pCandidates.toString()));

        // initialize sweep state
        sweepIndex = 0;
        results.clear();
        // move to sweep state
        state = 10;
        stateStartTime = 0;
        // store ff for reuse by sweep
        thisCalcedFF = ff;
      }
    } else if (state == 10) {
      // start next candidate in sweep
      if (sweepIndex >= pCandidates.size()) {
        // sweep finished: pick best candidate
        SweepResult best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        for (SweepResult r : results) {
          if (Double.isNaN(r.rise)) continue;
          double score = Math.abs(r.rise - desiredRiseSecs);
          // penalize steady-state overshoot > +5% by adding a large penalty
          if (r.steady > 1.05 * targetRPM) score += 10.0;
          if (score < bestScore) {
            bestScore = score;
            best = r;
          }
        }
        if (best != null) {
          System.out.println(String.format("[ShooterTuner] best P=%.8f rise=%.3f steady=%.1f (software PID — not applying to SparkMax)", best.p, best.rise, best.steady));
          // Persist chosen gains to disk on the roboRIO so they survive restarts.
          try {
            java.util.Properties props = new java.util.Properties();
            props.setProperty("p", Double.toString(best.p));
            props.setProperty("i", Double.toString(0.0));
            props.setProperty("d", Double.toString(0.0));
            props.setProperty("ff", Double.toString(thisCalcedFF));
            java.io.File out = new java.io.File("/home/lvuser/shooter_gains.properties");
            try (java.io.FileOutputStream fos = new java.io.FileOutputStream(out)) {
              props.store(fos, "Shooter closed-loop gains (written by ShooterTuner)");
            }
            System.out.println("[ShooterTuner] persisted gains to /home/lvuser/shooter_gains.properties");
          } catch (Exception e) {
            System.out.println("[ShooterTuner] failed to persist gains: " + e);
          }
        } else {
          System.out.println("[ShooterTuner] no valid sweep result found; keeping last config");
        }
        state = 3; // finished
        return;
      }
      double p = pCandidates.get(sweepIndex);
      // apply gains and start sampling
      // Software PID — gains are constants, no SparkMax apply needed
      System.out.println(String.format("[ShooterTuner] testing P=%.8f (info only)", p));
      samples.clear();
      sampleTimes.clear();
      stateStartTime = Timer.getFPGATimestamp();
      // command closed-loop target
      shooter.setClosedLoopTargetRPM(targetRPM);
      state = 11;
    } else if (state == 11) {
      // sampling for current sweep candidate
      samples.add(rpm);
      sampleTimes.add(now);
      if (elapsed >= closedLoopTestDurationSecs) {
        // compute t10 and t90
        double t10 = Double.NaN;
        double t90 = Double.NaN;
        double t0 = stateStartTime;
        for (int i = 0; i < samples.size(); i++) {
          double r = samples.get(i);
          double t = sampleTimes.get(i);
          if (Double.isNaN(t10) && r >= 0.1 * targetRPM) {
            t10 = t - t0;
          }
          if (Double.isNaN(t90) && r >= 0.9 * targetRPM) {
            t90 = t - t0;
          }
        }
        double rise = Double.NaN;
        if (!Double.isNaN(t10) && !Double.isNaN(t90)) {
          rise = t90 - t10;
        }
        int lastCount = Math.min(samples.size(), Math.max(1, (int)(0.5 / 0.02)) );
        double sum = 0;
        for (int i = samples.size() - lastCount; i < samples.size(); i++) sum += samples.get(i);
        double steady = sum / lastCount;
        double sse = steady - targetRPM;
        SweepResult sr = new SweepResult();
        sr.p = pCandidates.get(sweepIndex);
        sr.t10 = t10;
        sr.t90 = t90;
        sr.rise = rise;
        sr.steady = steady;
        sr.sse = sse;
        results.add(sr);
        System.out.println(String.format("[ShooterTuner] P=%.8f -> rise=%.3f steady=%.1f sse=%.1f", sr.p, sr.rise, sr.steady, sr.sse));
        sweepIndex++;
        state = 10; // go to next candidate
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooterSpeed();
    System.out.println("[ShooterTuner] end interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    return state == 3 || state == 99;
  }
}
