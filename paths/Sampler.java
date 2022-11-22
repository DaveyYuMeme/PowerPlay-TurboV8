package org.firstinspires.ftc.teamcode.paths;

import org.firstinspires.ftc.teamcode.Vec;
import org.firstinspires.ftc.teamcode.utils.SumUtil;

import org.firstinspires.ftc.teamcode.paths.components.*;

import java.lang.Exception;

public class Sampler {
  private Component[] valueComponents;
  private Component[] timingComponents;
  private int dim;
  private int valueCount;
  private int timingCount;
  private SumUtil valueLengthSumUtil;
  private SumUtil timingLengthSumUtil;

  public Sampler(Component[] valueComponents, Component[] timingComponents) throws Vec.DimMismatchException, Sampler.CountMismatchException {
    int valueCount = 0;
    for (int i = 0; i < valueComponents.length; i++) valueCount += valueComponents[i].getCount();
    int timingCount = 0;
    for (int i = 0; i < timingComponents.length; i++) timingCount += timingComponents[i].getCount();
    int valueDim = valueComponents[0].getDim();
    for (int i = 1; i < valueComponents.length; i++) if (valueComponents[i].getDim() != valueDim) throw new Vec.DimMismatchException();
    int timingDim = timingComponents[0].getDim();
    for (int i = 1; i < timingComponents.length; i++) if (timingComponents[i].getDim() != timingDim) throw new Vec.DimMismatchException();
    if (valueCount != timingCount) throw new Sampler.CountMismatchException();
    if (timingDim != 1) throw new Vec.DimMismatchException("Timing component dimension mismatch");
    this.valueComponents = valueComponents;
    this.timingComponents = timingComponents;
    this.dim = valueDim;
    this.valueCount = valueCount;
    this.timingCount = timingCount;
    this.buildValueLengthSumUtil();
    this.buildTimingLengthSumUtil();
  }

  public static class CountMismatchException extends Exception {
    public CountMismatchException(String error) {
      super(error);
    }

    public CountMismatchException() {
      super("Component count mismatch.");
    }
  }

  private void buildValueLengthSumUtil() {
    double[] lengths = new double[this.valueCount];
    for (int i = 0; i < this.valueCount; i++) lengths[i] = this.valueComponents[i].getLength();
    this.valueLengthSumUtil = new SumUtil(lengths);
  }

  private void buildTimingLengthSumUtil() {
    double[] lengths = new double[this.timingCount];
    for (int i = 0; i < this.timingCount; i++) lengths[i] = this.timingComponents[i].getLength();
    this.timingLengthSumUtil = new SumUtil(lengths);
  }

  public int getDim() {
    return this.dim;
  }

  public int getValueCount() {
    return this.valueCount;
  }

  public int getTimingCount() {
    return this.timingCount;
  }

  public double getValueLength(int l, int r) {
    return this.valueLengthSumUtil.rangeSum(l, r);
  }

  public double getTimingLength(int l, int r) {
    return this.timingLengthSumUtil.rangeSum(l, r);
  }

  public Vec getValue(double t) throws Vec.DimMismatchException {
    double s = this.getTime(t) * this.getValueLength(0, this.valueCount);
    int i = this.valueLengthSumUtil.lowerBound(s);
    double f = (s - this.getValueLength(0, i)) / this.getValueLength(i, i + 1);
    return this.valueComponents[i].getPosition(f);
  }

  public double getTime(double t) throws Vec.DimMismatchException {
    double s = t * this.getTimingLength(0, this.timingCount);
    int i = this.timingLengthSumUtil.lowerBound(s);
    double f = (s - this.getTimingLength(0, i)) / this.getTimingLength(i, i + 1);
    return this.timingComponents[i].getPosition(f).dims[0];
  }
}