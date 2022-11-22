package org.firstinspires.ftc.teamcode.paths.components;

import org.firstinspires.ftc.teamcode.Vec;
import org.firstinspires.ftc.teamcode.utils.SumUtil;

public class CardinalSplineComponent implements Component {
  private Vec[] points;
  private int dim;
  private int count;
  private SumUtil lengthSumUtil;

  public CardinalSplineComponent(Vec[] points) throws Vec.DimMismatchException {
    Vec.checkDims(points);
    this.points = points;
    this.dim = dim;
    this.count = points.length - 1;
    this.buildLengthSumUtil();
  }

  private static class CardinalCurve {
    public static final double alpha = 0.5;

    private static void getCoefficients(Vec v0, Vec v1, Vec v2, Vec v3, double u, Vec a, Vec b, Vec c, Vec d) throws Vec.DimMismatchException {
      double t01 = Math.pow(Vec.distance(v0, v1), CardinalCurve.alpha);
      double t12 = Math.pow(Vec.distance(v1, v2), CardinalCurve.alpha);
      double t23 = Math.pow(Vec.distance(v2, v3), CardinalCurve.alpha);
      Vec m1 = Vec.mul(1.0 - u, Vec.add(Vec.sub(v2, v1), Vec.mul(t12, Vec.sub(Vec.div(Vec.sub(v1, v0), t01), Vec.div(Vec.sub(v2, v0), t01 + t12)))));
      Vec m2 = Vec.mul(1.0 - u, Vec.add(Vec.sub(v2, v1), Vec.mul(t12, Vec.sub(Vec.div(Vec.sub(v3, v2), t23), Vec.div(Vec.sub(v3, v1), t12 + t23)))));
      a.setDims(Vec.add(Vec.add(Vec.mul(2.0, Vec.sub(v1, v2)), m1), m2));
      b.setDims(Vec.sub(Vec.sub(Vec.sub(Vec.mul(-3.0, Vec.sub(v1, v2)), m1), m1), m2));
      c.setDims(m1);
      d.setDims(v1);
    }

    public static Vec getPosition(Vec v0, Vec v1, Vec v2, Vec v3, double t, double u) throws Vec.DimMismatchException {
      Vec a = new Vec(), b = new Vec(), c = new Vec(), d = new Vec();
      CardinalCurve.getCoefficients(v0, v1, v2, v3, u, a, b, c, d);
      return Vec.add(Vec.add(Vec.add(Vec.mul(a, t * t * t), Vec.mul(b, t * t)), Vec.mul(c, t)), d);
    }

    public static Vec getTangent(Vec v0, Vec v1, Vec v2, Vec v3, double t, double u) throws Vec.DimMismatchException {
      Vec a = new Vec(), b = new Vec(), c = new Vec(), d = new Vec();
      CardinalCurve.getCoefficients(v0, v1, v2, v3, u, a, b, c, d);
      return Vec.normalize(Vec.add(Vec.add(Vec.mul(a, 3 * t * t), Vec.mul(b, 2 * t)), c));
    }

    public static Vec getCurve(Vec v0, Vec v1, Vec v2, Vec v3, double t, double u) throws Vec.DimMismatchException {
      Vec a = new Vec(), b = new Vec(), c = new Vec(), d = new Vec();
      CardinalCurve.getCoefficients(v0, v1, v2, v3, u, a, b, c, d);
      return Vec.normalize(Vec.add(Vec.mul(a, 6 * t), Vec.mul(b, 2)));
    }

    public static Vec getPosition(Vec v0, Vec v1, Vec v2, Vec v3, double t) throws Vec.DimMismatchException {
      return CardinalCurve.getPosition(v0, v1, v2, v3, t, 0.5);
    }

    public static Vec getTangent(Vec v0, Vec v1, Vec v2, Vec v3, double t) throws Vec.DimMismatchException {
      return CardinalCurve.getTangent(v0, v1, v2, v3, t, 0.5);
    }

    public static Vec getCurve(Vec v0, Vec v1, Vec v2, Vec v3, double t) throws Vec.DimMismatchException {
      return CardinalCurve.getCurve(v0, v1, v2, v3, t, 0.5);
    }
  }

  private void buildLengthSumUtil() throws Vec.DimMismatchException {
    double[] lengths = new double[this.count];
    for (int i = 0; i < this.count; i++) lengths[i] = Vec.distance(this.points[i], this.points[i + 1]); // approx.
    this.lengthSumUtil = new SumUtil(lengths);
  }

  public int getDim() {
    return this.dim;
  }

  public int getCount() {
    return this.count;
  }

  public void getSegment(int i, Vec v0, Vec v1, Vec v2, Vec v3) throws Vec.DimMismatchException {
    if (i - 1 >= 0) v0.setDims(this.points[i - 1]);
    else v0.setDims(Vec.sub(Vec.mul(2.0, this.points[i]), this.points[i + 1]));
    if (i + 2 <= this.count) v3.setDims(this.points[i + 2]);
    else v3.setDims(Vec.sub(Vec.mul(2.0, this.points[i + 1]), this.points[i]));
    v1.setDims(this.points[i]);
    v2.setDims(this.points[i + 1]);
  }

  public double getSegment(double t, Vec v0, Vec v1, Vec v2, Vec v3) throws Vec.DimMismatchException {
    double s = t * this.getLength(0, this.count);
    int i = this.lengthSumUtil.lowerBound(s);
    this.getSegment(i, v0, v1, v2, v3);
    return (s - this.getLength(0, i)) / this.getLength(i, i + 1);
  }

  public Vec getPosition(double t, double u) throws Vec.DimMismatchException {
    Vec v0 = new Vec(), v1 = new Vec(), v2 = new Vec(), v3 = new Vec();
    double f = this.getSegment(t, v0, v1, v2, v3);
    return CardinalCurve.getPosition(v0, v1, v2, v3, f, u);
  }

  public Vec getTangent(double t, double u) throws Vec.DimMismatchException {
    Vec v0 = new Vec(), v1 = new Vec(), v2 = new Vec(), v3 = new Vec();
    double f = this.getSegment(t, v0, v1, v2, v3);
    return CardinalCurve.getTangent(v0, v1, v2, v3, f, u);
  }

  public Vec getCurve(double t, double u) throws Vec.DimMismatchException {
    Vec v0 = new Vec(), v1 = new Vec(), v2 = new Vec(), v3 = new Vec();
    double f = this.getSegment(t, v0, v1, v2, v3);
    return CardinalCurve.getCurve(v0, v1, v2, v3, f, u);
  }

  public double getLength(int l, int r) {
    return this.lengthSumUtil.rangeSum(l, r);
  }

  public Vec getPosition(double t) throws Vec.DimMismatchException {
    return this.getPosition(t, 0.5);
  }

  public Vec getTangent(double t) throws Vec.DimMismatchException {
    return this.getTangent(t, 0.5);
  }

  public Vec getCurve(double t) throws Vec.DimMismatchException {
    return this.getCurve(t, 0.5);
  }

  public double getLength() {
    return this.getLength(0, this.count);
  }
}