package org.firstinspires.ftc.teamcode.paths.components;

import org.firstinspires.ftc.teamcode.Vec;

public class LinearComponent implements Component {
  private Vec v0;
  private Vec v1;
  private int dim;
  private double length;

  public LinearComponent(Vec v0, Vec v1) throws Vec.DimMismatchException {
    if (v0.getDim() != v1.getDim()) throw new Vec.DimMismatchException();
    this.v0 = v0;
    this.v1 = v1;
    this.dim = v0.getDim();
    this.length = Vec.distance(v0, v1);
  }

  public int getDim() {
    return this.dim;
  }

  public int getCount() {
    return 2;
  }

  public Vec getPosition(double t) throws Vec.DimMismatchException {
    return Vec.mix(this.v0, this.v1, new Vec(this.dim, t));
  }

  public Vec getTangent(double t) throws Vec.DimMismatchException {
    return Vec.normalize(Vec.sub(v1, v0));
  }

  public Vec getCurve(double t) throws Vec.DimMismatchException {
    return new Vec(this.dim, 0.0);
  }

  public double getLength() {
    return this.length;
  }
}