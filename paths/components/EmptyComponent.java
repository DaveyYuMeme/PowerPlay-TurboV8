package org.firstinspires.ftc.teamcode.paths.components;

import org.firstinspires.ftc.teamcode.Vec;

public class EmptyComponent implements Component {
  public EmptyComponent() {}

  public int getDim() {
    return 0;
  }

  public int getCount() {
    return 0;
  }

  public Vec getPosition(double t) throws Vec.DimMismatchException {
    return null;
  }

  public Vec getTangent(double t) throws Vec.DimMismatchException {
    return null;
  }

  public Vec getCurve(double t) throws Vec.DimMismatchException {
    return null;
  }

  public double getLength() {
    return 0.0;
  }
}