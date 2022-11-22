package org.firstinspires.ftc.teamcode.paths.components;

import org.firstinspires.ftc.teamcode.Vec;

public interface Component {
  public int getDim();
  public int getCount();
  public Vec getPosition(double t) throws Vec.DimMismatchException;
  public Vec getTangent(double t) throws Vec.DimMismatchException;
  public Vec getCurve(double t) throws Vec.DimMismatchException;
  public double getLength();
}