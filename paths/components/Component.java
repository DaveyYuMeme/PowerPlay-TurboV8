package org.firstinspires.ftc.teamcode;

public interface Component {
  public int getDim();
  public int getCount();
  public Vec getPosition(double t) throws Vec.DimMismatchException;
  public Vec getTangent(double t) throws Vec.DimMismatchException;
  public Vec getCurve(double t) throws Vec.DimMismatchException;
  public double getLength();
}