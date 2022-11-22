package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import java.lang.Exception;

public class Vec {
  public double[] dims;
  public int dim;

  public Vec() {}

  public Vec(int dim) {
    this.dims = new double[dim];
    this.dim = dim;
  }

  public Vec(int dim, double val) {
    this.dims = new double[dim];
    this.dim = dim;
    Arrays.fill(this.dims, val);
  }

  public Vec(double[] dims) {
    this.dims = dims;
    this.dim = dims.length;
  }

  public Vec(Vec vec) {
    this.dims = vec.dims.clone();
    this.dim = vec.getDim();
  }

  public static class DimMismatchException extends Exception {
    public DimMismatchException(String error) {
      super(error);
    }

    public DimMismatchException() {
      super("Vector dimension mismatch.");
    }
  }

  public void setDims(double[] dims) {
    this.dims = dims;
    this.dim = dims.length;
  }

  public void setDims(Vec vec) {
    this.dims = vec.dims.clone();
    this.dim = vec.getDim();
  }

  public int getDim() {
    return this.dim;
  }

  public static Vec add(Vec a, Vec b) throws Vec.DimMismatchException {
    if (a.getDim() != b.getDim()) throw new Vec.DimMismatchException();
    Vec res = new Vec(a);
    for (int i = 0; i < res.getDim(); i++) res.dims[i] += b.dims[i];
    return res;
  }

  public static Vec sub(Vec a, Vec b) throws Vec.DimMismatchException {
    if (a.getDim() != b.getDim()) throw new Vec.DimMismatchException();
    Vec res = new Vec(a);
    for (int i = 0; i < res.getDim(); i++) res.dims[i] -= b.dims[i];
    return res;
  }

  public static Vec mul(Vec a, Vec b) throws Vec.DimMismatchException {
    if (a.getDim() != b.getDim()) throw new Vec.DimMismatchException();
    Vec res = new Vec(a);
    for (int i = 0; i < res.getDim(); i++) res.dims[i] *= b.dims[i];
    return res;
  }

  public static Vec div(Vec a, Vec b) throws Vec.DimMismatchException {
    if (a.getDim() != b.getDim()) throw new Vec.DimMismatchException();
    Vec res = new Vec(a);
    for (int i = 0; i < res.getDim(); i++) res.dims[i] /= b.dims[i];
    return res;
  }

  public static Vec pow(Vec a, Vec b) throws Vec.DimMismatchException {
    if (a.getDim() != b.getDim()) throw new Vec.DimMismatchException();
    Vec res = new Vec(a.getDim());
    for (int i = 0; i < res.getDim(); i++) res.dims[i] = Math.pow(a.dims[i], b.dims[i]);
    return res;
  }

  public static double dot(Vec a, Vec b) throws Vec.DimMismatchException {
    if (a.getDim() != b.getDim()) throw new Vec.DimMismatchException();
    double res = 0.0;
    for (int i = 0; i < a.getDim(); i++) res += a.dims[i] * b.dims[i];
    return res;
  }

  public static Vec add(Vec a, double b) throws Vec.DimMismatchException {
    return Vec.add(a, new Vec(a.getDim(), b));
  }

  public static Vec sub(Vec a, double b) throws Vec.DimMismatchException {
    return Vec.sub(a, new Vec(a.getDim(), b));
  }

  public static Vec mul(Vec a, double b) throws Vec.DimMismatchException {
    return Vec.mul(a, new Vec(a.getDim(), b));
  }

  public static Vec div(Vec a, double b) throws Vec.DimMismatchException {
    return Vec.div(a, new Vec(a.getDim(), b));
  }

  public static Vec pow(Vec a, double b) throws Vec.DimMismatchException {
    return Vec.pow(a, new Vec(a.getDim(), b));
  }

  public static double dot(Vec a, double b) throws Vec.DimMismatchException {
    return Vec.dot(a, new Vec(a.getDim(), b));
  }

  public static Vec add(double a, Vec b) throws Vec.DimMismatchException {
    return Vec.add(new Vec(b.getDim(), a), b);
  }

  public static Vec sub(double a, Vec b) throws Vec.DimMismatchException {
    return Vec.sub(new Vec(b.getDim(), a), b);
  }

  public static Vec mul(double a, Vec b) throws Vec.DimMismatchException {
    return Vec.mul(new Vec(b.getDim(), a), b);
  }

  public static Vec div(double a, Vec b) throws Vec.DimMismatchException {
    return Vec.div(new Vec(b.getDim(), a), b);
  }

  public static Vec pow(double a, Vec b) throws Vec.DimMismatchException {
    return Vec.pow(new Vec(b.getDim(), a), b);
  }

  public static double dot(double a, Vec b) throws Vec.DimMismatchException {
    return Vec.dot(new Vec(b.getDim(), a), b);
  }

  public static double length(Vec a) throws Vec.DimMismatchException {
    return Math.sqrt(Vec.dot(a, a));
  }

  public static double distance(Vec a, Vec b) throws Vec.DimMismatchException {
    return Vec.length(Vec.sub(a, b));
  }

  public static Vec normalize(Vec a) throws Vec.DimMismatchException {
    return Vec.div(a, Vec.length(a));
  }

  public static Vec mix(Vec a, Vec b, Vec w) throws Vec.DimMismatchException {
    return Vec.add(Vec.mul(a, Vec.sub(1.0, w)), Vec.mul(b, w));
  }

  public static void checkDims(Vec[] points) throws Vec.DimMismatchException {
    int dim = points[0].getDim();
    for (int i = 1; i < points.length; i++) if (points[i].getDim() != dim) throw new Vec.DimMismatchException();
  }

  public static double[] buildPrefixes(Vec[] points) throws Vec.DimMismatchException {
    int count = points.length - 1;
    double[] prefixes = new double[count + 1];
    for (int i = 0; i < count; i++) prefixes[i + 1] = Vec.distance(points[i], points[i + 1]);
    for (int i = 0; i < count; i++) prefixes[i + 1] += prefixes[i];
    return prefixes;
  }
}