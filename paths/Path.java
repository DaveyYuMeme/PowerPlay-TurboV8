package org.firstinspires.ftc.teamcode.paths;

import org.firstinspires.ftc.teamcode.Vec;

public class Path {
  private Sampler positionSampler;
  private Sampler rotationSampler;

  public Path(Sampler positionSampler, Sampler rotationSampler) throws Vec.DimMismatchException {
    if (positionSampler.getDim() != 2) throw new Vec.DimMismatchException("Position sampler dimension mismatch");
    if (rotationSampler.getDim() != 1) throw new Vec.DimMismatchException("Rotation sampler dimension mismatch");
    this.positionSampler = positionSampler;
    this.rotationSampler = rotationSampler;
  }
}