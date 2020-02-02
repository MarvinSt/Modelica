within ;
block dogClutch5
  Modelica.Mechanics.Rotational.Interfaces.Flange_a input_shaft
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b output_shaft
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Mechanics.Translational.Interfaces.Flange_a fork
    annotation (Placement(transformation(extent={{-10,90},{10,110}})));

import SI = Modelica.SIunits;

final constant Integer Engaged = 0;
final constant Integer Sliding = 1;
final constant Integer Open = 2;

parameter Integer nDogs = 5;

parameter SI.Angle aDogWindow = 15 * pi / 180;
parameter SI.Angle aDogLockAngle = 1 * pi / 180;

parameter Real cDogTrans = 1000;
parameter Real dDogTrans = 20;

parameter Real cForkStop = 1000000;
parameter Real dForkStop = 2000;

parameter SI.Distance sDogThrow = 0.0025;
parameter SI.Distance sDogGap = 0.0020;

parameter SI.Distance rDogRing = 0.05;

parameter Real rGears[:] = { 1.0, 1.2};

SI.Angle aDog;
SI.Angle aDogMod;
discrete SI.Angle aDogOffset;

SI.AngularVelocity vDog;
SI.Torque tauDogTrans;

SI.Force fForkDrive;

SI.Position sFork;
SI.Force fFork;

Integer iDogState;

Real rGear; // = 1.0;

protected
  parameter SI.Angle aDogRep = 2 * pi / nDogs;
  parameter SI.Angle aDogHalfWindow = aDogWindow / 2;
  final constant Real pi = 2 * Modelica.Math.asin(1.0);

equation
  when iDogState == Engaged then
    // If the dog engages reset the angle
    aDogOffset = floor((input_shaft.phi - rGear * output_shaft.phi) / aDogRep) * aDogRep;
  end when;

  // Dynamically change gear ratio
  rGear = rGears[1] + (rGears[2] - rGears[1]) * min(max((sDogGap - sFork) / (sDogGap), 0), 1);

  // Calculate relative dog angle
  aDog   = input_shaft.phi - rGear * output_shaft.phi - aDogOffset - aDogRep / 2;
  vDog   = der(input_shaft.phi - rGear * output_shaft.phi);

  // Calculate dog window state (wrap to zero for each dog)
  aDogMod = noEvent(mod(input_shaft.phi - rGear * output_shaft.phi, aDogRep) - aDogRep / 2);

  // Calculate dog clutch torque transfer (tangential force)
  tauDogTrans = noEvent(if not pre(iDogState) == Engaged then 0 else
     if aDog > +aDogHalfWindow then max((aDog - aDogHalfWindow) * cDogTrans + vDog * dDogTrans, 0) else
     if aDog < -aDogHalfWindow then min((aDog + aDogHalfWindow) * cDogTrans + vDog * dDogTrans, 0) else
     0);

  // Torque transfer
  input_shaft.tau * rGear = +tauDogTrans;
  output_shaft.tau = -tauDogTrans;

  // State machine
  iDogState = if abs(fork.s) < sDogGap then Open else
  if pre(iDogState) == Engaged or abs(aDogMod) < aDogHalfWindow then Engaged
  else Sliding;

  // Reaction force on the fork
  if iDogState == Sliding then
      // Dog to dog fork contact force (axial force)
      fForkDrive = 0;
      fFork = noEvent(if sFork > +sDogGap then max((sFork - sDogGap) * cForkStop + der(sFork) * dForkStop, 0)
             else
                 if sFork < -sDogGap then min((sFork + sDogGap) * cForkStop + der(sFork) * dForkStop, 0)
             else
                 0);
  elseif iDogState == Engaged then
      // Dog engaged + meshing force due to dog angle and drive torque (axial force)
      fForkDrive = noEvent(sign(sFork) * abs(tauDogTrans) / rDogRing * sin(aDogLockAngle) / cos(aDogLockAngle));
      fFork = noEvent(if sFork > +sDogThrow then max((sFork - sDogThrow) * cForkStop + der(sFork) * dForkStop, 0)
             else
                 if sFork < -sDogThrow then min((sFork + sDogThrow) * cForkStop + der(sFork) * dForkStop, 0)
             else
                 0);
  else
      // No significant forces acting on the fork
      fForkDrive = 0;
      fFork = 0;
  end if;

  // Fork force and position
  fork.f = fFork - fForkDrive;
  fork.s = sFork;

  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.3")));
end dogClutch5;
