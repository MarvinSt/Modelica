within ;
block dogClutch2
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

parameter SI.Angle aDogHoldAngle = 0.5 * pi / 180;
parameter SI.Force fMaxHoldForce = 1000;

parameter Real cDogTrans = 1000;
parameter Real dDogTrans = 20;

parameter Real cForkStop = 10000;
parameter Real dForkStop = 200;

parameter SI.Distance sDogThrow = 0.0025;
parameter SI.Distance sDogGap = 0.0020;

SI.Angle aDog;
SI.Angle aDogMod;
SI.Angle aDogOffset;

SI.AngularVelocity vDog;
SI.Torque tauDogTrans;

SI.Position sFork;
SI.Force fFork;

// Internal variables
Integer iDogState;
Boolean bEngaged;

protected
  parameter SI.Angle aDogRep = 2 * pi / nDogs;
  parameter SI.Angle aDogHalfWindow = aDogWindow / 2;
  final constant Real pi = 2 * Modelica.Math.asin(1.0);

equation
  // Set a boolean whether the dog is engaged
  bEngaged = iDogState == Engaged;



  der(aDogOffset) = 0;
  when bEngaged then
    // If the dog engages reset the angle
    reinit(aDogOffset, floor((input_shaft.phi - output_shaft.phi) / aDogRep) * aDogRep);
  end when;

  // Calculate relative dog angle
  aDog = input_shaft.phi - output_shaft.phi - aDogOffset - aDogRep / 2;
  vDog = der(input_shaft.phi - output_shaft.phi);

  // Calculate dog window state (wrap to zero for each dog)
  aDogMod = mod(input_shaft.phi - output_shaft.phi, aDogRep) - aDogRep / 2;

  // Calculate dog clutch torque transfer
  tauDogTrans = if not pre(bEngaged) then 0
 else
     if aDog > +aDogHalfWindow then (aDog - aDogHalfWindow) * cDogTrans + max(vDog, 0) * dDogTrans
 else
     if aDog < -aDogHalfWindow then (aDog + aDogHalfWindow) * cDogTrans + min(vDog, 0) * dDogTrans
 else
     0;

  // Torque transfer
  input_shaft.tau  = +tauDogTrans;
  output_shaft.tau = -tauDogTrans;

  // State machine
  iDogState = if abs(fork.s) < sDogGap then Open
 else
     if pre(bEngaged) or abs(aDogMod) < aDogHalfWindow then Engaged
 else
     Sliding;

  // Reaction force on the fork
  if iDogState == Open then
      fFork = 0;
  elseif iDogState == Sliding then
      // Dog to dog contact force
      fFork = sign(sFork) * (max(abs(sFork) - sDogGap, 0) * cForkStop + max(sign(sFork) * der(sFork), 0) * dForkStop);
  elseif aDog > +aDogHalfWindow - aDogHoldAngle or aDog < -aDogHalfWindow + aDogHoldAngle then
      // Drive and coast side fork engagement force
      fFork = max( min( (sFork - sign(sFork) * sDogThrow) * cForkStop + der(sFork) * dForkStop, fMaxHoldForce), -fMaxHoldForce);
  else
      fFork = 0;
  end if;


  fork.f = fFork;
  fork.s = sFork;

  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.2")));
end dogClutch2;
