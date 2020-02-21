within ;
model dogClutchSimple
  Modelica.Mechanics.Rotational.Interfaces.Flange_a input_shaft
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b output_shaft
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));


    parameter Integer nDogs = 5;

    parameter Real rGear = 1.0;

    final constant Real pi = 2 * Modelica.Math.asin(1.0);



    parameter Real rDogDrive = 0.7;
    parameter Real rDogCoast = 0.35;
    parameter Real rDogDelRelease = 0.01;

    Real aDogDrive = rDogDrive * aDogRep;
    Real aDogCoast = rDogCoast * aDogRep;
    Real aDogDelRelease = rDogDelRelease * aDogRep;


    parameter Real cDogTrans = 1000 * 180 / pi;
    parameter Real dDogTrans = 0.001 * cDogTrans;


    Real aDogRep = 2 * pi / nDogs;

    Real aDogMod;
    Real aDog;
    Real vDog;
    Real tauDogTrans;
    Real aDogOffset;


    Integer iDogState( start = Open);


    final constant Integer Open = 0;
    final constant Integer Engaging = 1;
    final constant Integer Engaged = 2;


  Modelica.Blocks.Interfaces.BooleanInput bEngage annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={0,100})));




algorithm
  // Simple state machine
  when iDogState == Open and bEngage and aDogMod > aDogCoast and aDogMod < aDogDrive then
    iDogState :=Engaged;
       elsewhen
            iDogState == Engaged and not bEngage and aDog > aDogCoast + aDogDelRelease and aDog < aDogDrive - aDogDelRelease then
    iDogState :=Open;
  end when;

equation
  when pre(iDogState) == Engaged then
    // If the dog engages reset the angle
    aDogOffset = floor((input_shaft.phi - rGear * output_shaft.phi) / aDogRep) * aDogRep;
  end when;

  // Calculate relative dog angle
  aDog   = input_shaft.phi - rGear * output_shaft.phi - aDogOffset;
  vDog   = der(input_shaft.phi - rGear * output_shaft.phi);

  // Calculate dog window state (wrap to zero for each dog)
  aDogMod = noEvent(mod(input_shaft.phi - rGear * output_shaft.phi, aDogRep));

  // Calculate dog clutch torque transfer (tangential force)
  tauDogTrans = noEvent(if not pre(iDogState) == Engaged then 0 else
     if aDog > aDogDrive then max((aDog - aDogDrive) * cDogTrans + vDog * dDogTrans, 0) else
     if aDog < aDogCoast then min((aDog - aDogCoast) * cDogTrans + vDog * dDogTrans, 0) else
     0);

  // Torque transfer
  input_shaft.tau * rGear = +tauDogTrans;
  output_shaft.tau = -tauDogTrans;

  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.2")));
end dogClutchSimple;
