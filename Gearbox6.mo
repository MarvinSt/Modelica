within ;
model Gearbox6
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.1)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=0.11)
    annotation (Placement(transformation(extent={{80,-10},{100,10}})));
  Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c= 1000, d=
5) annotation (Placement(transformation(extent={{50,-10},{70,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=0.05)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque(
      tau_constant=10)
    annotation (Placement(transformation(extent={{-120,-10},{-100,10}})));
  Modelica.Mechanics.Rotational.Sources.TorqueStep torqueStep(
    startTime=0.5,
    offsetTorque=0,
    stepTorque=0)
    annotation (Placement(transformation(extent={{80,-50},{100,-30}})));
  Modelica.Mechanics.Rotational.Sources.TorqueStep torqueStep1(
    stepTorque=-15,
    offsetTorque=0,
    startTime=0.85)
    annotation (Placement(transformation(extent={{-120,-40},{-100,-20}})));

  Modelica.Mechanics.Translational.Components.Mass mass(m=0.01)
    annotation (Placement(transformation(extent={{-20,30},{0,50}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia3(J=0.001)
    annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
  dogClutch4 dogClutch4_1
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Mechanics.Translational.Components.SpringDamper springDamper1(c=
        10000, d=50)
    annotation (Placement(transformation(extent={{-50,30},{-30,50}})));
  TablurGearR2T tablurGearR2T(table=[0,0; 0.654498,0; 0.741765,0.0025; 0.785398,
        0.0025; 0.829031,0.0025; 0.916298,0; 2.225295,0; 2.312561,-0.0025;
        2.356194,-0.0025; 2.399828,-0.0025; 2.487094,0; 6.283185,0])
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
  Modelica.Mechanics.Rotational.Sources.Position position
    annotation (Placement(transformation(extent={{-140,30},{-120,50}})));
  Modelica.Blocks.Sources.Step step(
    height=2*45*3.14/180,
    offset=45*3.14/180,
    startTime=1)
    annotation (Placement(transformation(extent={{-180,30},{-160,50}})));
equation
  connect(springDamper.flange_b, inertia1.flange_a) annotation (
    Line(points = {{70, 0}, {80, 0}}, color = {0, 0, 0}));
  connect(inertia2.flange_b, springDamper.flange_a) annotation (
    Line(points = {{40, 0}, {50, 0}}, color = {0, 0, 0}));
  connect(constantTorque.flange, inertia.flange_a) annotation (
    Line(points={{-100,0},{-60,0}},      color = {0, 0, 0}));
  connect(torqueStep.flange, inertia1.flange_b) annotation (
    Line(points = {{100, -40}, {110, -40}, {110, 0}, {100, 0}}, color = {0, 0, 0}));
  connect(torqueStep1.flange, constantTorque.flange) annotation (
    Line(points={{-100,-30},{-80,-30},{-80,0},{-100,0}},          color = {0, 0, 0}));
  connect(inertia.flange_b, dogClutch4_1.input_shaft)
    annotation (Line(points={{-40,0},{-10,0}}, color={0,0,0}));
  connect(dogClutch4_1.output_shaft, inertia2.flange_a)
    annotation (Line(points={{10,0},{20,0}}, color={0,0,0}));
  connect(mass.flange_b, dogClutch4_1.fork)
    annotation (Line(points={{0,40},{0,10}}, color={0,127,0}));
  connect(springDamper1.flange_b, mass.flange_a)
    annotation (Line(points={{-30,40},{-20,40}}, color={0,127,0}));
  connect(inertia3.flange_b, tablurGearR2T.flangeR)
    annotation (Line(points={{-90,40},{-80,40}}, color={0,0,0}));
  connect(tablurGearR2T.flangeT, springDamper1.flange_a)
    annotation (Line(points={{-60,40},{-50,40}}, color={0,127,0}));
  connect(position.flange, inertia3.flange_a)
    annotation (Line(points={{-120,40},{-110,40}}, color={0,0,0}));
  connect(step.y, position.phi_ref)
    annotation (Line(points={{-159,40},{-142,40}}, color={0,0,127}));
  annotation (uses(Modelica(version="3.2.3")));
end Gearbox6;
