within ;
model Gearbox
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.1)
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
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
    annotation (Placement(transformation(extent={{-140,-40},{-120,-20}})));

  Modelica.Mechanics.Translational.Components.IdealGearR2T idealGearR2T(ratio=
        0.01/45*180/3.14)
    annotation (Placement(transformation(extent={{-50,30},{-30,50}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorque1(
      tau_constant=1)
    annotation (Placement(transformation(extent={{-120,30},{-100,50}})));
  Modelica.Mechanics.Translational.Components.Mass mass(m=0.01)
    annotation (Placement(transformation(extent={{-20,30},{0,50}})));
  dogClutch2 dogClutch2_1
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Mechanics.Rotational.Sources.TorqueStep torqueStep2(
    offsetTorque=0,
    stepTorque=-2,
    startTime=0.7)
    annotation (Placement(transformation(extent={{-140,60},{-120,80}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia3(J=0.001)
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
equation
  connect(springDamper.flange_b, inertia1.flange_a)
    annotation (Line(points={{70,0},{80,0}}, color={0,0,0}));
  connect(inertia2.flange_b, springDamper.flange_a)
    annotation (Line(points={{40,0},{50,0}}, color={0,0,0}));
  connect(constantTorque.flange, inertia.flange_a)
    annotation (Line(points={{-100,0},{-50,0}}, color={0,0,0}));
  connect(torqueStep.flange, inertia1.flange_b) annotation (Line(points={{100,-40},
          {110,-40},{110,0},{100,0}},color={0,0,0}));
  connect(torqueStep1.flange, constantTorque.flange)
    annotation (Line(points={{-120,-30},{-90,-30},{-90,0},{-100,0}},
                                                   color={0,0,0}));
  connect(idealGearR2T.flangeT, mass.flange_a)
    annotation (Line(points={{-30,40},{-20,40}}, color={0,127,0}));
  connect(dogClutch2_1.output_shaft, inertia2.flange_a)
    annotation (Line(points={{10,0},{20,0}}, color={0,0,0}));
  connect(inertia.flange_b, dogClutch2_1.input_shaft)
    annotation (Line(points={{-30,0},{-10,0}}, color={0,0,0}));
  connect(dogClutch2_1.fork, mass.flange_b)
    annotation (Line(points={{0,10},{0,40}}, color={0,127,0}));
  connect(constantTorque1.flange, inertia3.flange_a)
    annotation (Line(points={{-100,40},{-80,40}}, color={0,0,0}));
  connect(torqueStep2.flange, inertia3.flange_a) annotation (Line(points={{-120,70},
          {-90,70},{-90,40},{-80,40}},       color={0,0,0}));
  connect(inertia3.flange_b, idealGearR2T.flangeR)
    annotation (Line(points={{-60,40},{-50,40}}, color={0,0,0}));
  annotation (uses(Modelica(version="3.2.2")));
end Gearbox;
