within ;
model GearboxTest

  dogClutchSimple dogClutchSimple1(cDogTrans=10000, dDogTrans=10)
    annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
  Modelica.Blocks.Sources.BooleanPulse booleanPulse(period=1, width=90)
    annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.1)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J=0.01)
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c=10000, d
      =100) annotation (Placement(transformation(extent={{60,-10},{80,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia2(J=0.01)
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=1)
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  Modelica.Mechanics.Rotational.Sources.Torque torque
    annotation (Placement(transformation(extent={{-100,-10},{-80,10}})));
  Modelica.Blocks.Sources.SawTooth sawTooth(
    amplitude=100,
    period=1,
    startTime=-0.1,
    offset=-1)
    annotation (Placement(transformation(extent={{-140,-10},{-120,10}})));
  Modelica.Mechanics.Rotational.Components.IdealRollingWheel idealRollingWheel
    annotation (Placement(transformation(extent={{120,-10},{140,10}})));
  Modelica.Mechanics.Translational.Components.Mass mass(m=900)
    annotation (Placement(transformation(extent={{150,-10},{170,10}})));
  dogClutchSimple dogClutchSimple2(dDogTrans=1, cDogTrans=100000)
    annotation (Placement(transformation(extent={{-30,-80},{-10,-60}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia3(J=0.1)
    annotation (Placement(transformation(extent={{-70,-80},{-50,-60}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia4(J=0.01)
    annotation (Placement(transformation(extent={{30,-80},{50,-60}})));
  Modelica.Mechanics.Rotational.Components.SpringDamper springDamper1(c=10000,
      d=10) annotation (Placement(transformation(extent={{60,-80},{80,-60}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia5(J=0.01)
    annotation (Placement(transformation(extent={{90,-80},{110,-60}})));
  Modelica.Mechanics.Rotational.Components.IdealGear idealGear1(ratio=2)
    annotation (Placement(transformation(extent={{0,-80},{20,-60}})));
  Modelica.Mechanics.Rotational.Sources.Torque torque1
    annotation (Placement(transformation(extent={{-100,-80},{-80,-60}})));
  Modelica.Mechanics.Rotational.Components.IdealRollingWheel idealRollingWheel1
    annotation (Placement(transformation(extent={{120,-80},{140,-60}})));
  Modelica.Mechanics.Translational.Components.Mass mass1(m=900)
    annotation (Placement(transformation(extent={{150,-80},{170,-60}})));
  Modelica.Blocks.Sources.BooleanStep booleanStep(startTime=2)
    annotation (Placement(transformation(extent={{-100,-50},{-80,-30}})));
  dogClutchSimple dogClutchSimple3(dDogTrans=1, cDogTrans=100000)
    annotation (Placement(transformation(extent={{-40,-110},{-20,-90}})));
  Modelica.Mechanics.Rotational.Components.IdealGear idealGear2(ratio=1)
    annotation (Placement(transformation(extent={{0,-110},{20,-90}})));
  Modelica.Blocks.Logical.Not not1
    annotation (Placement(transformation(extent={{-48,-50},{-28,-30}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=200)
    annotation (Placement(transformation(extent={{-140,-80},{-120,-60}})));
equation
  connect(booleanPulse.y, dogClutchSimple1.bEngage)
    annotation (Line(points={{-79,40},{-20,40},{-20,10}}, color={255,0,255}));
  connect(inertia.flange_b, dogClutchSimple1.input_shaft)
    annotation (Line(points={{-40,0},{-30,0}}, color={0,0,0}));
  connect(inertia1.flange_b, springDamper.flange_a)
    annotation (Line(points={{50,0},{60,0}}, color={0,0,0}));
  connect(springDamper.flange_b, inertia2.flange_a)
    annotation (Line(points={{80,0},{90,0}}, color={0,0,0}));
  connect(dogClutchSimple1.output_shaft, idealGear.flange_a)
    annotation (Line(points={{-10,0},{0,0}}, color={0,0,0}));
  connect(idealGear.flange_b, inertia1.flange_a)
    annotation (Line(points={{20,0},{30,0}}, color={0,0,0}));
  connect(torque.flange, inertia.flange_a)
    annotation (Line(points={{-80,0},{-60,0}}, color={0,0,0}));
  connect(sawTooth.y, torque.tau)
    annotation (Line(points={{-119,0},{-102,0}}, color={0,0,127}));
  connect(inertia2.flange_b, idealRollingWheel.flangeR)
    annotation (Line(points={{110,0},{120,0}}, color={0,0,0}));
  connect(mass.flange_a, idealRollingWheel.flangeT)
    annotation (Line(points={{150,0},{140,0}}, color={0,127,0}));
  connect(inertia3.flange_b, dogClutchSimple2.input_shaft)
    annotation (Line(points={{-50,-70},{-30,-70}}, color={0,0,0}));
  connect(inertia4.flange_b, springDamper1.flange_a)
    annotation (Line(points={{50,-70},{60,-70}}, color={0,0,0}));
  connect(springDamper1.flange_b, inertia5.flange_a)
    annotation (Line(points={{80,-70},{90,-70}}, color={0,0,0}));
  connect(dogClutchSimple2.output_shaft, idealGear1.flange_a)
    annotation (Line(points={{-10,-70},{0,-70}}, color={0,0,0}));
  connect(idealGear1.flange_b, inertia4.flange_a)
    annotation (Line(points={{20,-70},{30,-70}}, color={0,0,0}));
  connect(torque1.flange, inertia3.flange_a)
    annotation (Line(points={{-80,-70},{-70,-70}}, color={0,0,0}));
  connect(inertia5.flange_b, idealRollingWheel1.flangeR)
    annotation (Line(points={{110,-70},{120,-70}}, color={0,0,0}));
  connect(mass1.flange_a, idealRollingWheel1.flangeT)
    annotation (Line(points={{150,-70},{140,-70}}, color={0,127,0}));
  connect(dogClutchSimple3.input_shaft, dogClutchSimple2.input_shaft)
    annotation (Line(points={{-40,-100},{-50,-100},{-50,-70},{-30,-70}}, color=
          {0,0,0}));
  connect(dogClutchSimple3.output_shaft, idealGear2.flange_a)
    annotation (Line(points={{-20,-100},{0,-100}}, color={0,0,0}));
  connect(idealGear2.flange_b, inertia4.flange_a)
    annotation (Line(points={{20,-100},{30,-100},{30,-70}}, color={0,0,0}));
  connect(booleanStep.y, not1.u)
    annotation (Line(points={{-79,-40},{-50,-40}}, color={255,0,255}));
  connect(not1.y, dogClutchSimple2.bEngage) annotation (Line(points={{-27,-40},
          {-20,-40},{-20,-60}}, color={255,0,255}));
  connect(dogClutchSimple3.bEngage, not1.u) annotation (Line(points={{-30,-90},
          {-46,-90},{-46,-56},{-62,-56},{-62,-40},{-50,-40}}, color={255,0,255}));
  connect(realExpression.y, torque1.tau)
    annotation (Line(points={{-119,-70},{-102,-70}}, color={0,0,127}));
  annotation (uses(Modelica(version="3.2.2")));
end GearboxTest;
