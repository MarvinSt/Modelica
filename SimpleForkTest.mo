within ;
model SimpleForkTest
  Modelica.Mechanics.Rotational.Sources.ConstantSpeed constantSpeed(w_fixed(
        displayUnit="deg/s") = 6.2831853071796)
    annotation (Placement(transformation(extent={{-96,-10},{-76,10}})));
  Modelica.Mechanics.Translational.Components.Fixed fixed
    annotation (Placement(transformation(extent={{72,-10},{92,10}})));
  Modelica.Mechanics.Translational.Components.SpringDamper springDamper1(c=10,
      d=0.1) annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.1)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  TableR2T tableR2T( smoothness=Modelica.Blocks.Types.Smoothness.MonotoneContinuousDerivative1,table=[0,0; 30,0; 40,-1; 50,-1; 60,0; 75,0; 85,1; 95,1; 105,
        0; 120,0; 130,-1; 140,-1; 150,0; 165,0; 175,1; 185,1; 195,0; 210,0; 220,
        -1; 230,-1; 240,0; 255,0; 265,1; 275,1; 285,0; 300,0; 310,-1; 320,-1;
        330,0; 360,0])
    annotation (Placement(transformation(extent={{-12,-10},{8,10}})));
equation
  connect(springDamper1.flange_b, fixed.flange)
    annotation (Line(points={{60,0},{82,0}}, color={0,127,0}));
  connect(constantSpeed.flange, inertia.flange_a)
    annotation (Line(points={{-76,0},{-60,0}}, color={0,0,0}));
  connect(tableR2T.flangeT, springDamper1.flange_a)
    annotation (Line(points={{8,0},{40,0}}, color={0,127,0}));
  connect(inertia.flange_b, tableR2T.flangeR)
    annotation (Line(points={{-40,0},{-12,0}}, color={0,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.2")));
end SimpleForkTest;
