within ;
block TableR2T
  "Table look-up in one dimension (matrix/file) with one input and n outputs"
  // extends Modelica.Blocks.Interfaces.SIMO(final nout=size(columns, 1));
  extends Modelica.Blocks.Icons.Block;




  parameter Real table[:, :] = fill(0.0, 0, 2)
    "Table matrix (grid = first column; e.g., table=[0,2])"
    annotation (Dialog(group="Table data definition",enable=not tableOnFile));


parameter Real ratio = 180 / pi;

  parameter Modelica.Blocks.Types.Smoothness smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments
    "Smoothness of table interpolation"
    annotation (Dialog(group="Table data interpretation"));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flangeR
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Mechanics.Translational.Interfaces.Flange_b flangeT
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));



protected
parameter Integer nout=size(columns, 1) "Number of outputs";

  parameter Integer columns[:]=2:size(table, 2)
    "Columns of table to be interpolated"
    annotation (Dialog(group="Table data interpretation"));

  Real u;
  Real y;

  Modelica.Mechanics.Translational.Interfaces.InternalSupport internalSupportT
    annotation (Placement(transformation(extent={{90,-50},{110,-30}})));
  Modelica.Mechanics.Rotational.Interfaces.InternalSupport internalSupportR
    annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
  Modelica.Mechanics.Translational.Components.Fixed fixed
    annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
  Modelica.Mechanics.Rotational.Components.Fixed fixed1
    annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));


  final constant Real pi = 2.0 * asin(1.0);

  Modelica.Blocks.Types.ExternalCombiTable1D tableID=
      Modelica.Blocks.Types.ExternalCombiTable1D("NoName", "NoName", table, columns, smoothness) "External table object";

  parameter Real tableOnFileRead(fixed=false)
    "= 1, if table was successfully read from file";

  function getTableValue "Interpolate 1-dim. table defined by matrix"
    extends Modelica.Icons.Function;
    input Modelica.Blocks.Types.ExternalCombiTable1D tableID;
    input Integer icol;
    input Real u;
    input Real tableAvailable
      "Dummy input to ensure correct sorting of function calls";
    output Real y;
    external"C" y = ModelicaStandardTables_CombiTable1D_getValue(tableID, icol, u)
      annotation (Library={"ModelicaStandardTables", "ModelicaMatIO", "zlib"});
    annotation (derivative(noDerivative=tableAvailable) = getDerTableValue);
  end getTableValue;

  function getTableValueNoDer
    "Interpolate 1-dim. table defined by matrix (but do not provide a derivative function)"
    extends Modelica.Icons.Function;
    input Modelica.Blocks.Types.ExternalCombiTable1D tableID;
    input Integer icol;
    input Real u;
    input Real tableAvailable
      "Dummy input to ensure correct sorting of function calls";
    output Real y;
    external"C" y = ModelicaStandardTables_CombiTable1D_getValue(tableID, icol, u)
      annotation (Library={"ModelicaStandardTables", "ModelicaMatIO", "zlib"});
  end getTableValueNoDer;

  function getDerTableValue
    "Derivative of interpolated 1-dim. table defined by matrix"
    extends Modelica.Icons.Function;
    input Modelica.Blocks.Types.ExternalCombiTable1D tableID;
    input Integer icol;
    input Real u;
    input Real tableAvailable
      "Dummy input to ensure correct sorting of function calls";
    input Real der_u;
    output Real der_y;
    external"C" der_y = ModelicaStandardTables_CombiTable1D_getDerValue(tableID, icol, u, der_u)
      annotation (Library={"ModelicaStandardTables", "ModelicaMatIO", "zlib"});
  end getDerTableValue;


equation
  // Relative angle and displacement
  u = flangeR.phi - internalSupportR.phi;
  y = flangeT.s - internalSupportT.s;

  // Force equilibrium
  0 = flangeR.tau + der(y) * flangeT.f;

  // Constraints
  0 = flangeR.tau + internalSupportR.tau;
  0 = flangeT.f + internalSupportT.f;

  if smoothness == Modelica.Blocks.Types.Smoothness.ConstantSegments then
      y = getTableValueNoDer(tableID, 1, u * ratio, tableOnFileRead);
  else
      y = getTableValue(tableID, 1, u * ratio, tableOnFileRead);
  end if;

  connect(internalSupportR.flange, fixed1.flange)
    annotation (Line(points={{-100,-40},{-100,-60}}, color={0,0,0}));
  connect(internalSupportT.flange, fixed.flange)
    annotation (Line(points={{100,-40},{100,-60}}, color={0,127,0}));
  annotation (Documentation(info="<html>
<p> Block has one continuous Real input signal and a
    vector of continuous Real output signals.</p>

</html>"),
    Documentation(info="<html>
<p>
<b>Linear interpolation</b> in <b>one</b> dimension of a <b>table</b>.
Via parameter <b>columns</b> it can be defined how many columns of the
table are interpolated. If, e.g., icol={2,4}, it is assumed that one input
and 2 output signals are present and that the first output interpolates
via column 2 and the second output interpolates via column 4 of the
table matrix.
</p>
<p>
The grid points and function values are stored in a matrix \"table[i,j]\",
where the first column \"table[:,1]\" contains the grid points and the
other columns contain the data to be interpolated. Example:
</p>
<pre>
   table = [0,  0;
            1,  1;
            2,  4;
            4, 16]
   If, e.g., the input u = 1.0, the output y =  1.0,
       e.g., the input u = 1.5, the output y =  2.5,
       e.g., the input u = 2.0, the output y =  4.0,
       e.g., the input u =-1.0, the output y = -1.0 (i.e., extrapolation).
</pre>
<ul>
<li> The interpolation is <b>efficient</b>, because a search for a new interpolation
     starts at the interval used in the last call.</li>
<li> If the table has only <b>one row</b>, the table value is returned,
     independent of the value of the input signal.</li>
<li> If the input signal <b>u</b> is <b>outside</b> of the defined <b>interval</b>, i.e.,
     u &gt; table[size(table,1),1] or u &lt; table[1,1], the corresponding
     value is also determined by linear
     interpolation through the last or first two points of the table.</li>
<li> The grid values (first column) have to be strictly increasing.</li>
</ul>
<p>
The table matrix can be defined in the following ways:
</p>
<ol>
<li> Explicitly supplied as <b>parameter matrix</b> \"table\",
     and the other parameters have the following values:
<pre>
   tableName is \"NoName\" or has only blanks,
   fileName  is \"NoName\" or has only blanks.
</pre></li>
<li> <b>Read</b> from a <b>file</b> \"fileName\" where the matrix is stored as
      \"tableName\". Both ASCII and MAT-file format is possible.
      (The ASCII format is described below).
      The MAT-file format comes in four different versions: v4, v6, v7 and v7.3.
      The library supports at least v4, v6 and v7 whereas v7.3 is optional.
      It is most convenient to generate the MAT-file from FreeMat or MATLAB&reg;
      by command
<pre>
   save tables.mat tab1 tab2 tab3
</pre>
      or Scilab by command
<pre>
   savematfile tables.mat tab1 tab2 tab3
</pre>
      when the three tables tab1, tab2, tab3 should be used from the model.<br>
      Note, a fileName can be defined as URI by using the helper function
      <a href=\"modelica://Modelica.Utilities.Files.loadResource\">loadResource</a>.</li>
<li>  Statically stored in function \"usertab\" in file \"usertab.c\".
      The matrix is identified by \"tableName\". Parameter
      fileName = \"NoName\" or has only blanks. Row-wise storage is always to be
      preferred as otherwise the table is reallocated and transposed.
      See the <a href=\"modelica://Modelica.Blocks.Tables\">Tables</a> package
      documentation for more details.</li>
</ol>
<p>
When the constant \"NO_FILE_SYSTEM\" is defined, all file I/O related parts of the
source code are removed by the C-preprocessor, such that no access to files takes place.
</p>
<p>
If tables are read from an ASCII-file, the file needs to have the
following structure (\"-----\" is not part of the file content):
</p>
<pre>
-----------------------------------------------------
#1
double tab1(5,2)   # comment line
  0   0
  1   1
  2   4
  3   9
  4  16
double tab2(5,2)   # another comment line
  0   0
  2   2
  4   8
  6  18
  8  32
-----------------------------------------------------
</pre>
<p>
Note, that the first two characters in the file need to be
\"#1\" (a line comment defining the version number of the file format).
Afterwards, the corresponding matrix has to be declared
with type (= \"double\" or \"float\"), name and actual dimensions.
Finally, in successive rows of the file, the elements of the matrix
have to be given. The elements have to be provided as a sequence of
numbers in row-wise order (therefore a matrix row can span several
lines in the file and need not start at the beginning of a line).
Numbers have to be given according to C syntax (such as 2.3, -2, +2.e4).
Number separators are spaces, tab (\t), comma (,), or semicolon (;).
Several matrices may be defined one after another. Line comments start
with the hash symbol (#) and can appear everywhere.
Other characters, like trailing non comments, are not allowed in the file.
</p>
<p>
MATLAB is a registered trademark of The MathWorks, Inc.
</p>
</html>"),
    Icon(
    coordinateSystem(preserveAspectRatio=true,
      extent={{-100.0,-100.0},{100.0,100.0}}),
      graphics={
    Line(points={{-60.0,40.0},{-60.0,-40.0},{60.0,-40.0},{60.0,40.0},{30.0,40.0},{30.0,-40.0},{-30.0,-40.0},{-30.0,40.0},{-60.0,40.0},{-60.0,20.0},{60.0,20.0},{60.0,0.0},{-60.0,0.0},{-60.0,-20.0},{60.0,-20.0},{60.0,-40.0},{-60.0,-40.0},{-60.0,40.0},{60.0,40.0},{60.0,-40.0}}),
    Line(points={{0.0,40.0},{0.0,-40.0}}),
    Rectangle(fillColor={255,215,136},
      fillPattern=FillPattern.Solid,
      extent={{-60.0,20.0},{-30.0,40.0}}),
    Rectangle(fillColor={255,215,136},
      fillPattern=FillPattern.Solid,
      extent={{-60.0,0.0},{-30.0,20.0}}),
    Rectangle(fillColor={255,215,136},
      fillPattern=FillPattern.Solid,
      extent={{-60.0,-20.0},{-30.0,0.0}}),
    Rectangle(fillColor={255,215,136},
      fillPattern=FillPattern.Solid,
      extent={{-60.0,-40.0},{-30.0,-20.0}})}),
    Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}), graphics={
        Rectangle(
          extent={{-60,60},{60,-60}},
          fillColor={235,235,235},
          fillPattern=FillPattern.Solid,
          lineColor={0,0,255}),
        Line(points={{-100,0},{-58,0}}, color={0,0,255}),
        Line(points={{60,0},{100,0}}, color={0,0,255}),
        Text(
          extent={{-100,100},{100,64}},
          textString="1 dimensional linear table interpolation",
          lineColor={0,0,255}),
        Line(points={{-54,40},{-54,-40},{54,-40},{54,40},{28,40},{28,-40},{-28,
              -40},{-28,40},{-54,40},{-54,20},{54,20},{54,0},{-54,0},{-54,-20},
              {54,-20},{54,-40},{-54,-40},{-54,40},{54,40},{54,-40}}, color={
              0,0,0}),
        Line(points={{0,40},{0,-40}}),
        Rectangle(
          extent={{-54,40},{-28,20}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-54,20},{-28,0}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-54,0},{-28,-20}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-54,-20},{-28,-40}},
          lineColor={0,0,0},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-52,56},{-34,44}},
          textString="u",
          lineColor={0,0,255}),
        Text(
          extent={{-22,54},{2,42}},
          textString="y[1]",
          lineColor={0,0,255}),
        Text(
          extent={{4,54},{28,42}},
          textString="y[2]",
          lineColor={0,0,255}),
        Text(
          extent={{0,-40},{32,-54}},
          textString="columns",
          lineColor={0,0,255})}),
    uses(Modelica(version="3.2.2")));
end TableR2T;
