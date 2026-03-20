within IndustrialControlSystems.Applications.ControlStrategies;
model SelectorControl
  "Drum boiler with min-select override pressure control on the steam valve"
  extends Modelica.Icons.Example;

  // -----------------------------------------------------------------------
  //  Required inner declarations for Modelica.Fluid
  // -----------------------------------------------------------------------
  inner Modelica.Fluid.System system
    annotation (Placement(transformation(extent={{-200,160},{-180,180}})));

  // -----------------------------------------------------------------------
  //  Plant: EquilibriumDrumBoiler (Astrom-Bell model from MSL Fluid)
  // -----------------------------------------------------------------------
  Modelica.Fluid.Examples.DrumBoiler.BaseClasses.EquilibriumDrumBoiler
    evaporator(
    m_D      = 300e3,
    cp_D     = 500,
    V_t      = 100,
    V_l_start = 67,
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    energyDynamics = Modelica.Fluid.Types.Dynamics.FixedInitial,
    massDynamics   = Modelica.Fluid.Types.Dynamics.FixedInitial,
    p_start=8000000)
    annotation (Placement(transformation(extent={{-120,-40},{-100,-20}})));

  // -----------------------------------------------------------------------
  //  Heat source (furnace): 0 MW at t=0 ramping to 400 MW at t=3600 s
  // -----------------------------------------------------------------------
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow furnace
    annotation (Placement(transformation(
        origin={-110,-65},
        extent={{-10,-10},{10,10}},
        rotation=90)));

  Modelica.Blocks.Math.Gain MW2W(k=1e6)
    annotation (Placement(transformation(extent={{-150,-90},{-130,-70}})));

  Modelica.Blocks.Sources.TimeTable q_F_Tab(table=[0,300; 3600,400; 7210,400])
    annotation (Placement(transformation(extent={{-190,-90},{-170,-70}})));

  // -----------------------------------------------------------------------
  //  Feedwater pump (mass-flow source, flow set by drum level controller)
  // -----------------------------------------------------------------------
  Modelica.Fluid.Sources.MassFlowSource_h pump(
    nPorts        = 1,
    h             = 5e5,
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    use_m_flow_in = true)
    annotation (Placement(transformation(extent={{-170,-40},{-150,-20}})));

  // -----------------------------------------------------------------------
  //  Drum level control loop (retained from the MSL DrumBoiler example)
  // -----------------------------------------------------------------------
  Modelica.Blocks.Sources.Constant levelSetPoint(k=67)
    annotation (Placement(transformation(extent={{-190,20},{-170,40}})));

  Modelica.Blocks.Math.Feedback feedback
    annotation (Placement(transformation(extent={{-165,15},{-145,35}})));

  Modelica.Blocks.Continuous.PI levelController(
    T        = 120,
    k        = 10,
    initType = Modelica.Blocks.Types.Init.InitialState)
    annotation (Placement(transformation(extent={{-140,18},{-120,38}})));

  Modelica.Blocks.Nonlinear.Limiter limiter(uMin=0, uMax=500)
    annotation (Placement(transformation(extent={{-115,18},{-95,38}})));

  // -----------------------------------------------------------------------
  //  Steam-side sensors
  // -----------------------------------------------------------------------
  Modelica.Fluid.Sensors.MassFlowRate massFlowRate(
    redeclare package Medium = Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(
        origin={-60,-30},
        extent={{10,10},{-10,-10}},
        rotation=180)));

  Modelica.Fluid.Sensors.Temperature temperature(
    redeclare package Medium = Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(
        origin={-80,-10},
        extent={{10,10},{-10,-10}},
        rotation=180)));

  Modelica.Fluid.Sensors.Pressure upstreamPressure(redeclare package Medium =
        Modelica.Media.Examples.TwoPhaseWater)
    annotation (Placement(transformation(extent={{-50,-66},{-30,-46}})));

  Modelica.Fluid.Sensors.Pressure downstreamPressure(redeclare package Medium
      = Medium)
    annotation (Placement(transformation(extent={{90,-50},{110,-30}})));

  // -----------------------------------------------------------------------
  //  Steam valve (opening driven by min-selector output)
  // -----------------------------------------------------------------------
  Modelica.Fluid.Valves.ValveLinear SteamValve(
    dp_nominal     = 9000000,
    m_flow_nominal = 180,
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{50,-90},{70,-70}})));

  // -----------------------------------------------------------------------
  //  Downstream boundary (steam consumer / condenser at 0.5 bar)
  // -----------------------------------------------------------------------
  Modelica.Fluid.Sources.FixedBoundary sink(
    p=7000000,
    T=500,
    nPorts=1,
    redeclare package Medium = Medium)
    annotation (Placement(transformation(
        origin={190,-80},
        extent={{10,-10},{-10,10}})));

  // -----------------------------------------------------------------------
  //  Unit conversion for observable outputs
  // -----------------------------------------------------------------------

  Modelica.Thermal.HeatTransfer.Celsius.FromKelvin K2degC
    annotation (Placement(transformation(extent={{-61,8},{-41,28}})));

  // -----------------------------------------------------------------------
  //  Selector control — setpoints (both on far left, above level loop)
  // -----------------------------------------------------------------------
  parameter Real p_SP_upstream   = 85e5
    "Upstream (drum) pressure ceiling [Pa] — overpressure protection setpoint"
    annotation (Dialog(group = "Selector control setpoints"));
  parameter Real p_SP_downstream = 80e5
    "Downstream delivery pressure setpoint [Pa]"
    annotation (Dialog(group = "Selector control setpoints"));

  Modelica.Blocks.Sources.Constant spDownstream(k=p_SP_downstream)
    annotation (Placement(transformation(extent={{192,10},{172,29}})));
  Modelica.Blocks.Sources.Constant spUpstream(k=p_SP_upstream)
    annotation (Placement(transformation(extent={{-60,39},{-40,60}})));

  // -----------------------------------------------------------------------
  //  Selector control — ICS PI controllers
  //
  //  Layout: ctrlUpstream LEFT of valve (above upstream sensor),
  //          ctrlDownstream RIGHT of valve (above downstream sensor),
  //          minSelector centered directly above the valve.
  //
  //  ctrlUpstream   : PV = upstream pressure [Pa], SP = p_SP_upstream
  //    Overpressure protection — closes valve when drum pressure exceeds
  //    ceiling; always vetoes a larger opening from ctrlDownstream.
  //
  //  ctrlDownstream : PV = downstream pressure [Pa], SP = p_SP_downstream
  //    Delivery pressure control — opens valve to meet consumer demand.
  // -----------------------------------------------------------------------
  IndustrialControlSystems.Controllers.PI ctrlUpstream(
    useTS      = true,
    AntiWindup = true,
    CSmin      = 0,
    CSmax      = 1,
    CS_start   = 0,
    Kp         = 1e-7,
    Ti         = 600,
    Ts         = 0)
    annotation (Placement(transformation(extent={{-20,-1},{0,20}})));

  IndustrialControlSystems.Controllers.PI ctrlDownstream(
    useTS      = true,
    AntiWindup = true,
    CSmin      = 0,
    CSmax      = 1,
    CS_start   = 0,
    Kp         = 2e-7,
    Ti         = 300,
    Ts         = 0)
    annotation (Placement(transformation(extent={{140,0},{120,20}})));

  // -----------------------------------------------------------------------
  //  Selector control — MIN selector (directly above valve)
  // -----------------------------------------------------------------------
  Modelica.Blocks.Math.Min minSelector
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={60,-46})));

  // -----------------------------------------------------------------------
  //  Tracking switch logic — placed above their respective controllers
  //  greaterUp   : true when ctrlUpstream.CS   > minSelector.y
  //  greaterDown : true when ctrlDownstream.CS > minSelector.y
  // -----------------------------------------------------------------------
  Modelica.Blocks.Logical.Greater greaterUp
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-6,58})));

  Modelica.Blocks.Logical.Greater greaterDown
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={126,58})));

  Modelica.Fluid.Vessels.ClosedVolume volume(
    V=2,
    nPorts=3,
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{110,-70},{130,-50}})));
  Modelica.Fluid.Pipes.StaticPipe pipe(
    length=10,
    diameter=0.1,
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{142,-90},{162,-70}})));
  replaceable package Medium = Modelica.Media.Water.StandardWater annotation (
      choicesAllMatching=true);
equation
  // -----------------------------------------------------------------------
  //  Fuel heat-rate → furnace
  // -----------------------------------------------------------------------
  connect(q_F_Tab.y, MW2W.u)
    annotation (Line(points={{-169,-80},{-152,-80}}, color={0,0,127},
      smooth=Smooth.None));
  connect(MW2W.y, furnace.Q_flow)
    annotation (Line(points={{-129,-80},{-110,-80},{-110,-75}}, color={0,0,127},
      smooth=Smooth.None));
  connect(furnace.port, evaporator.heatPort)
    annotation (Line(points={{-110,-55},{-110,-40}}, color={191,0,0},
      smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Drum level control loop
  // -----------------------------------------------------------------------
  connect(levelSetPoint.y, feedback.u1)
    annotation (Line(points={{-169,30},{-163,25}}, color={0,0,127},
      smooth=Smooth.None));
  connect(evaporator.V, feedback.u2)
    annotation (Line(points={{-106,-19},{-106,5},{-155,5},{-155,17}},
      color={0,0,127}, smooth=Smooth.None));
  connect(feedback.y, levelController.u)
    annotation (Line(points={{-146,25},{-142,28}}, color={0,0,127},
      smooth=Smooth.None));
  connect(levelController.y, limiter.u)
    annotation (Line(points={{-119,28},{-117,28}}, color={0,0,127},
      smooth=Smooth.None));
  connect(limiter.y, pump.m_flow_in)
    annotation (Line(points={{-94,28},{-88,28},{-88,-8},{-178,-8},{-178,-22},
      {-170,-22}}, color={0,0,127}, smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Fluid connections: pump → evaporator → steam sensors → valve → sink
  // -----------------------------------------------------------------------
  connect(pump.ports[1], evaporator.port_a)
    annotation (Line(points={{-150,-30},{-120,-30}}, color={0,127,255},
      smooth=Smooth.None));
  connect(evaporator.port_b, massFlowRate.port_a)
    annotation (Line(points={{-100,-30},{-70,-30}}, color={0,127,255},
      smooth=Smooth.None));
  connect(temperature.port, massFlowRate.port_a)
    annotation (Line(points={{-80,-20},{-80,-30},{-70,-30}}, color={0,127,255},
      smooth=Smooth.None));
  connect(upstreamPressure.port, massFlowRate.port_a)
    annotation (Line(points={{-40,-66},{-76,-66},{-76,-30},{-70,-30}},
                                                            color={0,127,255},
      smooth=Smooth.None));
  connect(massFlowRate.port_b, SteamValve.port_a)
    annotation (Line(points={{-50,-30},{2,-30},{2,-80},{50,-80}},
                                                   color={0,127,255},
      smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Unit conversion and observable outputs
  // -----------------------------------------------------------------------
  connect(temperature.T, K2degC.Kelvin)
    annotation (Line(points={{-73,-10},{-68,-10},{-68,18},{-63,18}},
      color={0,0,127}, smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Selector control: setpoints → PI controllers
  //  spUpstream  → ctrlUpstream.SP  : short horizontal, left side
  //  spDownstream → ctrlDownstream.SP : route below all blocks at y=45,
  //                 then up along x=47, outside ctrlDownstream left edge
  // -----------------------------------------------------------------------
  connect(spUpstream.y, ctrlUpstream.SP)
    annotation (Line(points={{-39,49.5},{-26,49.5},{-26,15.8},{-18,15.8}},
      color={0,0,127}, smooth=Smooth.None));
  connect(spDownstream.y, ctrlDownstream.SP)
    annotation (Line(points={{171,19.5},{154.5,19.5},{154.5,16},{138,16}},
      color={0,0,127}, smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Selector control: pressure measurements → PV inputs
  //  upstreamPressure.p   → ctrlUpstream.PV   : left and up, outside block
  //  downstreamPressure.p → ctrlDownstream.PV : straight up to left edge
  // -----------------------------------------------------------------------
  connect(upstreamPressure.p, ctrlUpstream.PV)
    annotation (Line(points={{-29,-56},{-26,-56},{-26,8.45},{-18,8.45}},
      color={0,0,127}, smooth=Smooth.None));
  connect(downstreamPressure.p, ctrlDownstream.PV)
    annotation (Line(points={{111,-40},{148,-40},{148,9},{138,9}},
      color={0,0,127}, smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Selector control: PI outputs → MIN selector
  //  ctrlUpstream.CS  → u1 : direct horizontal (left controller → left input)
  //  ctrlDownstream.CS → u2 : route OVER the top at y=97 to avoid crossing
  // -----------------------------------------------------------------------

  // -----------------------------------------------------------------------
  //  Selected output → steam valve opening
  //  (straight down at x=15, clear of all sensors)
  // -----------------------------------------------------------------------
  connect(minSelector.y, SteamValve.opening)
    annotation (Line(points={{60,-57},{60,-72}},
      color={0,0,127}, smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Tracking switch detection: CS vs minSelector.y → Greater blocks
  //  ctrlUpstream.CS  → greaterUp.u1   : step right of block, up, left
  //  ctrlDownstream.CS → greaterDown.u1 : step right, above top, down left
  // -----------------------------------------------------------------------
  connect(ctrlUpstream.CS, greaterUp.u1)
    annotation (Line(points={{-1,9.5},{10,9.5},{10,78},{-6,78},{-6,70}},
      color={0,0,127}, smooth=Smooth.None));
  connect(ctrlDownstream.CS, greaterDown.u1)
    annotation (Line(points={{121,10},{104,10},{104,78},{126,78},{126,70}},
      color={0,0,127}, smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  minSelector.y → greaterX.u2
  //  Route below all selector blocks at y=62, around outside edges
  // -----------------------------------------------------------------------
  connect(minSelector.y, greaterUp.u2)
    annotation (Line(points={{60,-57},{60,-62},{8,-62},{8,26},{-22,26},{-22,70},
          {-14,70}},
      color={0,0,127}, smooth=Smooth.None));
  connect(minSelector.y, greaterDown.u2)
    annotation (Line(points={{60,-57},{60,-62},{18,-62},{18,74},{118,74},{118,70}},
      color={0,0,127}, smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Tracking reference: minSelector.y → TR ports
  //  Route above block tops at y=96, then tool drops to TR port
  // -----------------------------------------------------------------------
  connect(minSelector.y, ctrlUpstream.TR)
    annotation (Line(points={{60,-57},{60,-62},{8,-62},{8,26},{-10,26},{-10,17.9}},
      color={0,0,127}, smooth=Smooth.None));
  connect(minSelector.y, ctrlDownstream.TR)
    annotation (Line(points={{60,-57},{60,-62},{28,-62},{28,30},{130,30},{130,18}},
      color={0,0,127}, smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Tracking switch: greaterX.y → TS ports (short drop from Greater output)
  // -----------------------------------------------------------------------
  connect(greaterUp.y, ctrlUpstream.TS)
    annotation (Line(points={{-6,47},{-6,17.9}},
      color={255,0,255}, smooth=Smooth.None));
  connect(greaterDown.y, ctrlDownstream.TS)
    annotation (Line(points={{126,47},{126,18}},
      color={255,0,255}, smooth=Smooth.None));

  connect(ctrlDownstream.CS, minSelector.u1)
    annotation (Line(points={{121,10},{66,10},{66,-34}}, color={0,0,127}));
  connect(ctrlUpstream.CS, minSelector.u2)
    annotation (Line(points={{-1,9.5},{54,9.5},{54,-34}}, color={0,0,127}));
  connect(SteamValve.port_b, volume.ports[1]) annotation (Line(points={{70,-80},
          {118.667,-80},{118.667,-70}}, color={0,127,255}));
  connect(downstreamPressure.port, volume.ports[2]) annotation (Line(points={{
          100,-50},{100,-80},{120,-80},{120,-70}}, color={0,127,255}));
  connect(pipe.port_b, sink.ports[1])
    annotation (Line(points={{162,-80},{180,-80}}, color={0,127,255}));
  connect(pipe.port_a, volume.ports[3]) annotation (Line(points={{142,-80},{122,
          -80},{122,-76},{121.333,-76},{121.333,-70}}, color={0,127,255}));
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-200,-120},{200,170}})),
    experiment(StopTime=5400, Tolerance=1e-6),
    experimentSetupOutput,
    Documentation(info="
  <HTML>
  <h4>Selector (Override / Min-Select) Pressure Control on the Drum Boiler</h4>
  <p>
  This example demonstrates <b>selector control</b> (also called
  <em>override control</em> or <em>min-select control</em>) applied to the
  steam valve of the Astrom-Bell equilibrium drum-boiler plant from
  <code>Modelica.Fluid.Examples.DrumBoiler</code>.
  </p>

  <h4>Plant</h4>
  <p>
  The plant is the <code>EquilibriumDrumBoiler</code> model. A prescribed heat
  flow ramps from 0 to 400&nbsp;MW over the first 3600&nbsp;s, simulating boiler
  start-up. Feedwater is supplied by a mass-flow source whose flow rate is
  controlled by the original drum-level PI loop (retained unchanged from the
  MSL DrumBoiler example, using <code>Modelica.Blocks.Continuous.PI</code>).
  </p>

  <h4>Control problem</h4>
  <p>
  The steam valve position must satisfy two competing requirements:
  </p>
  <ol>
    <li><b>Upstream pressure protection</b> &mdash; the drum / steam-header
        pressure must not exceed a ceiling (default: 120&nbsp;bar). When pressure
        rises above this limit, the valve must open to release steam.
        This is a <em>safety constraint</em> that must always be enforced.</li>
    <li><b>Downstream delivery pressure</b> &mdash; the steam header on the
        consumer side should be maintained at a target (default: 1&nbsp;bar).
        When consumer demand exceeds supply, the valve should open more.
        This is a <em>performance objective</em>.</li>
  </ol>
  <p>
  Both objectives are handled by separate ICS PI controllers.
  The <b>MIN selector</b> (<code>Modelica.Blocks.Math.Min</code>) always
  passes the <em>smaller</em> of the two controller outputs to the valve:
  whichever controller demands a <em>less open</em> valve wins.  The upstream
  pressure-protection controller can always veto a larger opening demanded by
  the downstream controller, regardless of consumer demand.
  </p>

  <h4>Control architecture</h4>
  <pre>
  upstreamPressure.p  --&gt; ctrlUpstream   --+
                                            +-&gt; MIN selector --&gt; SteamValve.opening
  downstreamPressure.p --&gt; ctrlDownstream --+

  minSelector.y      --&gt; ctrlUpstream.TR    (tracking reference, bumpless transfer)
  CS_up &gt; min        --&gt; ctrlUpstream.TS    (tracking switch: true = not selected)

  minSelector.y      --&gt; ctrlDownstream.TR
  CS_down &gt; min      --&gt; ctrlDownstream.TS
  </pre>

  <h4>Anti-windup via tracking mode</h4>
  <p>
  Whenever a controller is not selected (its output exceeds the minimum),
  its integrator must not continue to wind up freely. The ICS PI controller
  prevents this via its <em>tracking mode</em>: when <code>TS&nbsp;=&nbsp;true</code>
  the output is forced to follow the reference <code>TR</code> instead of
  integrating the error. In this model both controllers receive
  <code>TR&nbsp;=&nbsp;minSelector.y</code> (the actual valve position), and
  <code>TS</code> is set by a <code>Modelica.Blocks.Logical.Greater</code>
  block that detects <code>controller.CS&nbsp;&gt;&nbsp;minSelector.y</code>.
  The non-active controller therefore tracks the actual valve command and
  is ready for bumpless transfer the moment conditions change.
  </p>

  <h4>Suggested plot variables</h4>
  <ul>
    <li><code>p_S</code> &mdash; upstream steam pressure [bar]</li>
    <li><code>downstreamPressure.p</code> &mdash; downstream pressure [Pa]</li>
    <li><code>SteamValve.opening</code> &mdash; actual valve position [0,&nbsp;1]</li>
    <li><code>ctrlUpstream.CS</code> &mdash; upstream controller demand</li>
    <li><code>ctrlDownstream.CS</code> &mdash; downstream controller demand</li>
    <li><code>minSelector.y</code> &mdash; selected (winning) valve command</li>
    <li><code>V_l</code> &mdash; drum liquid volume [m&sup3;]</li>
    <li><code>qm_S</code> &mdash; steam mass flow rate [kg/s]</li>
  </ul>

  <h4>Parameter guidance</h4>
  <p>
  The default controller gains are conservative starting values for an exploratory
  simulation and will require tuning. For <code>ctrlUpstream</code>,
  <code>Kp&nbsp;=&nbsp;1e-7</code> gives roughly 10&nbsp;% valve opening per
  1&nbsp;MPa pressure error; <code>Ti&nbsp;=&nbsp;600&nbsp;s</code> is slow
  enough not to interact with the drum thermal dynamics. For
  <code>ctrlDownstream</code>, twice the gain and half the integral time reflect
  the smaller and faster downstream volume.
  </p>

  <h4>References</h4>
  <dl>
    <dt>Astrom, Bell:</dt>
    <dd><em>Drum-boiler dynamics</em>, Automatica 36, 2000, pp.&nbsp;363&ndash;378.</dd>
    <dt>Franke, Rode, Krueger:</dt>
    <dd><em>On-line Optimization of Drum Boiler Startup</em>,
        3rd International Modelica Conference, Link&ouml;ping, 2003.</dd>
    <dt>Shinskey, F.G.:</dt>
    <dd><em>Process Control Systems</em>, 4th&nbsp;ed., McGraw-Hill, 1996.
        Chapter&nbsp;9: Override and selector control.</dd>
  </dl>
  </HTML>",
    revisions="<html>
<dl><dt>Industrial Control Systems : 2026</dt>
<dl><dt>List of revisions:</dt>
<p><ul>
<li>March 2026 (author: H. Tummescheit)</li>
</ul></p>
</dl></html>"));
end SelectorControl;
