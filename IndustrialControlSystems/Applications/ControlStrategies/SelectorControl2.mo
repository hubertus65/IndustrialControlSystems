within IndustrialControlSystems.Applications.ControlStrategies;
model SelectorControl2
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
    p_start        = 1e5)
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

  Modelica.Blocks.Sources.TimeTable q_F_Tab(
    table=[0, 0; 3600, 400; 7210, 400])
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

  Modelica.Fluid.Sensors.Pressure upstreamPressure(
    redeclare package Medium = Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{-36,-10},{-16,10}})));

  Modelica.Fluid.Sensors.Pressure downstreamPressure(
    redeclare package Medium = Modelica.Media.Water.StandardWater)
    annotation (Placement(transformation(extent={{162,-10},{142,10}})));

  // -----------------------------------------------------------------------
  //  Steam valve (opening driven by min-selector output)
  // -----------------------------------------------------------------------
  Modelica.Fluid.Valves.ValveLinear SteamValve(
    redeclare package Medium = Modelica.Media.Water.StandardWater,
    dp_nominal     = 9000000,
    m_flow_nominal = 180)
    annotation (Placement(transformation(extent={{40,-40},{60,-20}})));

  // -----------------------------------------------------------------------
  //  Downstream boundary (steam consumer / condenser at 0.5 bar)
  // -----------------------------------------------------------------------
  Modelica.Fluid.Sources.FixedBoundary sink(
    nPorts = 1,
    p      = 50000,
    T      = 500,
    redeclare package Medium = Modelica.Media.Water.StandardWaterOnePhase)
    annotation (Placement(transformation(
        origin={194,-30},
        extent={{10,-10},{-10,10}})));

  // -----------------------------------------------------------------------
  //  Unit conversion for observable outputs
  // -----------------------------------------------------------------------
  Modelica.Blocks.Math.Gain Pa2bar(k=1e-5)
    annotation (Placement(transformation(extent={{106,50},{126,70}})));

  Modelica.Thermal.HeatTransfer.Celsius.FromKelvin K2degC
    annotation (Placement(transformation(extent={{-20,80},{0,100}})));

  Modelica.Blocks.Interfaces.RealOutput T_S(final unit="degC")
    "Steam temperature"
    annotation (Placement(transformation(extent={{160,80},{180,100}})));
  Modelica.Blocks.Interfaces.RealOutput p_S(final unit="bar")
    "Steam pressure upstream of valve [bar]"
    annotation (Placement(transformation(extent={{160,50},{180,70}})));
  Modelica.Blocks.Interfaces.RealOutput qm_S(unit="kg/s")
    "Steam mass flow rate"
    annotation (Placement(transformation(extent={{160,20},{180,40}})));
  Modelica.Blocks.Interfaces.RealOutput V_l(unit="m3")
    "Drum liquid volume"
    annotation (Placement(transformation(extent={{160,110},{180,130}})));

  // -----------------------------------------------------------------------
  //  Selector control — setpoints
  // -----------------------------------------------------------------------
  parameter Real p_SP_upstream   = 120e5
    "Upstream (drum) pressure ceiling [Pa] — overpressure protection setpoint"
    annotation (Dialog(group = "Selector control setpoints"));
  parameter Real p_SP_downstream = 1e5
    "Downstream delivery pressure setpoint [Pa]"
    annotation (Dialog(group = "Selector control setpoints"));

  Modelica.Blocks.Sources.Constant spUpstream(k=p_SP_upstream)
    annotation (Placement(transformation(extent={{-40,42},{-20,62}})));

  Modelica.Blocks.Sources.Constant spDownstream(k=p_SP_downstream)
    annotation (Placement(transformation(extent={{198,6},{178,26}})));

  // -----------------------------------------------------------------------
  //  Selector control — ICS PI controllers (both with tracking enabled)
  //
  //  ctrlUpstream   : PV = upstream pressure [Pa], SP = p_SP_upstream
  //    When PV > SP (overpressure): error < 0 → CS decreases → valve closes
  //    → drum pressure protected. Acts as a pressure-ceiling governor.
  //
  //  ctrlDownstream : PV = downstream pressure [Pa], SP = p_SP_downstream
  //    When PV < SP (demand exceeds supply): error > 0 → CS increases
  //    → valve opens → more steam delivered to consumer.
  //
  //  The MIN selector passes whichever output is smaller to the valve:
  //  the upstream protection controller can always veto a larger opening
  //  demanded by the downstream controller.
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
    annotation (Placement(transformation(extent={{0,0},{20,20}})));

  IndustrialControlSystems.Controllers.PI ctrlDownstream(
    useTS      = true,
    AntiWindup = true,
    CSmin      = 0,
    CSmax      = 1,
    CS_start   = 0,
    Kp         = 2e-7,
    Ti         = 300,
    Ts         = 0)
    annotation (Placement(transformation(extent={{114,0},{94,20}})));

  // -----------------------------------------------------------------------
  //  Selector control — MIN selector
  // -----------------------------------------------------------------------
  Modelica.Blocks.Math.Min minSelector
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={50,6})));

  // -----------------------------------------------------------------------
  //  Tracking switch logic
  //  greaterUp   : true when ctrlUpstream.CS   > minSelector.y  (not selected)
  //  greaterDown : true when ctrlDownstream.CS > minSelector.y  (not selected)
  // -----------------------------------------------------------------------

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
    annotation (Line(points={{-26,-10},{-26,-48},{-80,-48},{-80,-30},{-70,-30}},
                                                            color={0,127,255},
      smooth=Smooth.None));
  connect(massFlowRate.port_b, SteamValve.port_a)
    annotation (Line(points={{-50,-30},{40,-30}},  color={0,127,255},
      smooth=Smooth.None));
  connect(SteamValve.port_b, downstreamPressure.port)
    annotation (Line(points={{60,-30},{152,-30},{152,-10}},
                                                         color={0,127,255},
      smooth=Smooth.None));
  connect(SteamValve.port_b, sink.ports[1])
    annotation (Line(points={{60,-30},{184,-30}},color={0,127,255},
      smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Unit conversion and observable outputs
  // -----------------------------------------------------------------------
  connect(upstreamPressure.p, Pa2bar.u)
    annotation (Line(points={{-15,0},{-10,0},{-10,60},{104,60}},
      color={0,0,127}, smooth=Smooth.None));
  connect(Pa2bar.y, p_S)
    annotation (Line(points={{127,60},{170,60}},
                                               color={0,0,127},
      smooth=Smooth.None));
  connect(temperature.T, K2degC.Kelvin)
    annotation (Line(points={{-73,-10},{-50,-10},{-50,90},{-22,90}},
      color={0,0,127}, smooth=Smooth.None));
  connect(K2degC.Celsius, T_S)
    annotation (Line(points={{1,90},{170,90}}, color={0,0,127},
      smooth=Smooth.None));
  connect(massFlowRate.m_flow, qm_S)
    annotation (Line(points={{-60,-19},{-60,30},{170,30}},
      color={0,0,127}, smooth=Smooth.None));
  connect(evaporator.V, V_l)
    annotation (Line(points={{-106,-19},{-106,120},{170,120}},
      color={0,0,127}, smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Selector control: setpoints to PI controllers
  // -----------------------------------------------------------------------

  // -----------------------------------------------------------------------
  //  Selector control: PV measurements to PI controllers
  // -----------------------------------------------------------------------
  connect(downstreamPressure.p, ctrlDownstream.PV)
    annotation (Line(points={{141,0},{122,0},{122,9},{112,9}},
                  color={0,0,127}, smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Selector control: PI outputs to MIN selector
  // -----------------------------------------------------------------------

  // -----------------------------------------------------------------------
  //  Selector control: selected output to steam valve
  // -----------------------------------------------------------------------
  connect(minSelector.y, SteamValve.opening)
    annotation (Line(points={{50,-5},{50,-22}},
      color={0,0,127}, smooth=Smooth.None));

  // -----------------------------------------------------------------------
  //  Tracking switch detection
  // -----------------------------------------------------------------------

  // -----------------------------------------------------------------------
  //  Tracking references and switches back to PI controllers
  //  TR = minSelector.y  (the actual valve position — value to track)
  //  TS = greaterX.y     (true when this controller is NOT the active one)
  // -----------------------------------------------------------------------
  connect(minSelector.y,  ctrlDownstream.TR)
    annotation (Line(points={{50,-5},{50,-10},{86,-10},{86,28},{104,28},{104,18}},
      color={0,0,127}, smooth=Smooth.None));

  connect(spDownstream.y, ctrlDownstream.SP)
    annotation (Line(points={{177,16},{112,16}}, color={0,0,127}));
  connect(upstreamPressure.p, ctrlUpstream.PV)
    annotation (Line(points={{-15,0},{-10,0},{-10,9},{2,9}}, color={0,0,127}));
  connect(ctrlUpstream.CS, minSelector.u2) annotation (Line(points={{19,10},{34,
          10},{34,26},{44,26},{44,18}}, color={0,0,127}));
  connect(ctrlDownstream.CS, minSelector.u1) annotation (Line(points={{95,10},{
          66,10},{66,26},{56,26},{56,18}}, color={0,0,127}));
  connect(ctrlUpstream.TR, minSelector.y) annotation (Line(points={{10,18},{10,
          28},{32,28},{32,-10},{50,-10},{50,-5}}, color={0,0,127}));
  connect(spUpstream.y, ctrlUpstream.SP) annotation (Line(points={{-19,52},{-8,
          52},{-8,16},{2,16}}, color={0,0,127}));
  annotation (
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-200,-120},{200,200}})),
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
end SelectorControl2;
