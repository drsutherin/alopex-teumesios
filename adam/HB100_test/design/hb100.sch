<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="6.1">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="yes" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="yes" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="yes" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="yes" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="yes" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="yes" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="yes" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="yes" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="yes" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="yes" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="yes" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="yes" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="yes" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="yes" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="yes" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="yes" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="yes" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="yes" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="yes" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="yes" active="no"/>
<layer number="50" name="dxf" color="7" fill="1" visible="yes" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="yes" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="yes" active="no"/>
<layer number="53" name="tGND_GNDA" color="7" fill="1" visible="yes" active="no"/>
<layer number="54" name="bGND_GNDA" color="7" fill="1" visible="yes" active="no"/>
<layer number="56" name="wert" color="7" fill="1" visible="yes" active="no"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="100" name="PaJa" color="12" fill="7" visible="no" active="yes"/>
<layer number="101" name="Doplnky" color="5" fill="1" visible="yes" active="yes"/>
<layer number="102" name="Kola" color="11" fill="7" visible="yes" active="yes"/>
<layer number="103" name="Popisy" color="2" fill="8" visible="yes" active="yes"/>
<layer number="104" name="Zapojeni" color="6" fill="7" visible="yes" active="yes"/>
<layer number="110" name="wago-seda" color="7" fill="8" visible="yes" active="yes"/>
<layer number="111" name="wago-cervena" color="12" fill="8" visible="yes" active="yes"/>
<layer number="112" name="wago-zelena" color="2" fill="8" visible="yes" active="yes"/>
<layer number="113" name="wago-modra" color="1" fill="8" visible="yes" active="yes"/>
<layer number="151" name="HeatSink" color="7" fill="1" visible="no" active="no"/>
<layer number="200" name="200bmp" color="1" fill="10" visible="no" active="no"/>
<layer number="250" name="Descript" color="3" fill="1" visible="no" active="no"/>
<layer number="251" name="SMDround" color="12" fill="11" visible="no" active="no"/>
<layer number="254" name="OrgLBR" color="13" fill="1" visible="no" active="no"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="#PaJa_konektory">
<description>&lt;B&gt;PaJa_konektory&lt;/B&gt; - knihovna   &amp;nbsp; &amp;nbsp; &amp;nbsp; &amp;nbsp; &amp;nbsp; &amp;nbsp; 
&lt;I&gt;(vytvoreno 1.6.2011)&lt;/I&gt;&lt;BR&gt;
Knihovna konektoru do Eagle &lt;I&gt;(od verze 5.6)&lt;/I&gt;&lt;BR&gt;
&lt;BR&gt;
Knihovna obsahuje: 91 soucastek na DPS, 92 do SCHematu&lt;BR&gt;
&lt;BR&gt;
&lt;Author&gt;Copyright (C) PaJa 2011&lt;BR&gt;
http://www.paja-trb.unas.cz&lt;BR&gt;
paja-trb@seznam.cz
&lt;/author&gt;</description>
<packages>
<package name="S1G3_JUM">
<wire x1="-1.27" y1="1.016" x2="-1.016" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.27" y1="1.016" x2="-1.524" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.27" y1="-1.016" x2="-1.016" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.27" y1="-1.016" x2="-1.524" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.524" y1="-1.27" x2="-3.556" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-3.81" y1="-1.016" x2="-3.556" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.016" y1="-1.27" x2="1.27" y2="-1.016" width="0.127" layer="21"/>
<wire x1="1.016" y1="-1.27" x2="-1.016" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.016" y1="1.27" x2="1.27" y2="1.016" width="0.127" layer="21"/>
<wire x1="1.016" y1="1.27" x2="-1.016" y2="1.27" width="0.127" layer="21"/>
<wire x1="-1.524" y1="1.27" x2="-3.556" y2="1.27" width="0.127" layer="21"/>
<wire x1="-3.81" y1="1.016" x2="-3.556" y2="1.27" width="0.127" layer="21"/>
<wire x1="-3.81" y1="1.016" x2="-3.81" y2="-1.016" width="0.127" layer="21"/>
<wire x1="-1.27" y1="0.954" x2="-1.27" y2="-0.954" width="0.127" layer="21"/>
<wire x1="3.818" y1="1.016" x2="3.564" y2="1.27" width="0.127" layer="21"/>
<wire x1="3.818" y1="-1.016" x2="3.564" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.564" y1="-1.27" x2="1.532" y2="-1.27" width="0.127" layer="21"/>
<wire x1="1.278" y1="-1.016" x2="1.532" y2="-1.27" width="0.127" layer="21"/>
<wire x1="3.564" y1="1.27" x2="1.532" y2="1.27" width="0.127" layer="21"/>
<wire x1="1.278" y1="1.016" x2="1.532" y2="1.27" width="0.127" layer="21"/>
<wire x1="1.278" y1="1.016" x2="1.278" y2="-1.016" width="0.127" layer="21"/>
<wire x1="3.818" y1="0.954" x2="3.818" y2="-0.954" width="0.127" layer="21"/>
<circle x="-2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="0" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="2.54" y="0" radius="0.5724" width="0.127" layer="102"/>
<pad name="1" x="-2.54" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="2" x="0" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="3" x="2.54" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<text x="-2.856" y="1.492" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.333" y="-2.762" size="1.27" layer="27">&gt;VALUE</text>
<text x="1.115" y="-0.954" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="-0.3048" y1="-0.3048" x2="0.3048" y2="0.3048" layer="51"/>
<rectangle x1="-2.8448" y1="-0.3048" x2="-2.2352" y2="0.3048" layer="51"/>
<rectangle x1="2.2432" y1="-0.3048" x2="2.8528" y2="0.3048" layer="51"/>
</package>
<package name="S1G2_JUM">
<wire x1="0" y1="1.016" x2="0.254" y2="1.27" width="0.127" layer="21"/>
<wire x1="0" y1="1.016" x2="-0.254" y2="1.27" width="0.127" layer="21"/>
<wire x1="0" y1="-1.016" x2="0.254" y2="-1.27" width="0.127" layer="21"/>
<wire x1="0" y1="-1.016" x2="-0.254" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.27" x2="-2.286" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-2.54" y1="-1.016" x2="-2.286" y2="-1.27" width="0.127" layer="21"/>
<wire x1="2.286" y1="-1.27" x2="2.54" y2="-1.016" width="0.127" layer="21"/>
<wire x1="2.286" y1="-1.27" x2="0.254" y2="-1.27" width="0.127" layer="21"/>
<wire x1="2.54" y1="-1.016" x2="2.54" y2="1.016" width="0.127" layer="21"/>
<wire x1="2.286" y1="1.27" x2="2.54" y2="1.016" width="0.127" layer="21"/>
<wire x1="2.286" y1="1.27" x2="0.254" y2="1.27" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.27" x2="-2.286" y2="1.27" width="0.127" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="-2.286" y2="1.27" width="0.127" layer="21"/>
<wire x1="-2.54" y1="1.016" x2="-2.54" y2="-1.016" width="0.127" layer="21"/>
<wire x1="0" y1="0.954" x2="0" y2="-0.954" width="0.127" layer="21"/>
<circle x="-1.27" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="1.27" y="0" radius="0.5724" width="0.127" layer="102"/>
<pad name="1" x="-1.27" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<pad name="2" x="1.27" y="0" drill="1.016" diameter="1.778" shape="octagon" rot="R90"/>
<text x="-2.54" y="1.492" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-2.762" size="1.27" layer="27">&gt;VALUE</text>
<text x="-0.159" y="-0.954" size="0.254" layer="100" rot="R90">PaJa</text>
<rectangle x1="0.9652" y1="-0.3048" x2="1.5748" y2="0.3048" layer="51"/>
<rectangle x1="-1.5748" y1="-0.3048" x2="-0.9652" y2="0.3048" layer="51"/>
</package>
</packages>
<symbols>
<symbol name="S1G3">
<wire x1="-4.445" y1="1.27" x2="-4.445" y2="5.715" width="0.4064" layer="94"/>
<wire x1="2.54" y1="3.81" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="0" y1="3.81" x2="0" y2="2.54" width="0.6096" layer="94"/>
<wire x1="-2.54" y1="3.81" x2="-2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="4.445" y1="5.715" x2="-4.445" y2="5.715" width="0.4064" layer="94"/>
<wire x1="-4.445" y1="1.27" x2="4.445" y2="1.27" width="0.4064" layer="94"/>
<wire x1="4.445" y1="5.715" x2="4.445" y2="1.27" width="0.4064" layer="94"/>
<text x="6.8262" y="0.1587" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<text x="-5.08" y="0.4762" size="1.778" layer="95" rot="R90">&gt;Part</text>
<text x="4.1275" y="1.5875" size="0.254" layer="100" rot="R90">PaJa</text>
<pin name="1" x="-2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="2" x="0" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="3" x="2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
<symbol name="S1G2">
<wire x1="-1.905" y1="1.27" x2="-1.905" y2="5.715" width="0.4064" layer="94"/>
<wire x1="2.54" y1="3.81" x2="2.54" y2="2.54" width="0.6096" layer="94"/>
<wire x1="0" y1="3.81" x2="0" y2="2.54" width="0.6096" layer="94"/>
<wire x1="4.445" y1="5.715" x2="-1.905" y2="5.715" width="0.4064" layer="94"/>
<wire x1="-1.905" y1="1.27" x2="4.445" y2="1.27" width="0.4064" layer="94"/>
<wire x1="4.445" y1="5.715" x2="4.445" y2="1.27" width="0.4064" layer="94"/>
<text x="6.8262" y="0.1587" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<text x="-2.54" y="0.4762" size="1.778" layer="95" rot="R90">&gt;Part</text>
<text x="4.1275" y="1.5875" size="0.254" layer="100" rot="R90">PaJa</text>
<pin name="1" x="0" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
<pin name="2" x="2.54" y="-2.54" visible="pad" length="middle" direction="pas" swaplevel="1" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="S1G3_JUMP" prefix="JUM">
<description>&lt;B&gt;Radove konektory&lt;/B&gt; - koliky - 3x</description>
<gates>
<gate name="JUMP" symbol="S1G3" x="-43.18" y="33.02"/>
</gates>
<devices>
<device name="" package="S1G3_JUM">
<connects>
<connect gate="JUMP" pin="1" pad="1"/>
<connect gate="JUMP" pin="2" pad="2"/>
<connect gate="JUMP" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="S1G2_JUMP" prefix="JUM">
<description>&lt;B&gt;Radove konektory&lt;/B&gt; - koliky - 2x</description>
<gates>
<gate name="JUMP" symbol="S1G2" x="-40.64" y="40.64"/>
</gates>
<devices>
<device name="" package="S1G2_JUM">
<connects>
<connect gate="JUMP" pin="1" pad="1"/>
<connect gate="JUMP" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="#PaJa_30">
<description>&lt;B&gt;PaJa 30&lt;/B&gt; - knihovna   &amp;nbsp; &amp;nbsp; &amp;nbsp; &amp;nbsp; &amp;nbsp; &amp;nbsp; 
&lt;I&gt;(vytvoreno 1.6.2011)&lt;/I&gt;&lt;BR&gt;
Univerzální knihovna soucastek do Eagle &lt;I&gt;(od verze 5.6)&lt;/I&gt;&lt;BR&gt;
&lt;BR&gt;
Knihovna obsahuje: 196 soucastek na DPS, 298 do SCHematu&lt;BR&gt;
&lt;BR&gt;
&lt;Author&gt;Copyright (C) PaJa 2001-2011&lt;BR&gt;
http://www.paja-trb.unas.cz&lt;BR&gt;
paja-trb@seznam.cz
&lt;/author&gt;</description>
<packages>
<package name="DIL8">
<wire x1="-5.079" y1="-0.635" x2="-5.079" y2="0.635" width="0.127" layer="21" curve="180" cap="flat"/>
<wire x1="3.81" y1="3.3338" x2="3.81" y2="3.175" width="0.127" layer="102"/>
<wire x1="-3.81" y1="3.3338" x2="-3.81" y2="3.175" width="0.127" layer="102"/>
<wire x1="-1.27" y1="3.3338" x2="-1.27" y2="3.175" width="0.127" layer="102"/>
<wire x1="1.27" y1="3.3338" x2="1.27" y2="3.175" width="0.127" layer="102"/>
<wire x1="-3.81" y1="-3.3338" x2="-3.81" y2="-3.175" width="0.127" layer="102"/>
<wire x1="3.81" y1="-3.3338" x2="3.81" y2="-3.175" width="0.127" layer="102"/>
<wire x1="1.27" y1="-3.3338" x2="1.27" y2="-3.175" width="0.127" layer="102"/>
<wire x1="-1.27" y1="-3.3338" x2="-1.27" y2="-3.175" width="0.127" layer="102"/>
<wire x1="-5.08" y1="3.175" x2="-5.08" y2="0.635" width="0.127" layer="21"/>
<wire x1="-5.08" y1="-0.635" x2="-5.08" y2="-3.175" width="0.127" layer="21"/>
<wire x1="-5.08" y1="3.175" x2="5.08" y2="3.175" width="0.127" layer="21"/>
<wire x1="5.08" y1="3.175" x2="5.08" y2="-3.175" width="0.127" layer="21"/>
<wire x1="5.08" y1="-3.175" x2="-5.08" y2="-3.175" width="0.127" layer="21"/>
<circle x="3.81" y="3.81" radius="0.4763" width="0.127" layer="102"/>
<circle x="-3.81" y="3.81" radius="0.4763" width="0.127" layer="102"/>
<circle x="-1.27" y="3.81" radius="0.4763" width="0.127" layer="102"/>
<circle x="1.27" y="3.81" radius="0.4763" width="0.127" layer="102"/>
<circle x="-3.81" y="-3.81" radius="0.4763" width="0.127" layer="102"/>
<circle x="3.81" y="-3.81" radius="0.4763" width="0.127" layer="102"/>
<circle x="1.27" y="-3.81" radius="0.4763" width="0.127" layer="102"/>
<circle x="-1.27" y="-3.81" radius="0.4763" width="0.127" layer="102"/>
<pad name="1" x="-3.81" y="-3.81" drill="0.8128" diameter="1.4" shape="long" rot="R90"/>
<pad name="2" x="-1.27" y="-3.81" drill="0.8128" diameter="1.4" shape="long" rot="R90"/>
<pad name="3" x="1.27" y="-3.81" drill="0.8128" diameter="1.4" shape="long" rot="R90"/>
<pad name="4" x="3.81" y="-3.81" drill="0.8128" diameter="1.4" shape="long" rot="R90"/>
<pad name="5" x="3.81" y="3.81" drill="0.8128" diameter="1.4" shape="long" rot="R90"/>
<pad name="6" x="1.27" y="3.81" drill="0.8128" diameter="1.4" shape="long" rot="R90"/>
<pad name="7" x="-1.27" y="3.81" drill="0.8128" diameter="1.4" shape="long" rot="R90"/>
<pad name="8" x="-3.81" y="3.81" drill="0.8128" diameter="1.4" shape="long" rot="R90"/>
<text x="-3.975" y="-2.385" size="1.016" layer="101">1</text>
<text x="-4.134" y="1.431" size="1.016" layer="101">8</text>
<text x="3.339" y="-2.385" size="1.016" layer="101">4</text>
<text x="3.498" y="1.431" size="1.016" layer="101">5</text>
<text x="4.929" y="-0.636" size="0.254" layer="100" rot="R90">PaJa</text>
<text x="-2.2305" y="0.5974" size="1.4224" layer="25">&gt;Name</text>
<text x="-3.5015" y="-1.1486" size="1.4224" layer="27">&gt;Value</text>
</package>
<package name="C-EL_2">
<wire x1="-0.1378" y1="-1.02" x2="-0.1378" y2="-1.782" width="0.127" layer="21"/>
<wire x1="-0.5188" y1="-1.401" x2="0.2432" y2="-1.401" width="0.127" layer="21"/>
<wire x1="0.4762" y1="-1.524" x2="0.9842" y2="-1.524" width="0.127" layer="21"/>
<wire x1="0.9842" y1="-1.524" x2="0.9842" y2="1.524" width="0.127" layer="21"/>
<wire x1="0.9842" y1="1.524" x2="0.4762" y2="1.524" width="0.127" layer="21"/>
<wire x1="0.4762" y1="1.524" x2="0.4762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="1.4112" y1="1.5" x2="1.7862" y2="1.5" width="0.127" layer="21"/>
<wire x1="1.7862" y1="1.5" x2="1.7862" y2="-1.5" width="0.127" layer="21"/>
<wire x1="1.7862" y1="-1.5" x2="1.4112" y2="-1.5" width="0.127" layer="21"/>
<wire x1="1.4112" y1="-1.5" x2="1.4112" y2="1.5" width="0.127" layer="21"/>
<wire x1="1.5762" y1="0" x2="2.1962" y2="0" width="0.127" layer="21"/>
<wire x1="0.0262" y1="0" x2="0.2712" y2="0" width="0.127" layer="21"/>
<wire x1="0.2712" y1="0" x2="0.3362" y2="0" width="0.127" layer="21"/>
<wire x1="0.2712" y1="0" x2="0.439" y2="0" width="0.127" layer="21"/>
<circle x="1.1112" y="0" radius="2.4848" width="0.127" layer="21"/>
<circle x="-0.0018" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="2.2242" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="C+" x="0" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<pad name="C-" x="2.2225" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<text x="-1.2738" y="2.544" size="1.27" layer="25" font="vector">&gt;Name</text>
<text x="-1.5918" y="-3.816" size="1.27" layer="27" font="vector">&gt;Value</text>
<text x="0.6342" y="1.908" size="0.254" layer="100" font="vector">PaJa</text>
<rectangle x1="1.4112" y1="-1.5" x2="1.7862" y2="1.425" layer="21"/>
</package>
<package name="C-EL_2,5">
<wire x1="-0.635" y1="-1.524" x2="-0.127" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.127" y1="-1.524" x2="-0.127" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.127" y1="1.524" x2="-0.635" y2="1.524" width="0.127" layer="21"/>
<wire x1="-1.868" y1="-0.861" x2="-1.868" y2="-1.623" width="0.127" layer="21"/>
<wire x1="-2.249" y1="-1.242" x2="-1.487" y2="-1.242" width="0.127" layer="21"/>
<wire x1="-0.635" y1="1.524" x2="-0.635" y2="-1.524" width="0.127" layer="21"/>
<wire x1="0.3" y1="1.5" x2="0.675" y2="1.5" width="0.127" layer="21"/>
<wire x1="0.675" y1="1.5" x2="0.675" y2="-1.5" width="0.127" layer="21"/>
<wire x1="0.675" y1="-1.5" x2="0.3" y2="-1.5" width="0.127" layer="21"/>
<wire x1="0.3" y1="-1.5" x2="0.3" y2="1.5" width="0.127" layer="21"/>
<wire x1="0.62" y1="0" x2="1.24" y2="0" width="0.127" layer="21"/>
<wire x1="-1.24" y1="0" x2="-0.8146" y2="0" width="0.127" layer="21"/>
<wire x1="-0.8146" y1="0" x2="-0.775" y2="0" width="0.127" layer="21"/>
<wire x1="-0.8146" y1="0" x2="-0.6758" y2="0" width="0.127" layer="21"/>
<circle x="0" y="0" radius="3.255" width="0.127" layer="21"/>
<circle x="-1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="C+" x="-1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<pad name="C-" x="1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<text x="-1.431" y="1.59" size="1.27" layer="25" font="vector">&gt;Name</text>
<text x="-1.272" y="-2.862" size="1.27" layer="27" font="vector">&gt;Value</text>
<text x="-0.2447" y="-0.4478" size="0.254" layer="100" font="vector" rot="R90">PaJa</text>
<rectangle x1="0.3" y1="-1.5" x2="0.675" y2="1.425" layer="21"/>
</package>
<package name="C-EL_3,5">
<wire x1="1.1113" y1="-1.524" x2="1.6193" y2="-1.524" width="0.127" layer="21"/>
<wire x1="1.6193" y1="-1.524" x2="1.6193" y2="1.524" width="0.127" layer="21"/>
<wire x1="1.6193" y1="1.524" x2="1.1113" y2="1.524" width="0.127" layer="21"/>
<wire x1="0.1838" y1="-1.1705" x2="0.1838" y2="-1.9325" width="0.127" layer="21"/>
<wire x1="-0.1972" y1="-1.5515" x2="0.5648" y2="-1.5515" width="0.127" layer="21"/>
<wire x1="1.1113" y1="1.524" x2="1.1113" y2="-1.524" width="0.127" layer="21"/>
<wire x1="2.0463" y1="1.5" x2="2.4213" y2="1.5" width="0.127" layer="21"/>
<wire x1="2.4213" y1="1.5" x2="2.4213" y2="-1.5" width="0.127" layer="21"/>
<wire x1="2.4213" y1="-1.5" x2="2.0463" y2="-1.5" width="0.127" layer="21"/>
<wire x1="2.0463" y1="-1.5" x2="2.0463" y2="1.5" width="0.127" layer="21"/>
<wire x1="2.3623" y1="0" x2="2.9783" y2="0" width="0.127" layer="51"/>
<wire x1="0.4747" y1="0" x2="0.9713" y2="0" width="0.127" layer="21"/>
<wire x1="0.5063" y1="0" x2="1.0705" y2="0" width="0.127" layer="51"/>
<circle x="1.7463" y="0.155" radius="4.03" width="0.127" layer="21"/>
<circle x="-0.0027" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="3.4953" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="C+" x="0" y="0" drill="0.8128" diameter="1.9304"/>
<pad name="C-" x="3.4925" y="0" drill="0.8128" diameter="1.9304" shape="square"/>
<text x="-0.6476" y="-2.9909" size="1.27" layer="27" font="vector">&gt;VALUE</text>
<text x="4.7965" y="3.0261" size="1.27" layer="25" font="vector" rot="R180">&gt;NAME</text>
<text x="2.7003" y="1.113" size="0.254" layer="100" font="vector">PaJa</text>
<rectangle x1="2.0463" y1="-1.5" x2="2.4213" y2="1.425" layer="21"/>
</package>
<package name="C-EL_5">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="0" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="1.2065" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-1.236" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-2.003" y1="0" x2="-1.236" y2="0" width="0.127" layer="51"/>
<wire x1="1.2325" y1="0" x2="2.003" y2="0" width="0.127" layer="51"/>
<wire x1="-1.905" y1="-1.27" x2="-1.27" y2="-1.27" width="0.127" layer="21"/>
<wire x1="-1.5875" y1="-0.9525" x2="-1.5875" y2="-1.5875" width="0.127" layer="21"/>
<circle x="0" y="0" radius="5.1308" width="0.127" layer="21"/>
<circle x="-2.54" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="2.54" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="C-" x="2.54" y="0" drill="0.8128" diameter="2.1844" shape="square"/>
<pad name="C+" x="-2.54" y="0" drill="0.8128" diameter="2.1844"/>
<text x="1.272" y="0.318" size="0.254" layer="100" font="vector" rot="R90">PaJa</text>
<text x="-3.017" y="1.758" size="1.4224" layer="25" font="vector">&gt;Name</text>
<text x="-3.663" y="-3.186" size="1.4224" layer="27" font="vector">&gt;Value</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="C-EL_5+">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="0" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="1.0478" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-1.0478" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-2.003" y1="0" x2="-1.0775" y2="0" width="0.127" layer="51"/>
<wire x1="1.0738" y1="0" x2="2.003" y2="0" width="0.127" layer="51"/>
<wire x1="-2.019" y1="-1.5655" x2="-1.081" y2="-1.5655" width="0.127" layer="21"/>
<wire x1="-1.55" y1="-1.0965" x2="-1.55" y2="-2.0345" width="0.127" layer="21"/>
<circle x="0" y="0" radius="6.519" width="0.127" layer="21"/>
<circle x="-2.544" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="2.544" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="C+" x="-2.54" y="0" drill="0.8128" diameter="2.54"/>
<pad name="C-" x="2.54" y="0" drill="0.8128" diameter="2.54" shape="square"/>
<text x="-4.299" y="-3.663" size="1.6764" layer="27" font="vector">&gt;VALUE</text>
<text x="-3.653" y="1.758" size="1.6764" layer="25" font="vector">&gt;NAME</text>
<text x="1.113" y="0.318" size="0.254" layer="100" font="vector" rot="R90">PaJa</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="C-EL_7,5">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="0" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="2.343" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-2.347" y2="0" width="0.127" layer="21"/>
<wire x1="-0.762" y1="0" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-3.275" y1="0" x2="-2.347" y2="0" width="0.127" layer="51"/>
<wire x1="2.343" y1="0" x2="3.275" y2="0" width="0.127" layer="51"/>
<wire x1="-2.54" y1="-0.7938" x2="-2.54" y2="-1.7463" width="0.127" layer="21"/>
<wire x1="-3.0163" y1="-1.27" x2="-2.0638" y2="-1.27" width="0.127" layer="21"/>
<circle x="0" y="0" radius="8.1339" width="0.127" layer="21"/>
<circle x="-3.816" y="0" radius="0.5732" width="0.127" layer="102"/>
<circle x="3.816" y="0" radius="0.5732" width="0.127" layer="102"/>
<pad name="C-" x="3.81" y="0" drill="1.016" diameter="2.54" shape="square"/>
<pad name="C+" x="-3.81" y="0" drill="1.016" diameter="2.54"/>
<text x="-4.458" y="-3.981" size="1.778" layer="27" font="vector">&gt;VALUE</text>
<text x="-3.812" y="2.076" size="1.778" layer="25" font="vector">&gt;NAME</text>
<text x="1.113" y="0.159" size="0.254" layer="100" font="vector">PaJa</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="C-EL7,5+">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-2.6558" y1="-1.089" x2="-1.7178" y2="-1.089" width="0.127" layer="21"/>
<wire x1="-2.1868" y1="-0.62" x2="-2.1868" y2="-1.558" width="0.127" layer="21"/>
<wire x1="-2.3813" y1="0" x2="-3.175" y2="0" width="0.127" layer="51"/>
<wire x1="2.3813" y1="0" x2="3.175" y2="0" width="0.127" layer="51"/>
<wire x1="-2.3813" y1="0" x2="-0.7938" y2="0" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="2.3813" y2="0" width="0.127" layer="21"/>
<circle x="0" y="0" radius="9.063" width="0.127" layer="21"/>
<circle x="-3.816" y="0" radius="0.5732" width="0.127" layer="102"/>
<circle x="3.816" y="0" radius="0.5732" width="0.127" layer="102"/>
<pad name="C-" x="3.81" y="0" drill="1.016" diameter="2.54" shape="square"/>
<pad name="C+" x="-3.81" y="0" drill="1.016" diameter="2.54"/>
<text x="-4.458" y="-3.981" size="1.778" layer="27" font="vector">&gt;VALUE</text>
<text x="-3.812" y="2.076" size="1.778" layer="25" font="vector">&gt;NAME</text>
<text x="1.113" y="0.159" size="0.254" layer="100" font="vector">PaJa</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="C-EL_10">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="3.3338" y2="0" width="0.127" layer="21"/>
<wire x1="-0.7938" y1="0" x2="-3.3338" y2="0" width="0.127" layer="21"/>
<wire x1="-3.3338" y1="-1.1113" x2="-2.0638" y2="-1.1113" width="0.127" layer="21"/>
<wire x1="-2.6988" y1="-0.4763" x2="-2.6988" y2="-1.7463" width="0.127" layer="21"/>
<wire x1="3.3338" y1="0" x2="4.2863" y2="0" width="0.127" layer="51"/>
<wire x1="-3.3338" y1="0" x2="-4.2863" y2="0" width="0.127" layer="51"/>
<circle x="0" y="0" radius="11.1125" width="0.127" layer="21"/>
<circle x="-5.08" y="0" radius="0.7099" width="0.127" layer="102"/>
<circle x="5.08" y="0" radius="0.7099" width="0.127" layer="102"/>
<pad name="C-" x="5.08" y="0" drill="1.27" diameter="3.2" shape="square"/>
<pad name="C+" x="-5.08" y="0" drill="1.27" diameter="3.2"/>
<text x="-5.728" y="-4.9335" size="2.1844" layer="27" font="vector">&gt;VALUE</text>
<text x="-4.7645" y="3.0285" size="2.1844" layer="25" font="vector">&gt;NAME</text>
<text x="1.113" y="0.159" size="0.254" layer="100" font="vector">PaJa</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="C-EL_10+">
<wire x1="-0.762" y1="-1.524" x2="-0.254" y2="-1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="-1.524" x2="-0.254" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.254" y1="1.524" x2="-0.762" y2="1.524" width="0.127" layer="21"/>
<wire x1="-0.762" y1="1.524" x2="-0.762" y2="-1.524" width="0.127" layer="21"/>
<wire x1="0.635" y1="0" x2="3.1751" y2="0" width="0.127" layer="21"/>
<wire x1="-0.7938" y1="0" x2="-3.175" y2="0" width="0.127" layer="21"/>
<wire x1="-3.3338" y1="-1.1113" x2="-2.0638" y2="-1.1113" width="0.127" layer="21"/>
<wire x1="-2.6988" y1="-0.4763" x2="-2.6988" y2="-1.7463" width="0.127" layer="21"/>
<wire x1="3.175" y1="0" x2="4.2863" y2="0" width="0.127" layer="51"/>
<wire x1="-3.175" y1="0" x2="-4.2863" y2="0" width="0.127" layer="51"/>
<circle x="-5.08" y="0" radius="0.7099" width="0.127" layer="102"/>
<circle x="5.08" y="0" radius="0.7099" width="0.127" layer="102"/>
<circle x="0" y="0" radius="15.24" width="0.254" layer="21"/>
<pad name="C-" x="5.08" y="0" drill="1.27" diameter="3.2" shape="square"/>
<pad name="C+" x="-5.08" y="0" drill="1.27" diameter="3.2"/>
<text x="-5.728" y="-5.886" size="2.1844" layer="27" font="vector">&gt;VALUE</text>
<text x="-4.7645" y="3.0285" size="2.1844" layer="25" font="vector">&gt;NAME</text>
<text x="1.113" y="0.159" size="0.254" layer="100" font="vector">PaJa</text>
<rectangle x1="0.254" y1="-1.524" x2="0.762" y2="1.524" layer="21"/>
</package>
<package name="C-2,5">
<circle x="-1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="1.272" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<pad name="2" x="1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<text x="-1.6704" y="1.3513" size="1.016" layer="25" font="vector">&gt;Name</text>
<text x="-1.6709" y="-2.3853" size="1.016" layer="27" font="vector">&gt;Value</text>
<text x="0.159" y="0.318" size="0.254" layer="100" font="vector" rot="R90">PaJa</text>
<rectangle x1="-0.5556" y1="-1.27" x2="-0.1746" y2="1.27" layer="21"/>
<rectangle x1="0.1746" y1="-1.27" x2="0.5556" y2="1.27" layer="21"/>
<rectangle x1="-0.7938" y1="-0.1588" x2="-0.5556" y2="0.1588" layer="51"/>
<rectangle x1="0.5556" y1="-0.1588" x2="0.7938" y2="0.1588" layer="51"/>
</package>
<package name="C-5">
<circle x="-2.544" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="2.544" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<text x="0.795" y="0.954" size="1.016" layer="25" font="vector">&gt;Name</text>
<text x="0.795" y="-1.9085" size="1.016" layer="27" font="vector">&gt;Value</text>
<text x="0.159" y="0.3182" size="0.254" layer="100" font="vector" rot="R90">PaJa</text>
<rectangle x1="-0.7143" y1="-1.27" x2="-0.238" y2="1.27" layer="21"/>
<rectangle x1="0.2381" y1="-1.27" x2="0.7144" y2="1.27" layer="21"/>
<rectangle x1="-2.0638" y1="-0.1588" x2="-1.4288" y2="0.1588" layer="51"/>
<rectangle x1="1.4288" y1="-0.1588" x2="2.0638" y2="0.1588" layer="51"/>
<rectangle x1="-1.4288" y1="-0.1588" x2="-0.635" y2="0.1588" layer="21"/>
<rectangle x1="0.635" y1="-0.1588" x2="1.4288" y2="0.1588" layer="21"/>
</package>
<package name="C-7,5">
<circle x="-3.814" y="0" radius="0.477" width="0.127" layer="102"/>
<circle x="3.814" y="0" radius="0.477" width="0.127" layer="102"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<text x="0.795" y="0.954" size="1.016" layer="25" font="vector">&gt;Name</text>
<text x="0.795" y="-1.9085" size="1.016" layer="27" font="vector">&gt;Value</text>
<text x="0.159" y="0.477" size="0.254" layer="100" font="vector" rot="R90">PaJa</text>
<rectangle x1="-0.7155" y1="-1.431" x2="-0.2385" y2="1.431" layer="21"/>
<rectangle x1="0.2385" y1="-1.431" x2="0.7155" y2="1.431" layer="21"/>
<rectangle x1="-2.6988" y1="-0.1588" x2="-0.635" y2="0.1588" layer="21"/>
<rectangle x1="0.635" y1="-0.1588" x2="2.6988" y2="0.1588" layer="21"/>
<rectangle x1="-3.3338" y1="-0.1588" x2="-2.6988" y2="0.1588" layer="51"/>
<rectangle x1="2.6988" y1="-0.1588" x2="3.3338" y2="0.1588" layer="51"/>
</package>
<package name="C-10">
<wire x1="-6.35" y1="2.6035" x2="-6.35" y2="-2.6035" width="0.127" layer="21"/>
<wire x1="-6.35" y1="-2.6035" x2="6.35" y2="-2.6035" width="0.127" layer="21"/>
<wire x1="6.35" y1="-2.6035" x2="6.35" y2="2.6035" width="0.127" layer="21"/>
<wire x1="6.35" y1="2.6035" x2="-6.35" y2="2.6035" width="0.127" layer="21"/>
<circle x="-5.08" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="5.08" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" diameter="2.1844" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" diameter="2.1844" shape="octagon"/>
<text x="0.159" y="0.3182" size="0.254" layer="100" font="vector" rot="R90">PaJa</text>
<text x="-4.0444" y="1.1525" size="1.27" layer="25" font="vector">&gt;Name</text>
<text x="-4.3507" y="-2.4225" size="1.27" layer="27" font="vector">&gt;Value</text>
<rectangle x1="-0.7144" y1="-1.27" x2="-0.2381" y2="1.27" layer="21"/>
<rectangle x1="0.238" y1="-1.27" x2="0.7143" y2="1.27" layer="21"/>
<rectangle x1="-4.6038" y1="-0.1588" x2="-3.81" y2="0.1588" layer="51"/>
<rectangle x1="3.81" y1="-0.1588" x2="4.6038" y2="0.1588" layer="51"/>
<rectangle x1="-3.81" y1="-0.1588" x2="-0.635" y2="0.1588" layer="21"/>
<rectangle x1="0.635" y1="-0.1588" x2="3.81" y2="0.1588" layer="21"/>
</package>
<package name="1206">
<description>&lt;B&gt;SMD&lt;/B&gt; - velikost 1206</description>
<wire x1="-1.0541" y1="0.7938" x2="-0.7938" y2="0.7938" width="0.127" layer="51"/>
<wire x1="-0.7938" y1="0.7938" x2="0.7938" y2="0.7938" width="0.127" layer="21"/>
<wire x1="0.7938" y1="0.7938" x2="1.0541" y2="0.7938" width="0.127" layer="51"/>
<wire x1="-1.0541" y1="-0.7938" x2="-0.7938" y2="-0.7938" width="0.127" layer="51"/>
<wire x1="-0.7938" y1="-0.7938" x2="0.7938" y2="-0.7938" width="0.127" layer="21"/>
<wire x1="0.7938" y1="-0.7938" x2="1.0541" y2="-0.7938" width="0.127" layer="51"/>
<wire x1="1.0541" y1="0.7938" x2="1.0541" y2="-0.7938" width="0.127" layer="51"/>
<wire x1="-1.0541" y1="0.7938" x2="-1.0541" y2="-0.7938" width="0.127" layer="51"/>
<smd name="1" x="-1.5875" y="0" dx="1.143" dy="1.7018" layer="1"/>
<smd name="2" x="1.5875" y="0" dx="1.143" dy="1.7018" layer="1"/>
<text x="-0.3175" y="-1.1906" size="0.254" layer="100" font="vector">PaJa</text>
<text x="-0.7938" y="-0.4763" size="1.016" layer="25" font="vector">&gt;Name</text>
<text x="-0.7938" y="0.9525" size="1.016" layer="27" font="vector">&gt;Value</text>
<rectangle x1="-1.4541" y1="-0.7874" x2="-0.9461" y2="0.7874" layer="51"/>
<rectangle x1="0.9461" y1="-0.7874" x2="1.4541" y2="0.7874" layer="51"/>
</package>
<package name="0805">
<description>&lt;B&gt;SMD&lt;/B&gt; - velikost 0805</description>
<wire x1="-0.3226" y1="0.5645" x2="-0.5645" y2="0.5645" width="0.127" layer="51"/>
<wire x1="-0.5645" y1="0.5645" x2="-0.5645" y2="-0.5645" width="0.127" layer="51"/>
<wire x1="-0.5645" y1="-0.5645" x2="-0.3226" y2="-0.5645" width="0.127" layer="51"/>
<wire x1="0.3226" y1="0.5645" x2="0.5645" y2="0.5645" width="0.127" layer="51"/>
<wire x1="0.5645" y1="0.5645" x2="0.5645" y2="-0.5645" width="0.127" layer="51"/>
<wire x1="0.5645" y1="-0.5645" x2="0.3226" y2="-0.5645" width="0.127" layer="51"/>
<wire x1="-0.3226" y1="0.5645" x2="0.3226" y2="0.5645" width="0.127" layer="21"/>
<wire x1="0.3226" y1="-0.5645" x2="-0.3226" y2="-0.5645" width="0.127" layer="21"/>
<smd name="1" x="-0.9525" y="0" dx="1.016" dy="1.4224" layer="1"/>
<smd name="2" x="0.9525" y="0" dx="1.016" dy="1.4224" layer="1"/>
<text x="-1.397" y="-1.6351" size="0.8128" layer="27" font="vector" ratio="10">&gt;VALUE</text>
<text x="-1.3177" y="0.8413" size="0.8128" layer="25" font="vector" ratio="10">&gt;NAME</text>
<text x="0.3956" y="-0.4763" size="0.254" layer="100" font="vector" rot="R90">PaJa</text>
<rectangle x1="0.4064" y1="-0.6096" x2="0.9144" y2="0.6096" layer="51"/>
<rectangle x1="-0.9144" y1="-0.6096" x2="-0.4064" y2="0.6096" layer="51"/>
</package>
<package name="R-5">
<description>&lt;B&gt;Odpor&lt;/B&gt; - vel. 0204 - 0,4W - miniaturni</description>
<wire x1="-1.778" y1="0.635" x2="-1.524" y2="0.889" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="-1.778" y1="-0.635" x2="-1.524" y2="-0.889" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="1.524" y1="-0.889" x2="1.778" y2="-0.635" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="1.524" y1="0.889" x2="1.778" y2="0.6388" width="0.127" layer="21" curve="-89.149199"/>
<wire x1="1.778" y1="0.6388" x2="1.778" y2="0.635" width="0.127" layer="21" curve="-0.857165"/>
<wire x1="-1.524" y1="0.889" x2="-1.27" y2="0.889" width="0.127" layer="21"/>
<wire x1="-1.143" y1="0.762" x2="-1.27" y2="0.889" width="0.127" layer="21"/>
<wire x1="-1.524" y1="-0.889" x2="-1.27" y2="-0.889" width="0.127" layer="21"/>
<wire x1="-1.143" y1="-0.762" x2="-1.27" y2="-0.889" width="0.127" layer="21"/>
<wire x1="1.143" y1="0.762" x2="1.27" y2="0.889" width="0.127" layer="21"/>
<wire x1="1.143" y1="0.762" x2="-1.143" y2="0.762" width="0.127" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="1.27" y2="-0.889" width="0.127" layer="21"/>
<wire x1="1.143" y1="-0.762" x2="-1.143" y2="-0.762" width="0.127" layer="21"/>
<wire x1="1.524" y1="0.889" x2="1.27" y2="0.889" width="0.127" layer="21"/>
<wire x1="1.524" y1="-0.889" x2="1.27" y2="-0.889" width="0.127" layer="21"/>
<wire x1="1.778" y1="0.6388" x2="1.778" y2="-0.6332" width="0.127" layer="51"/>
<wire x1="-1.7787" y1="0.6274" x2="-1.7787" y2="-0.6446" width="0.127" layer="51"/>
<circle x="-2.54" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="2.54" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-2.54" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<pad name="2" x="2.54" y="0" drill="0.8128" diameter="1.778" shape="octagon"/>
<text x="-1.59" y="-0.477" size="1.016" layer="25" font="vector">&gt;Name</text>
<text x="-2.544" y="-1.908" size="1.016" layer="27" font="vector">&gt;Value</text>
<text x="-0.4797" y="0.8527" size="0.254" layer="100" font="vector">PaJa</text>
<rectangle x1="-2.1022" y1="-0.306" x2="-1.8124" y2="0.3068" layer="51"/>
<rectangle x1="1.8124" y1="-0.3068" x2="2.1022" y2="0.306" layer="51"/>
</package>
<package name="R-10">
<description>&lt;B&gt;Odpor&lt;/B&gt; - vel. 0207 - 0,6W - vetsi roztec</description>
<wire x1="-2.572" y1="1.016" x2="-2.699" y2="1.143" width="0.127" layer="21"/>
<wire x1="-2.572" y1="-1.016" x2="-2.699" y2="-1.143" width="0.127" layer="21"/>
<wire x1="2.572" y1="1.016" x2="2.699" y2="1.143" width="0.127" layer="21"/>
<wire x1="2.572" y1="1.016" x2="-2.572" y2="1.016" width="0.127" layer="21"/>
<wire x1="2.572" y1="-1.016" x2="2.699" y2="-1.143" width="0.127" layer="21"/>
<wire x1="2.572" y1="-1.016" x2="-2.572" y2="-1.016" width="0.127" layer="21"/>
<wire x1="3.08" y1="1.139" x2="2.699" y2="1.139" width="0.127" layer="21"/>
<wire x1="3.08" y1="-1.147" x2="2.699" y2="-1.147" width="0.127" layer="21"/>
<wire x1="-3.334" y1="0.893" x2="-3.08" y2="1.147" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="-3.334" y1="-0.885" x2="-3.08" y2="-1.139" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="-3.08" y1="-1.139" x2="-2.699" y2="-1.139" width="0.127" layer="21"/>
<wire x1="-3.08" y1="1.147" x2="-2.699" y2="1.147" width="0.127" layer="21"/>
<wire x1="-3.3321" y1="0.8823" x2="-3.3321" y2="-0.8667" width="0.127" layer="21"/>
<wire x1="3.08" y1="-1.147" x2="3.334" y2="-0.893" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="3.08" y1="1.139" x2="3.334" y2="0.885" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="3.3321" y1="-0.8823" x2="3.3321" y2="0.8667" width="0.127" layer="21"/>
<circle x="-5.08" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="5.08" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-5.08" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<pad name="2" x="5.08" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<text x="-3.1152" y="-0.6276" size="1.27" layer="25" font="vector">&gt;Name</text>
<text x="-0.3178" y="-0.6358" size="1.27" layer="27" font="vector">&gt;Value</text>
<text x="2.3342" y="-0.9351" size="0.254" layer="100" font="vector">PaJa</text>
<rectangle x1="-4.6038" y1="-0.3175" x2="-3.9688" y2="0.3175" layer="51"/>
<rectangle x1="3.9688" y1="-0.3175" x2="4.6038" y2="0.3175" layer="51"/>
<rectangle x1="-3.9688" y1="-0.3175" x2="-3.3338" y2="0.3175" layer="21"/>
<rectangle x1="3.3338" y1="-0.3175" x2="3.9688" y2="0.3175" layer="21"/>
</package>
<package name="R-12,7">
<description>&lt;B&gt;Odpor&lt;/B&gt; - roztec nozek 12,7mm</description>
<wire x1="3.7648" y1="1.2546" x2="3.8918" y2="1.3816" width="0.127" layer="21"/>
<wire x1="3.7648" y1="-1.2546" x2="3.8918" y2="-1.3816" width="0.127" layer="21"/>
<wire x1="4.2728" y1="1.3776" x2="3.8918" y2="1.3776" width="0.127" layer="21"/>
<wire x1="4.2728" y1="-1.3856" x2="3.8918" y2="-1.3856" width="0.127" layer="21"/>
<wire x1="4.2728" y1="-1.3856" x2="4.5268" y2="-1.1316" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="4.2728" y1="1.3776" x2="4.5268" y2="1.1236" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="4.5249" y1="-1.1209" x2="4.5249" y2="1.1053" width="0.127" layer="21"/>
<wire x1="-3.7649" y1="1.2547" x2="-3.8919" y2="1.3817" width="0.127" layer="21"/>
<wire x1="-3.7649" y1="-1.2546" x2="-3.8919" y2="-1.3816" width="0.127" layer="21"/>
<wire x1="3.7648" y1="1.2546" x2="-3.7649" y2="1.2547" width="0.127" layer="21"/>
<wire x1="3.7648" y1="-1.2546" x2="-3.7649" y2="-1.2546" width="0.127" layer="21"/>
<wire x1="-4.5269" y1="1.1316" x2="-4.2729" y2="1.3856" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="-4.5269" y1="-1.1236" x2="-4.2729" y2="-1.3776" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="-4.2729" y1="-1.3776" x2="-3.8919" y2="-1.3776" width="0.127" layer="21"/>
<wire x1="-4.2729" y1="1.3856" x2="-3.8919" y2="1.3856" width="0.127" layer="21"/>
<wire x1="-4.525" y1="1.1209" x2="-4.525" y2="-1.1054" width="0.127" layer="21"/>
<circle x="-6.35" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="6.35" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" diameter="2.1844" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" diameter="2.1844" shape="octagon"/>
<text x="-0.4813" y="-0.7958" size="1.4224" layer="27" font="vector">&gt;Value</text>
<text x="-4.2905" y="-0.7144" size="1.4224" layer="25" font="vector">&gt;Name</text>
<text x="3.5712" y="-1.1046" size="0.254" layer="100" font="vector">PaJa</text>
<rectangle x1="-5.08" y1="-0.3175" x2="-4.445" y2="0.3175" layer="21"/>
<rectangle x1="4.6038" y1="-0.3175" x2="5.08" y2="0.3175" layer="21"/>
<rectangle x1="4.445" y1="-0.3175" x2="5.08" y2="0.3175" layer="21"/>
<rectangle x1="-5.8738" y1="-0.3175" x2="-5.08" y2="0.3175" layer="51"/>
<rectangle x1="5.08" y1="-0.3175" x2="5.8738" y2="0.3175" layer="51"/>
</package>
<package name="R-7,5">
<description>&lt;B&gt;Odpor&lt;/B&gt; - vel. 0207 - 0,6W</description>
<wire x1="-3.175" y1="0.893" x2="-2.921" y2="1.147" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="-3.175" y1="-0.885" x2="-2.921" y2="-1.139" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="2.413" y1="-1.012" x2="2.54" y2="-1.139" width="0.127" layer="21"/>
<wire x1="2.413" y1="1.02" x2="2.54" y2="1.147" width="0.127" layer="21"/>
<wire x1="-2.413" y1="-1.012" x2="-2.54" y2="-1.139" width="0.127" layer="21"/>
<wire x1="-2.413" y1="-1.012" x2="2.413" y2="-1.012" width="0.127" layer="21"/>
<wire x1="-2.413" y1="1.02" x2="-2.54" y2="1.147" width="0.127" layer="21"/>
<wire x1="-2.413" y1="1.02" x2="2.413" y2="1.02" width="0.127" layer="21"/>
<wire x1="-2.921" y1="-1.139" x2="-2.54" y2="-1.139" width="0.127" layer="21"/>
<wire x1="-2.921" y1="1.147" x2="-2.54" y2="1.147" width="0.127" layer="21"/>
<wire x1="-3.1731" y1="0.8823" x2="-3.1731" y2="-0.8667" width="0.127" layer="51"/>
<wire x1="2.921" y1="-1.147" x2="3.175" y2="-0.893" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="2.921" y1="1.139" x2="3.175" y2="0.885" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="2.921" y1="1.139" x2="2.54" y2="1.139" width="0.127" layer="21"/>
<wire x1="2.921" y1="-1.147" x2="2.54" y2="-1.147" width="0.127" layer="21"/>
<wire x1="3.1731" y1="-0.8823" x2="3.1731" y2="0.8667" width="0.127" layer="51"/>
<circle x="-3.81" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="3.81" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-3.81" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<pad name="2" x="3.81" y="0" drill="0.8128" diameter="1.9304" shape="octagon"/>
<text x="-0.3178" y="-0.477" size="1.016" layer="27" font="vector">&gt;Value</text>
<text x="-2.7033" y="-0.477" size="1.016" layer="25" font="vector">&gt;Name</text>
<text x="2.1354" y="-0.8658" size="0.254" layer="100" font="vector">PaJa</text>
<rectangle x1="-3.4323" y1="-0.3053" x2="-3.1758" y2="0.3061" layer="51"/>
<rectangle x1="3.1759" y1="-0.3061" x2="3.4324" y2="0.3053" layer="51"/>
</package>
<package name="R-_2W">
<description>&lt;B&gt;Odpor&lt;/B&gt; - 2W - vel. 0414</description>
<wire x1="4.3998" y1="1.8896" x2="4.5268" y2="2.0166" width="0.127" layer="21"/>
<wire x1="4.3998" y1="-1.8896" x2="4.5268" y2="-2.0166" width="0.127" layer="21"/>
<wire x1="4.9078" y1="2.0126" x2="4.5268" y2="2.0126" width="0.127" layer="21"/>
<wire x1="4.9078" y1="-2.0206" x2="4.5268" y2="-2.0206" width="0.127" layer="21"/>
<wire x1="4.9078" y1="-2.0206" x2="5.1618" y2="-1.7666" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="4.9078" y1="2.0126" x2="5.1618" y2="1.7586" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="-4.3999" y1="1.8897" x2="-4.5269" y2="2.0167" width="0.127" layer="21"/>
<wire x1="-4.3999" y1="-1.8896" x2="-4.5269" y2="-2.0166" width="0.127" layer="21"/>
<wire x1="4.3998" y1="1.8896" x2="-4.3999" y2="1.8897" width="0.127" layer="21"/>
<wire x1="4.3998" y1="-1.8896" x2="-4.3999" y2="-1.8896" width="0.127" layer="21"/>
<wire x1="-5.1619" y1="1.7666" x2="-4.9079" y2="2.0206" width="0.127" layer="21" curve="-90" cap="flat"/>
<wire x1="-5.1619" y1="-1.7586" x2="-4.9079" y2="-2.0126" width="0.127" layer="21" curve="90" cap="flat"/>
<wire x1="-4.9079" y1="-2.0126" x2="-4.5269" y2="-2.0126" width="0.127" layer="21"/>
<wire x1="-4.9079" y1="2.0206" x2="-4.5269" y2="2.0206" width="0.127" layer="21"/>
<wire x1="-5.16" y1="1.7559" x2="-5.16" y2="0.9584" width="0.127" layer="21"/>
<wire x1="-5.16" y1="-0.9429" x2="-5.16" y2="-1.7404" width="0.127" layer="21"/>
<wire x1="5.1588" y1="-0.943" x2="5.1588" y2="-1.7405" width="0.127" layer="21"/>
<wire x1="5.1588" y1="1.7559" x2="5.1588" y2="0.9584" width="0.127" layer="21"/>
<wire x1="-5.16" y1="0.9621" x2="-5.16" y2="-0.9467" width="0.127" layer="51"/>
<wire x1="5.1588" y1="0.9621" x2="5.1588" y2="-0.9467" width="0.127" layer="51"/>
<circle x="-6.35" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="6.35" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-6.35" y="0" drill="0.8128" diameter="2.54" shape="octagon"/>
<pad name="2" x="6.35" y="0" drill="0.8128" diameter="2.54" shape="octagon"/>
<text x="-0.4813" y="-0.7958" size="1.6764" layer="27" font="vector">&gt;Value</text>
<text x="-4.9255" y="-0.7144" size="1.6764" layer="25" font="vector">&gt;Name</text>
<text x="4.1268" y="-1.7396" size="0.254" layer="100" font="vector">PaJa</text>
<text x="-4.7625" y="-1.5875" size="0.6096" layer="21" font="vector">2W</text>
<rectangle x1="-5.953" y1="-0.3175" x2="-5.1593" y2="0.3175" layer="51"/>
<rectangle x1="5.1594" y1="-0.3175" x2="5.9531" y2="0.3175" layer="51"/>
</package>
<package name="R-_10W">
<description>&lt;B&gt;Odpor&lt;/B&gt; - 10W - dratovy</description>
<wire x1="-24.13" y1="5.3975" x2="-24.13" y2="-5.3975" width="0.127" layer="21"/>
<wire x1="-24.13" y1="-5.3975" x2="24.13" y2="-5.3975" width="0.127" layer="21"/>
<wire x1="24.13" y1="-5.3975" x2="24.13" y2="5.3975" width="0.127" layer="21"/>
<wire x1="24.13" y1="5.3975" x2="-24.13" y2="5.3975" width="0.127" layer="21"/>
<circle x="-25.7175" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="25.7175" y="0" radius="0.5723" width="0.127" layer="102"/>
<pad name="1" x="-25.7175" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<pad name="2" x="25.7175" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<text x="-4.9375" y="-3.08" size="1.9304" layer="27" font="vector">&gt;VALUE</text>
<text x="-4.9375" y="1.2225" size="1.9304" layer="25" font="vector">&gt;NAME</text>
<text x="22.86" y="-5.08" size="0.254" layer="100" font="vector">PaJa</text>
<text x="-23.1775" y="-3.81" size="1.27" layer="21" font="vector">10W</text>
<rectangle x1="-25.2412" y1="-0.635" x2="-24.1299" y2="0.635" layer="51"/>
<rectangle x1="-25.5587" y1="0.4763" x2="-25.2412" y2="0.635" layer="51"/>
<rectangle x1="-25.5587" y1="-0.6349" x2="-25.2412" y2="-0.4762" layer="51"/>
<rectangle x1="24.13" y1="-0.635" x2="25.2413" y2="0.635" layer="51"/>
<rectangle x1="25.2413" y1="-0.6349" x2="25.5588" y2="-0.4762" layer="51"/>
<rectangle x1="25.2413" y1="0.4763" x2="25.5588" y2="0.635" layer="51"/>
</package>
<package name="R-_20W">
<description>&lt;B&gt;Odpor&lt;/B&gt; - 20W - dratovy</description>
<wire x1="30.1625" y1="6.985" x2="-30.1625" y2="6.985" width="0.127" layer="21"/>
<wire x1="30.1625" y1="1.1113" x2="30.1625" y2="-1.1113" width="0.127" layer="51"/>
<wire x1="-30.1626" y1="1.1113" x2="-30.1626" y2="-1.1113" width="0.127" layer="51"/>
<wire x1="-30.1625" y1="1.1113" x2="-30.1625" y2="6.985" width="0.127" layer="21"/>
<wire x1="-30.1625" y1="-1.1113" x2="-30.1625" y2="-6.985" width="0.127" layer="21"/>
<wire x1="-30.1625" y1="-6.985" x2="30.1625" y2="-6.985" width="0.127" layer="21"/>
<wire x1="30.1625" y1="-6.985" x2="30.1625" y2="-1.1113" width="0.127" layer="21"/>
<wire x1="30.1625" y1="6.985" x2="30.1625" y2="1.1113" width="0.127" layer="21"/>
<circle x="-31.75" y="0" radius="0.7099" width="0.127" layer="102"/>
<circle x="31.75" y="0" radius="0.7099" width="0.127" layer="102"/>
<pad name="1" x="-31.75" y="0" drill="1.27" diameter="3.2" shape="octagon"/>
<pad name="2" x="31.75" y="0" drill="1.27" diameter="3.2" shape="octagon"/>
<text x="-4.9375" y="-4.6675" size="2.1844" layer="27" font="vector">&gt;VALUE</text>
<text x="-4.9375" y="1.2225" size="2.1844" layer="25" font="vector">&gt;NAME</text>
<text x="28.8925" y="-6.6675" size="0.254" layer="100" font="vector">PaJa</text>
<text x="-29.21" y="-5.715" size="1.27" layer="21" font="vector">20W</text>
<rectangle x1="30.1625" y1="-0.635" x2="31.115" y2="0.635" layer="51"/>
<rectangle x1="-31.1149" y1="-0.635" x2="-30.1624" y2="0.635" layer="51"/>
<rectangle x1="-31.2738" y1="0.4763" x2="-31.115" y2="0.635" layer="51"/>
<rectangle x1="-31.2738" y1="-0.6349" x2="-31.115" y2="-0.4762" layer="51"/>
<rectangle x1="31.115" y1="-0.6349" x2="31.2738" y2="-0.4762" layer="51"/>
<rectangle x1="31.115" y1="0.4763" x2="31.2738" y2="0.635" layer="51"/>
</package>
<package name="R-_5W">
<description>&lt;B&gt;Odpor&lt;/B&gt; - 5W - keramicky</description>
<wire x1="-11.1125" y1="5.08" x2="-11.1125" y2="-5.08" width="0.127" layer="21"/>
<wire x1="-11.1125" y1="-5.08" x2="11.1125" y2="-5.08" width="0.127" layer="21"/>
<wire x1="11.1125" y1="-5.08" x2="11.1125" y2="5.08" width="0.127" layer="21"/>
<wire x1="11.1125" y1="5.08" x2="-11.1125" y2="5.08" width="0.127" layer="21"/>
<circle x="-12.7" y="0" radius="0.5724" width="0.127" layer="102"/>
<circle x="12.7" y="0" radius="0.5723" width="0.127" layer="102"/>
<pad name="1" x="-12.7" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<pad name="2" x="12.7" y="0" drill="1.016" diameter="2.54" shape="octagon"/>
<text x="-4.9375" y="-3.08" size="1.9304" layer="27" font="vector">&gt;VALUE</text>
<text x="-4.9375" y="1.2225" size="1.9304" layer="25" font="vector">&gt;NAME</text>
<text x="-10.16" y="-3.81" size="1.27" layer="21" font="vector">5W</text>
<text x="9.8425" y="-4.7625" size="0.254" layer="100" font="vector">PaJa</text>
<rectangle x1="-12.2237" y1="-0.635" x2="-11.1124" y2="0.635" layer="51"/>
<rectangle x1="-12.5412" y1="0.4763" x2="-12.2237" y2="0.635" layer="51"/>
<rectangle x1="-12.5412" y1="-0.6349" x2="-12.2237" y2="-0.4762" layer="51"/>
<rectangle x1="11.1125" y1="-0.635" x2="12.2238" y2="0.635" layer="51"/>
<rectangle x1="12.2238" y1="-0.6349" x2="12.5413" y2="-0.4762" layer="51"/>
<rectangle x1="12.2238" y1="0.4763" x2="12.5413" y2="0.635" layer="51"/>
</package>
<package name="R-STOJ">
<description>&lt;B&gt;Odpor&lt;/B&gt; - vel. 0207 - 0,6W - nastojato</description>
<wire x1="-1.905" y1="1.1113" x2="-1.905" y2="-1.1113" width="0.127" layer="21" curve="120.512458"/>
<wire x1="-0.635" y1="1.1113" x2="-0.635" y2="-1.1113" width="0.127" layer="21" curve="-120.512458"/>
<wire x1="-1.905" y1="1.1113" x2="-0.635" y2="1.1113" width="0.127" layer="51" curve="-59.487542"/>
<wire x1="-0.635" y1="-1.1113" x2="-1.905" y2="-1.1113" width="0.127" layer="51" curve="-59.487542"/>
<circle x="-1.27" y="0" radius="0.4763" width="0.127" layer="102"/>
<circle x="1.27" y="0" radius="0.4762" width="0.127" layer="102"/>
<pad name="1" x="-1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<pad name="2" x="1.27" y="0" drill="0.8128" diameter="1.27" shape="long" rot="R90"/>
<text x="-2.389" y="1.433" size="1.016" layer="25" font="vector">&gt;Name</text>
<text x="-2.544" y="-2.385" size="1.016" layer="27" font="vector">&gt;Value</text>
<text x="0.636" y="-1.272" size="0.254" layer="100" font="vector" rot="R90">PaJa</text>
<rectangle x1="-0.7938" y1="-0.3175" x2="-0.4763" y2="0.3175" layer="51"/>
<rectangle x1="0.4763" y1="-0.3175" x2="0.7938" y2="0.3175" layer="51"/>
<rectangle x1="-0.4763" y1="-0.3175" x2="0.4763" y2="0.3175" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="+5V">
<wire x1="-1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="1.27" y2="-1.905" width="0.254" layer="94"/>
<wire x1="0" y1="-2.5399" x2="0" y2="0" width="0.1524" layer="94"/>
<text x="0.9525" y="0.635" size="1.778" layer="96" rot="R90">&gt;Value</text>
<text x="1.27" y="-4.445" size="1.016" layer="101" ratio="6" rot="R90">+5V</text>
<pin name="+5V" x="0" y="-5.08" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="0V">
<wire x1="-1.7463" y1="-0.0001" x2="1.7463" y2="-0.0001" width="0.6096" layer="94"/>
<text x="0.3175" y="0.635" size="1.778" layer="96">&gt;Value</text>
<text x="-1.905" y="0.4762" size="1.016" layer="101" ratio="6">0V</text>
<text x="-0.1588" y="0.3175" size="0.254" layer="100" rot="R90">PaJa</text>
<pin name="0V" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="OZ">
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-5.08" x2="5.08" y2="0" width="0.254" layer="94"/>
<wire x1="5.08" y1="0" x2="-2.54" y2="5.08" width="0.254" layer="94"/>
<wire x1="-1.905" y1="2.54" x2="-0.635" y2="2.54" width="0.254" layer="94"/>
<wire x1="-1.905" y1="-2.54" x2="-0.635" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-1.27" y1="-1.905" x2="-1.27" y2="-3.175" width="0.254" layer="94"/>
<text x="-2.2226" y="-0.9526" size="1.778" layer="95">&gt;Name</text>
<text x="0.1587" y="-5.3975" size="1.778" layer="96">&gt;Value</text>
<text x="-2.0638" y="-4.445" size="0.254" layer="100" rot="R90">PaJa</text>
<pin name="IN-" x="-5.08" y="2.54" visible="pad" length="short" direction="in"/>
<pin name="NEIN+" x="-5.08" y="-2.54" visible="pad" length="short" direction="in"/>
<pin name="VYSTUP" x="7.62" y="0" visible="pad" length="short" direction="out" rot="R180"/>
</symbol>
<symbol name="POWER_SY">
<text x="-0.1587" y="2.54" size="0.254" layer="100" rot="R90">PaJa</text>
<text x="1.7464" y="2.3812" size="1.4224" layer="95" rot="R90">Vcc</text>
<text x="1.7462" y="-7.4613" size="1.4224" layer="95" rot="R90">Vee</text>
<text x="-0.9526" y="-2.0637" size="1.6764" layer="95">&gt;Part</text>
<pin name="VCC" x="0" y="5.08" visible="pad" length="short" direction="pwr" rot="R270"/>
<pin name="VEE" x="0" y="-7.62" visible="pad" length="short" direction="pwr" rot="R90"/>
</symbol>
<symbol name="C-EL">
<wire x1="-3.8173" y1="0.9547" x2="-2.5453" y2="0.9547" width="0.155" layer="94"/>
<wire x1="-3.1812" y1="1.5908" x2="-3.1812" y2="0.3188" width="0.155" layer="94"/>
<wire x1="-2.0638" y1="1.7463" x2="-1.4288" y2="1.7463" width="0.254" layer="94"/>
<wire x1="-1.4288" y1="1.7463" x2="-1.4288" y2="-1.5875" width="0.254" layer="94"/>
<wire x1="-1.4288" y1="-1.5875" x2="-2.0638" y2="-1.5875" width="0.254" layer="94"/>
<wire x1="-2.0638" y1="-1.5875" x2="-2.0638" y2="0" width="0.254" layer="94"/>
<wire x1="-2.0638" y1="0" x2="-2.0638" y2="1.7463" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0" x2="-2.0638" y2="0" width="0.152" layer="94"/>
<wire x1="-0.4763" y1="0" x2="0" y2="0" width="0.152" layer="94"/>
<text x="-1.589" y="-0.477" size="0.254" layer="100" rot="R90">PaJa</text>
<text x="0.3175" y="0.635" size="1.6764" layer="95">&gt;Name</text>
<text x="0.3175" y="-0.635" size="1.6764" layer="96" rot="MR180">&gt;Value</text>
<rectangle x1="-0.9525" y1="-1.7463" x2="-0.3175" y2="1.905" layer="94"/>
<pin name="C_EL+" x="-5.08" y="0" visible="off" length="short" direction="pas"/>
<pin name="C_EL-" x="2.54" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
<symbol name="C">
<wire x1="-2.54" y1="0" x2="-2.0638" y2="0" width="0.152" layer="94"/>
<wire x1="-0.4763" y1="0" x2="0" y2="0" width="0.152" layer="94"/>
<text x="-1.111" y="-0.479" size="0.254" layer="100" rot="R90">PaJa</text>
<text x="0.3175" y="0.635" size="1.6764" layer="95">&gt;Name</text>
<text x="0.3175" y="-0.635" size="1.6764" layer="96" rot="MR180">&gt;Value</text>
<rectangle x1="-2.2225" y1="-1.905" x2="-1.5875" y2="1.905" layer="94"/>
<rectangle x1="-0.9525" y1="-1.905" x2="-0.3175" y2="1.905" layer="94"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
<pin name="2" x="2.54" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
<symbol name="R">
<wire x1="-2.54" y1="1.0319" x2="2.54" y2="1.0319" width="0.254" layer="94"/>
<wire x1="2.54" y1="1.0319" x2="2.54" y2="-1.0319" width="0.254" layer="94"/>
<wire x1="2.54" y1="-1.0319" x2="-2.54" y2="-1.0319" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-1.0319" x2="-2.54" y2="1.0319" width="0.254" layer="94"/>
<text x="2.3815" y="-0.476" size="0.254" layer="100" rot="R90">PaJa</text>
<text x="-2.2225" y="1.5875" size="1.6764" layer="95">&gt;Name</text>
<text x="-2.2225" y="-1.5875" size="1.6764" layer="96" rot="MR180">&gt;Value</text>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="+5V" prefix="NAP">
<description>&lt;B&gt;SCH symbol&lt;/B&gt; - napajeni +5V</description>
<gates>
<gate name="+5" symbol="+5V" x="0" y="-5.08"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="0V" prefix="0V">
<description>&lt;B&gt;SCH symbol&lt;/B&gt; - napajeni 0V</description>
<gates>
<gate name="0" symbol="0V" x="-43.18" y="35.56"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="LM358" prefix="OZ">
<description>&lt;B&gt;OZ&lt;/B&gt; - 2x - nizkoprikonovy</description>
<gates>
<gate name="A" symbol="OZ" x="-38.1" y="35.56" swaplevel="1"/>
<gate name="B" symbol="OZ" x="-38.1" y="22.86" addlevel="always" swaplevel="1"/>
<gate name="NS" symbol="POWER_SY" x="-55.88" y="33.02" addlevel="request"/>
</gates>
<devices>
<device name="" package="DIL8">
<connects>
<connect gate="A" pin="IN-" pad="2"/>
<connect gate="A" pin="NEIN+" pad="3"/>
<connect gate="A" pin="VYSTUP" pad="1"/>
<connect gate="B" pin="IN-" pad="6"/>
<connect gate="B" pin="NEIN+" pad="5"/>
<connect gate="B" pin="VYSTUP" pad="7"/>
<connect gate="NS" pin="VCC" pad="8"/>
<connect gate="NS" pin="VEE" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="C-ELEKTROLYT" prefix="C" uservalue="yes">
<description>&lt;b&gt;Kondenzator - elektrolyticky&lt;/b&gt;</description>
<gates>
<gate name="C" symbol="C-EL" x="0" y="0" swaplevel="1"/>
</gates>
<devices>
<device name="_2" package="C-EL_2">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_2,5" package="C-EL_2,5">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_3,5" package="C-EL_3,5">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_5" package="C-EL_5">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_5+" package="C-EL_5+">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_7,5" package="C-EL_7,5">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_7,5+" package="C-EL7,5+">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_10" package="C-EL_10">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_10+" package="C-EL_10+">
<connects>
<connect gate="C" pin="C_EL+" pad="C+"/>
<connect gate="C" pin="C_EL-" pad="C-"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="C-KERAMIK" prefix="C" uservalue="yes">
<description>&lt;b&gt;Kondenzator - keramicky&lt;/b&gt;</description>
<gates>
<gate name="C" symbol="C" x="0" y="0" swaplevel="1"/>
</gates>
<devices>
<device name="_2,5" package="C-2,5">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_5" package="C-5">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_7,5" package="C-7,5">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_10" package="C-10">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_SMD_1206" package="1206">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_SMD_0805" package="0805">
<connects>
<connect gate="C" pin="1" pad="1"/>
<connect gate="C" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="R" prefix="R" uservalue="yes">
<description>&lt;b&gt;Rezistor&lt;/b&gt;</description>
<gates>
<gate name="R" symbol="R" x="0" y="0" swaplevel="1"/>
</gates>
<devices>
<device name="_5" package="R-5">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_10" package="R-10">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_12,7" package="R-12,7">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_7,5" package="R-7,5">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_SMD_1206" package="1206">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="__2W" package="R-_2W">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="__10W" package="R-_10W">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="__20W" package="R-_20W">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="__5W" package="R-_5W">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_STOJ" package="R-STOJ">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="_SMD_0805" package="0805">
<connects>
<connect gate="R" pin="1" pad="1"/>
<connect gate="R" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="JUM1" library="#PaJa_konektory" deviceset="S1G3_JUMP" device=""/>
<part name="+5V" library="#PaJa_konektory" deviceset="S1G2_JUMP" device=""/>
<part name="GND1" library="#PaJa_konektory" deviceset="S1G2_JUMP" device=""/>
<part name="IF" library="#PaJa_konektory" deviceset="S1G2_JUMP" device=""/>
<part name="GND2" library="#PaJa_konektory" deviceset="S1G2_JUMP" device=""/>
<part name="NAP1" library="#PaJa_30" deviceset="+5V" device=""/>
<part name="NAP2" library="#PaJa_30" deviceset="+5V" device=""/>
<part name="0V1" library="#PaJa_30" deviceset="0V" device=""/>
<part name="0V2" library="#PaJa_30" deviceset="0V" device=""/>
<part name="OZ1" library="#PaJa_30" deviceset="LM358" device=""/>
<part name="C1" library="#PaJa_30" deviceset="C-ELEKTROLYT" device="_3,5" value="4u7"/>
<part name="C2" library="#PaJa_30" deviceset="C-ELEKTROLYT" device="_3,5" value="4u7"/>
<part name="C3" library="#PaJa_30" deviceset="C-ELEKTROLYT" device="_3,5" value="4u7"/>
<part name="0V3" library="#PaJa_30" deviceset="0V" device=""/>
<part name="0V4" library="#PaJa_30" deviceset="0V" device=""/>
<part name="0V5" library="#PaJa_30" deviceset="0V" device=""/>
<part name="C4" library="#PaJa_30" deviceset="C-KERAMIK" device="_5" value="2n2"/>
<part name="C5" library="#PaJa_30" deviceset="C-KERAMIK" device="_5" value="2n2"/>
<part name="R1" library="#PaJa_30" deviceset="R" device="_7,5" value="12k"/>
<part name="R2" library="#PaJa_30" deviceset="R" device="_7,5" value="1M"/>
<part name="R3" library="#PaJa_30" deviceset="R" device="_7,5" value="10k"/>
<part name="R4" library="#PaJa_30" deviceset="R" device="_7,5" value="8k2"/>
<part name="R5" library="#PaJa_30" deviceset="R" device="_7,5" value="330k"/>
<part name="R6" library="#PaJa_30" deviceset="R" device="_7,5" value="100k"/>
<part name="R7" library="#PaJa_30" deviceset="R" device="_7,5" value="100k"/>
<part name="R8" library="#PaJa_30" deviceset="R" device="_7,5" value="1M"/>
<part name="NAP3" library="#PaJa_30" deviceset="+5V" device=""/>
<part name="0V6" library="#PaJa_30" deviceset="0V" device=""/>
<part name="NAP4" library="#PaJa_30" deviceset="+5V" device=""/>
<part name="0V7" library="#PaJa_30" deviceset="0V" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="JUM1" gate="JUMP" x="142.24" y="50.8" rot="R270"/>
<instance part="+5V" gate="JUMP" x="-38.1" y="66.04" rot="R270"/>
<instance part="GND1" gate="JUMP" x="-2.54" y="66.04" rot="R270"/>
<instance part="IF" gate="JUMP" x="-38.1" y="40.64" rot="R270"/>
<instance part="GND2" gate="JUMP" x="-2.54" y="40.64" rot="R270"/>
<instance part="NAP1" gate="+5" x="132.08" y="73.66"/>
<instance part="NAP2" gate="+5" x="-45.72" y="76.2"/>
<instance part="0V1" gate="0" x="132.08" y="35.56"/>
<instance part="0V2" gate="0" x="-7.62" y="30.48"/>
<instance part="OZ1" gate="A" x="68.58" y="45.72" rot="MR180"/>
<instance part="OZ1" gate="B" x="114.3" y="48.26" rot="MR180"/>
<instance part="C1" gate="C" x="43.18" y="48.26" rot="R180"/>
<instance part="C2" gate="C" x="43.18" y="20.32" rot="R180"/>
<instance part="C3" gate="C" x="86.36" y="45.72" rot="R180"/>
<instance part="0V3" gate="0" x="-7.62" y="55.88"/>
<instance part="0V4" gate="0" x="30.48" y="12.7"/>
<instance part="0V5" gate="0" x="30.48" y="27.94"/>
<instance part="C4" gate="C" x="71.12" y="30.48"/>
<instance part="C5" gate="C" x="116.84" y="33.02"/>
<instance part="R1" gate="R" x="30.48" y="40.64" rot="R90"/>
<instance part="R2" gate="R" x="71.12" y="20.32" rot="R180"/>
<instance part="R3" gate="R" x="55.88" y="20.32" rot="R180"/>
<instance part="R4" gate="R" x="99.06" y="45.72" rot="R180"/>
<instance part="R5" gate="R" x="55.88" y="60.96" rot="R270"/>
<instance part="R6" gate="R" x="30.48" y="66.04" rot="R270"/>
<instance part="R7" gate="R" x="30.48" y="78.74" rot="R270"/>
<instance part="R8" gate="R" x="116.84" y="25.4"/>
<instance part="NAP3" gate="+5" x="30.48" y="93.98"/>
<instance part="0V6" gate="0" x="30.48" y="55.88"/>
<instance part="OZ1" gate="NS" x="17.78" y="73.66"/>
<instance part="NAP4" gate="+5" x="17.78" y="93.98"/>
<instance part="0V7" gate="0" x="17.78" y="55.88"/>
</instances>
<busses>
</busses>
<nets>
<net name="+5V" class="0">
<segment>
<pinref part="NAP2" gate="+5" pin="+5V"/>
<pinref part="+5V" gate="JUMP" pin="1"/>
<wire x1="-45.72" y1="66.04" x2="-45.72" y2="71.12" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="66.04" x2="-45.72" y2="66.04" width="0.1524" layer="91"/>
<pinref part="+5V" gate="JUMP" pin="2"/>
<wire x1="-40.64" y1="63.5" x2="-45.72" y2="63.5" width="0.1524" layer="91"/>
<wire x1="-45.72" y1="63.5" x2="-45.72" y2="66.04" width="0.1524" layer="91"/>
<junction x="-45.72" y="66.04"/>
</segment>
<segment>
<pinref part="R7" gate="R" pin="1"/>
<pinref part="NAP3" gate="+5" pin="+5V"/>
<wire x1="30.48" y1="83.82" x2="30.48" y2="88.9" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="OZ1" gate="NS" pin="VCC"/>
<pinref part="NAP4" gate="+5" pin="+5V"/>
<wire x1="17.78" y1="78.74" x2="17.78" y2="88.9" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="JUM1" gate="JUMP" pin="1"/>
<pinref part="NAP1" gate="+5" pin="+5V"/>
<wire x1="139.7" y1="53.34" x2="132.08" y2="53.34" width="0.1524" layer="91"/>
<wire x1="132.08" y1="53.34" x2="132.08" y2="68.58" width="0.1524" layer="91"/>
</segment>
</net>
<net name="0V" class="0">
<segment>
<pinref part="GND1" gate="JUMP" pin="1"/>
<pinref part="0V3" gate="0" pin="0V"/>
<wire x1="-5.08" y1="66.04" x2="-7.62" y2="66.04" width="0.1524" layer="91"/>
<wire x1="-7.62" y1="66.04" x2="-7.62" y2="63.5" width="0.1524" layer="91"/>
<pinref part="GND1" gate="JUMP" pin="2"/>
<wire x1="-7.62" y1="63.5" x2="-7.62" y2="58.42" width="0.1524" layer="91"/>
<wire x1="-5.08" y1="63.5" x2="-7.62" y2="63.5" width="0.1524" layer="91"/>
<junction x="-7.62" y="63.5"/>
</segment>
<segment>
<pinref part="GND2" gate="JUMP" pin="1"/>
<pinref part="0V2" gate="0" pin="0V"/>
<wire x1="-5.08" y1="40.64" x2="-7.62" y2="40.64" width="0.1524" layer="91"/>
<wire x1="-7.62" y1="40.64" x2="-7.62" y2="38.1" width="0.1524" layer="91"/>
<pinref part="GND2" gate="JUMP" pin="2"/>
<wire x1="-7.62" y1="38.1" x2="-7.62" y2="33.02" width="0.1524" layer="91"/>
<wire x1="-5.08" y1="38.1" x2="-7.62" y2="38.1" width="0.1524" layer="91"/>
<junction x="-7.62" y="38.1"/>
</segment>
<segment>
<pinref part="0V5" gate="0" pin="0V"/>
<pinref part="R1" gate="R" pin="1"/>
<wire x1="30.48" y1="30.48" x2="30.48" y2="35.56" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="0V4" gate="0" pin="0V"/>
<wire x1="30.48" y1="15.24" x2="30.48" y2="20.32" width="0.1524" layer="91"/>
<pinref part="C2" gate="C" pin="C_EL-"/>
<wire x1="30.48" y1="20.32" x2="40.64" y2="20.32" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="OZ1" gate="NS" pin="VEE"/>
<pinref part="0V7" gate="0" pin="0V"/>
<wire x1="17.78" y1="66.04" x2="17.78" y2="58.42" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="JUM1" gate="JUMP" pin="2"/>
<pinref part="0V1" gate="0" pin="0V"/>
<wire x1="139.7" y1="50.8" x2="132.08" y2="50.8" width="0.1524" layer="91"/>
<wire x1="132.08" y1="50.8" x2="132.08" y2="38.1" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="R6" gate="R" pin="2"/>
<pinref part="0V6" gate="0" pin="0V"/>
<wire x1="30.48" y1="60.96" x2="30.48" y2="58.42" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="IF" gate="JUMP" pin="1"/>
<wire x1="-40.64" y1="40.64" x2="-45.72" y2="40.64" width="0.1524" layer="91"/>
<wire x1="-45.72" y1="40.64" x2="-45.72" y2="48.26" width="0.1524" layer="91"/>
<pinref part="C1" gate="C" pin="C_EL-"/>
<wire x1="-45.72" y1="48.26" x2="30.48" y2="48.26" width="0.1524" layer="91"/>
<pinref part="IF" gate="JUMP" pin="2"/>
<wire x1="30.48" y1="48.26" x2="40.64" y2="48.26" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="38.1" x2="-45.72" y2="38.1" width="0.1524" layer="91"/>
<wire x1="-45.72" y1="38.1" x2="-45.72" y2="40.64" width="0.1524" layer="91"/>
<junction x="-45.72" y="40.64"/>
<pinref part="R1" gate="R" pin="2"/>
<wire x1="30.48" y1="45.72" x2="30.48" y2="48.26" width="0.1524" layer="91"/>
<junction x="30.48" y="48.26"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="C1" gate="C" pin="C_EL+"/>
<pinref part="OZ1" gate="A" pin="NEIN+"/>
<wire x1="48.26" y1="48.26" x2="55.88" y2="48.26" width="0.1524" layer="91"/>
<pinref part="R5" gate="R" pin="2"/>
<wire x1="55.88" y1="48.26" x2="63.5" y2="48.26" width="0.1524" layer="91"/>
<wire x1="55.88" y1="48.26" x2="55.88" y2="55.88" width="0.1524" layer="91"/>
<junction x="55.88" y="48.26"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="R6" gate="R" pin="1"/>
<pinref part="R7" gate="R" pin="2"/>
<wire x1="30.48" y1="71.12" x2="30.48" y2="73.66" width="0.1524" layer="91"/>
<wire x1="30.48" y1="71.12" x2="55.88" y2="71.12" width="0.1524" layer="91"/>
<wire x1="55.88" y1="71.12" x2="104.14" y2="71.12" width="0.1524" layer="91"/>
<wire x1="104.14" y1="71.12" x2="104.14" y2="50.8" width="0.1524" layer="91"/>
<junction x="30.48" y="71.12"/>
<pinref part="OZ1" gate="B" pin="NEIN+"/>
<wire x1="104.14" y1="50.8" x2="109.22" y2="50.8" width="0.1524" layer="91"/>
<pinref part="R5" gate="R" pin="1"/>
<wire x1="55.88" y1="66.04" x2="55.88" y2="71.12" width="0.1524" layer="91"/>
<junction x="55.88" y="71.12"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="C4" gate="C" pin="2"/>
<wire x1="73.66" y1="30.48" x2="78.74" y2="30.48" width="0.1524" layer="91"/>
<wire x1="78.74" y1="30.48" x2="78.74" y2="45.72" width="0.1524" layer="91"/>
<pinref part="OZ1" gate="A" pin="VYSTUP"/>
<wire x1="78.74" y1="45.72" x2="76.2" y2="45.72" width="0.1524" layer="91"/>
<pinref part="C3" gate="C" pin="C_EL-"/>
<wire x1="78.74" y1="45.72" x2="83.82" y2="45.72" width="0.1524" layer="91"/>
<junction x="78.74" y="45.72"/>
<wire x1="78.74" y1="30.48" x2="78.74" y2="20.32" width="0.1524" layer="91"/>
<junction x="78.74" y="30.48"/>
<pinref part="R2" gate="R" pin="1"/>
<wire x1="78.74" y1="20.32" x2="76.2" y2="20.32" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="C2" gate="C" pin="C_EL+"/>
<pinref part="R3" gate="R" pin="2"/>
<wire x1="48.26" y1="20.32" x2="50.8" y2="20.32" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="R3" gate="R" pin="1"/>
<pinref part="R2" gate="R" pin="2"/>
<wire x1="60.96" y1="20.32" x2="63.5" y2="20.32" width="0.1524" layer="91"/>
<pinref part="OZ1" gate="A" pin="IN-"/>
<wire x1="63.5" y1="20.32" x2="66.04" y2="20.32" width="0.1524" layer="91"/>
<wire x1="63.5" y1="43.18" x2="63.5" y2="30.48" width="0.1524" layer="91"/>
<junction x="63.5" y="20.32"/>
<pinref part="C4" gate="C" pin="1"/>
<wire x1="63.5" y1="30.48" x2="63.5" y2="20.32" width="0.1524" layer="91"/>
<wire x1="66.04" y1="30.48" x2="63.5" y2="30.48" width="0.1524" layer="91"/>
<junction x="63.5" y="30.48"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="R8" gate="R" pin="2"/>
<wire x1="121.92" y1="25.4" x2="127" y2="25.4" width="0.1524" layer="91"/>
<pinref part="OZ1" gate="B" pin="VYSTUP"/>
<wire x1="127" y1="25.4" x2="127" y2="33.02" width="0.1524" layer="91"/>
<wire x1="127" y1="33.02" x2="127" y2="48.26" width="0.1524" layer="91"/>
<wire x1="127" y1="48.26" x2="121.92" y2="48.26" width="0.1524" layer="91"/>
<pinref part="C5" gate="C" pin="2"/>
<wire x1="119.38" y1="33.02" x2="127" y2="33.02" width="0.1524" layer="91"/>
<junction x="127" y="33.02"/>
<pinref part="JUM1" gate="JUMP" pin="3"/>
<wire x1="139.7" y1="48.26" x2="127" y2="48.26" width="0.1524" layer="91"/>
<junction x="127" y="48.26"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="C3" gate="C" pin="C_EL+"/>
<pinref part="R4" gate="R" pin="2"/>
<wire x1="91.44" y1="45.72" x2="93.98" y2="45.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="R4" gate="R" pin="1"/>
<pinref part="OZ1" gate="B" pin="IN-"/>
<wire x1="104.14" y1="45.72" x2="106.68" y2="45.72" width="0.1524" layer="91"/>
<pinref part="R8" gate="R" pin="1"/>
<wire x1="106.68" y1="45.72" x2="109.22" y2="45.72" width="0.1524" layer="91"/>
<wire x1="106.68" y1="33.02" x2="106.68" y2="25.4" width="0.1524" layer="91"/>
<wire x1="106.68" y1="25.4" x2="111.76" y2="25.4" width="0.1524" layer="91"/>
<pinref part="C5" gate="C" pin="1"/>
<wire x1="111.76" y1="33.02" x2="106.68" y2="33.02" width="0.1524" layer="91"/>
<wire x1="106.68" y1="45.72" x2="106.68" y2="33.02" width="0.1524" layer="91"/>
<junction x="106.68" y="45.72"/>
<junction x="106.68" y="33.02"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
