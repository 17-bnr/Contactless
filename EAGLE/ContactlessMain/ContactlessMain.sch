<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.6.2">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="10" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="26" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="AUDIO">
<packages>
<package name="MJ-1835">
<pad name="P$1" x="0" y="5" drill="3.3" shape="square"/>
<pad name="P$2" x="0" y="0" drill="3.3" shape="long"/>
<pad name="P$3" x="0" y="-5" drill="3.3" shape="long"/>
<wire x1="-3.6" y1="10.5" x2="-3.6" y2="-6" width="0.127" layer="21"/>
<wire x1="-3.6" y1="-6" x2="3.6" y2="-6" width="0.127" layer="21"/>
<wire x1="3.6" y1="-6" x2="3.6" y2="10.5" width="0.127" layer="21"/>
<wire x1="3.6" y1="10.5" x2="-3.6" y2="10.5" width="0.127" layer="21"/>
<text x="-3.81" y="2.54" size="1.016" layer="25" font="vector" ratio="15" rot="R90">&gt;Name</text>
<text x="5.08" y="2.54" size="1.016" layer="27" font="vector" ratio="15" rot="R90">&gt;Value</text>
</package>
</packages>
<symbols>
<symbol name="STEREO">
<pin name="L" x="10.16" y="5.08" length="middle" rot="R180"/>
<pin name="R" x="10.16" y="0" length="middle" rot="R180"/>
<pin name="GND" x="10.16" y="-5.08" length="middle" rot="R180"/>
<wire x1="5.08" y1="7.62" x2="-7.62" y2="7.62" width="0.254" layer="94"/>
<wire x1="-7.62" y1="7.62" x2="-7.62" y2="-7.62" width="0.254" layer="94"/>
<wire x1="-7.62" y1="-7.62" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="-7.62" x2="5.08" y2="7.62" width="0.254" layer="94"/>
<text x="-7.62" y="8.382" size="1.27" layer="95">&gt;NAME</text>
<text x="-7.62" y="-9.398" size="1.27" layer="96">&gt;VALUE</text>
</symbol>
</symbols>
<devicesets>
<deviceset name="MJ-1835" prefix="J">
<gates>
<gate name="G$1" symbol="STEREO" x="0" y="0"/>
</gates>
<devices>
<device name="" package="MJ-1835">
<connects>
<connect gate="G$1" pin="GND" pad="P$1"/>
<connect gate="G$1" pin="L" pad="P$3"/>
<connect gate="G$1" pin="R" pad="P$2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="PINHEADER">
<packages>
<package name="PINHEADER-8*1">
<pad name="3" x="-1.27" y="0" drill="1.02" rot="R90"/>
<pad name="4" x="1.27" y="0" drill="1.02" rot="R90"/>
<pad name="5" x="3.81" y="0" drill="1.02" rot="R90"/>
<pad name="6" x="6.35" y="0" drill="1.02" rot="R90"/>
<pad name="2" x="-3.81" y="0" drill="1.02" rot="R90"/>
<pad name="1" x="-6.35" y="0" drill="1.02" shape="square" rot="R90"/>
<wire x1="-7.62" y1="1.25" x2="-7.62" y2="-1.25" width="0.1524" layer="21"/>
<wire x1="-7.62" y1="-1.25" x2="12.7" y2="-1.25" width="0.1524" layer="21"/>
<wire x1="12.7" y1="-1.25" x2="12.7" y2="1.25" width="0.1524" layer="21"/>
<wire x1="12.7" y1="1.25" x2="-7.62" y2="1.25" width="0.1524" layer="21"/>
<text x="-7.62" y="1.905" size="1.016" layer="25" font="vector" ratio="15">&gt;Name</text>
<pad name="7" x="8.89" y="0" drill="1.02" rot="R90"/>
<pad name="8" x="11.43" y="0" drill="1.02" rot="R90"/>
</package>
</packages>
<symbols>
<symbol name="CON-8PIN">
<pin name="1" x="-7.62" y="7.62" visible="pad" length="middle"/>
<pin name="2" x="-7.62" y="5.08" visible="pad" length="middle"/>
<pin name="3" x="-7.62" y="2.54" visible="pad" length="middle"/>
<pin name="4" x="-7.62" y="0" visible="pad" length="middle"/>
<pin name="5" x="-7.62" y="-2.54" visible="pad" length="middle"/>
<pin name="6" x="-7.62" y="-5.08" visible="pad" length="middle"/>
<wire x1="-2.54" y1="10.16" x2="-2.54" y2="-12.7" width="0.1524" layer="94"/>
<wire x1="-2.54" y1="-12.7" x2="5.08" y2="-12.7" width="0.1524" layer="94"/>
<wire x1="5.08" y1="-12.7" x2="5.08" y2="10.16" width="0.1524" layer="94"/>
<wire x1="5.08" y1="10.16" x2="-2.54" y2="10.16" width="0.1524" layer="94"/>
<text x="-2.54" y="11.43" size="1.778" layer="95">&gt;NAME</text>
<pin name="7" x="-7.62" y="-7.62" visible="pad" length="middle"/>
<pin name="8" x="-7.62" y="-10.16" visible="pad" length="middle"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="PINHEADER-8*1" prefix="J">
<gates>
<gate name="G$1" symbol="CON-8PIN" x="0" y="0"/>
</gates>
<devices>
<device name="8*1" package="PINHEADER-8*1">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="NUCLEO-F446RE-IO">
<packages>
<package name="MODULE_NUCLEO-F446RE">
<wire x1="-35" y1="-37.75" x2="-35" y2="39.75" width="0.127" layer="21"/>
<wire x1="-35" y1="39.75" x2="-33.5" y2="41.25" width="0.127" layer="21" curve="-90"/>
<wire x1="-33.5" y1="41.25" x2="33.5" y2="41.25" width="0.127" layer="21"/>
<wire x1="33.5" y1="41.25" x2="35" y2="39.75" width="0.127" layer="21" curve="-90"/>
<wire x1="35" y1="39.75" x2="35" y2="-37.75" width="0.127" layer="21"/>
<wire x1="35" y1="-37.75" x2="33.5" y2="-39.25" width="0.127" layer="21" curve="-90"/>
<wire x1="33.5" y1="-39.25" x2="13" y2="-39.25" width="0.127" layer="21"/>
<wire x1="13" y1="-39.25" x2="11" y2="-41.25" width="0.127" layer="21"/>
<wire x1="11" y1="-41.25" x2="-22" y2="-41.25" width="0.127" layer="21"/>
<wire x1="-22" y1="-41.25" x2="-24" y2="-39.25" width="0.127" layer="21"/>
<wire x1="-24" y1="-39.25" x2="-33.5" y2="-39.25" width="0.127" layer="21"/>
<wire x1="-33.5" y1="-39.25" x2="-35" y2="-37.75" width="0.127" layer="21" curve="-90"/>
<wire x1="27.98" y1="10.78" x2="27.98" y2="-37.48" width="0.127" layer="51"/>
<wire x1="27.98" y1="-37.48" x2="32.98" y2="-37.48" width="0.127" layer="51"/>
<wire x1="32.98" y1="-37.48" x2="32.98" y2="10.78" width="0.127" layer="51"/>
<wire x1="32.98" y1="10.78" x2="27.98" y2="10.78" width="0.127" layer="51"/>
<wire x1="27.98" y1="10.78" x2="27.98" y2="-37.48" width="0.127" layer="21"/>
<wire x1="27.98" y1="-37.48" x2="32.98" y2="-37.48" width="0.127" layer="21"/>
<wire x1="32.98" y1="-37.48" x2="32.98" y2="10.78" width="0.127" layer="21"/>
<wire x1="32.98" y1="10.78" x2="27.98" y2="10.78" width="0.127" layer="21"/>
<wire x1="-32.98" y1="10.78" x2="-32.98" y2="-37.48" width="0.127" layer="51"/>
<wire x1="-32.98" y1="-37.48" x2="-27.98" y2="-37.48" width="0.127" layer="51"/>
<wire x1="-27.98" y1="-37.48" x2="-27.98" y2="10.78" width="0.127" layer="51"/>
<wire x1="-27.98" y1="10.78" x2="-32.98" y2="10.78" width="0.127" layer="51"/>
<wire x1="-32.98" y1="10.78" x2="-32.98" y2="-37.48" width="0.127" layer="21"/>
<wire x1="-32.98" y1="-37.48" x2="-27.98" y2="-37.48" width="0.127" layer="21"/>
<wire x1="-27.98" y1="-37.48" x2="-27.98" y2="10.78" width="0.127" layer="21"/>
<wire x1="-27.98" y1="10.78" x2="-32.98" y2="10.78" width="0.127" layer="21"/>
<circle x="-33.5" y="9.51" radius="0.1" width="0.2" layer="21"/>
<circle x="-33.5" y="9.51" radius="0.1" width="0.2" layer="51"/>
<circle x="27.46" y="9.51" radius="0.1" width="0.2" layer="51"/>
<circle x="27.46" y="9.51" radius="0.1" width="0.2" layer="21"/>
<wire x1="-35" y1="-37.75" x2="-35" y2="39.75" width="0.127" layer="51"/>
<wire x1="-35" y1="39.75" x2="-33.5" y2="41.25" width="0.127" layer="51" curve="-90"/>
<wire x1="-33.5" y1="41.25" x2="33.5" y2="41.25" width="0.127" layer="51"/>
<wire x1="33.5" y1="41.25" x2="35" y2="39.75" width="0.127" layer="51" curve="-90"/>
<wire x1="35" y1="39.75" x2="35" y2="-37.75" width="0.127" layer="51"/>
<wire x1="35" y1="-37.75" x2="33.5" y2="-39.25" width="0.127" layer="51" curve="-90"/>
<wire x1="33.5" y1="-39.25" x2="13" y2="-39.25" width="0.127" layer="51"/>
<wire x1="13" y1="-39.25" x2="11" y2="-41.25" width="0.127" layer="51"/>
<wire x1="11" y1="-41.25" x2="-22" y2="-41.25" width="0.127" layer="51"/>
<wire x1="-22" y1="-41.25" x2="-24" y2="-39.25" width="0.127" layer="51"/>
<wire x1="-24" y1="-39.25" x2="-33.5" y2="-39.25" width="0.127" layer="51"/>
<wire x1="-33.5" y1="-39.25" x2="-35" y2="-37.75" width="0.127" layer="51" curve="-90"/>
<wire x1="-24.125" y1="-39.5" x2="-33.5" y2="-39.5" width="0.05" layer="39"/>
<wire x1="-33.5" y1="-39.5" x2="-35.25" y2="-37.75" width="0.05" layer="39" curve="-90"/>
<wire x1="-35.25" y1="-37.75" x2="-35.25" y2="39.75" width="0.05" layer="39"/>
<wire x1="-35.25" y1="39.75" x2="-33.5" y2="41.5" width="0.05" layer="39" curve="-90"/>
<wire x1="-33.5" y1="41.5" x2="33.5" y2="41.5" width="0.05" layer="39"/>
<wire x1="33.5" y1="41.5" x2="35.25" y2="39.75" width="0.05" layer="39" curve="-90"/>
<wire x1="35.25" y1="39.75" x2="35.25" y2="-37.75" width="0.05" layer="39"/>
<wire x1="35.25" y1="-37.75" x2="33.5" y2="-39.5" width="0.05" layer="39" curve="-90"/>
<wire x1="33.5" y1="-39.5" x2="13.125" y2="-39.5" width="0.05" layer="39"/>
<wire x1="13.125" y1="-39.5" x2="11.125" y2="-41.5" width="0.05" layer="39"/>
<wire x1="11.125" y1="-41.5" x2="-22.125" y2="-41.5" width="0.05" layer="39"/>
<wire x1="-22.125" y1="-41.5" x2="-24.125" y2="-39.5" width="0.05" layer="39"/>
<text x="-35" y="42" size="1.27" layer="25">&gt;NAME</text>
<text x="-35" y="-40" size="1.27" layer="27" align="top-left">&gt;VALUE</text>
<pad name="CN7_1" x="-31.75" y="9.51" drill="1.02" shape="square"/>
<pad name="CN7_37" x="-31.75" y="-36.21" drill="1.02"/>
<pad name="CN7_2" x="-29.21" y="9.51" drill="1.02"/>
<pad name="CN7_3" x="-31.75" y="6.97" drill="1.02"/>
<pad name="CN7_4" x="-29.21" y="6.97" drill="1.02"/>
<pad name="CN7_5" x="-31.75" y="4.43" drill="1.02"/>
<pad name="CN7_6" x="-29.21" y="4.43" drill="1.02"/>
<pad name="CN7_7" x="-31.75" y="1.89" drill="1.02"/>
<pad name="CN7_8" x="-29.21" y="1.89" drill="1.02"/>
<pad name="CN7_9" x="-31.75" y="-0.65" drill="1.02"/>
<pad name="CN7_10" x="-29.21" y="-0.65" drill="1.02"/>
<pad name="CN7_11" x="-31.75" y="-3.19" drill="1.02"/>
<pad name="CN7_12" x="-29.21" y="-3.19" drill="1.02"/>
<pad name="CN7_13" x="-31.75" y="-5.73" drill="1.02"/>
<pad name="CN7_14" x="-29.21" y="-5.73" drill="1.02"/>
<pad name="CN7_15" x="-31.75" y="-8.27" drill="1.02"/>
<pad name="CN7_16" x="-29.21" y="-8.27" drill="1.02"/>
<pad name="CN7_17" x="-31.75" y="-10.81" drill="1.02"/>
<pad name="CN7_18" x="-29.21" y="-10.81" drill="1.02"/>
<pad name="CN7_19" x="-31.75" y="-13.35" drill="1.02"/>
<pad name="CN7_20" x="-29.21" y="-13.35" drill="1.02"/>
<pad name="CN7_21" x="-31.75" y="-15.89" drill="1.02"/>
<pad name="CN7_22" x="-29.21" y="-15.89" drill="1.02"/>
<pad name="CN7_23" x="-31.75" y="-18.43" drill="1.02"/>
<pad name="CN7_24" x="-29.21" y="-18.43" drill="1.02"/>
<pad name="CN7_25" x="-31.75" y="-20.97" drill="1.02"/>
<pad name="CN7_26" x="-29.21" y="-20.97" drill="1.02"/>
<pad name="CN7_27" x="-31.75" y="-23.51" drill="1.02"/>
<pad name="CN7_28" x="-29.21" y="-23.51" drill="1.02"/>
<pad name="CN7_29" x="-31.75" y="-26.05" drill="1.02"/>
<pad name="CN7_30" x="-29.21" y="-26.05" drill="1.02"/>
<pad name="CN7_31" x="-31.75" y="-28.59" drill="1.02"/>
<pad name="CN7_32" x="-29.21" y="-28.59" drill="1.02"/>
<pad name="CN7_33" x="-31.75" y="-31.13" drill="1.02"/>
<pad name="CN7_34" x="-29.21" y="-31.13" drill="1.02"/>
<pad name="CN7_35" x="-31.75" y="-33.67" drill="1.02"/>
<pad name="CN7_36" x="-29.21" y="-33.67" drill="1.02"/>
<pad name="CN7_38" x="-29.21" y="-36.21" drill="1.02"/>
<pad name="CN10_1" x="29.21" y="9.51" drill="1.02" shape="square"/>
<pad name="CN10_37" x="29.21" y="-36.21" drill="1.02"/>
<pad name="CN10_2" x="31.75" y="9.51" drill="1.02"/>
<pad name="CN10_3" x="29.21" y="6.97" drill="1.02"/>
<pad name="CN10_4" x="31.75" y="6.97" drill="1.02"/>
<pad name="CN10_5" x="29.21" y="4.43" drill="1.02"/>
<pad name="CN10_6" x="31.75" y="4.43" drill="1.02"/>
<pad name="CN10_7" x="29.21" y="1.89" drill="1.02"/>
<pad name="CN10_8" x="31.75" y="1.89" drill="1.02"/>
<pad name="CN10_9" x="29.21" y="-0.65" drill="1.02"/>
<pad name="CN10_10" x="31.75" y="-0.65" drill="1.02"/>
<pad name="CN10_11" x="29.21" y="-3.19" drill="1.02"/>
<pad name="CN10_12" x="31.75" y="-3.19" drill="1.02"/>
<pad name="CN10_13" x="29.21" y="-5.73" drill="1.02"/>
<pad name="CN10_14" x="31.75" y="-5.73" drill="1.02"/>
<pad name="CN10_15" x="29.21" y="-8.27" drill="1.02"/>
<pad name="CN10_16" x="31.75" y="-8.27" drill="1.02"/>
<pad name="CN10_17" x="29.21" y="-10.81" drill="1.02"/>
<pad name="CN10_18" x="31.75" y="-10.81" drill="1.02"/>
<pad name="CN10_19" x="29.21" y="-13.35" drill="1.02"/>
<pad name="CN10_20" x="31.75" y="-13.35" drill="1.02"/>
<pad name="CN10_21" x="29.21" y="-15.89" drill="1.02"/>
<pad name="CN10_22" x="31.75" y="-15.89" drill="1.02"/>
<pad name="CN10_23" x="29.21" y="-18.43" drill="1.02"/>
<pad name="CN10_24" x="31.75" y="-18.43" drill="1.02"/>
<pad name="CN10_25" x="29.21" y="-20.97" drill="1.02"/>
<pad name="CN10_26" x="31.75" y="-20.97" drill="1.02"/>
<pad name="CN10_27" x="29.21" y="-23.51" drill="1.02"/>
<pad name="CN10_28" x="31.75" y="-23.51" drill="1.02"/>
<pad name="CN10_29" x="29.21" y="-26.05" drill="1.02"/>
<pad name="CN10_30" x="31.75" y="-26.05" drill="1.02"/>
<pad name="CN10_31" x="29.21" y="-28.59" drill="1.02"/>
<pad name="CN10_32" x="31.75" y="-28.59" drill="1.02"/>
<pad name="CN10_33" x="29.21" y="-31.13" drill="1.02"/>
<pad name="CN10_34" x="31.75" y="-31.13" drill="1.02"/>
<pad name="CN10_35" x="29.21" y="-33.67" drill="1.02"/>
<pad name="CN10_36" x="31.75" y="-33.67" drill="1.02"/>
<pad name="CN10_38" x="31.75" y="-36.21" drill="1.02"/>
<hole x="24.13" y="12.05" drill="3"/>
<hole x="-24.13" y="13.32" drill="3"/>
</package>
</packages>
<symbols>
<symbol name="NUCLEO-F446RE_IO-1">
<wire x1="-17.78" y1="40.64" x2="-17.78" y2="-43.18" width="0.254" layer="94"/>
<wire x1="-17.78" y1="-43.18" x2="17.78" y2="-43.18" width="0.254" layer="94"/>
<wire x1="17.78" y1="-43.18" x2="17.78" y2="40.64" width="0.254" layer="94"/>
<wire x1="17.78" y1="40.64" x2="-17.78" y2="40.64" width="0.254" layer="94"/>
<text x="-17.78" y="41.91" size="1.778" layer="95">&gt;NAME</text>
<text x="-17.78" y="-44.45" size="1.778" layer="96" rot="MR180">&gt;VALUE</text>
<pin name="PC10" x="-22.86" y="-25.4" visible="pin" length="middle"/>
<pin name="PC11" x="-22.86" y="-22.86" visible="pin" length="middle"/>
<pin name="PC12" x="-22.86" y="-27.94" visible="pin" length="middle"/>
<pin name="PD2" x="22.86" y="17.78" visible="pin" length="middle" rot="R180"/>
<pin name="VDD" x="22.86" y="27.94" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="E5V" x="22.86" y="33.02" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="BOOT0" x="-22.86" y="17.78" visible="pin" length="middle"/>
<pin name="CN7_IOREF" x="22.86" y="20.32" visible="pin" length="middle" rot="R180"/>
<pin name="PA13" x="-22.86" y="5.08" visible="pin" length="middle"/>
<pin name="CN7_RESET" x="-22.86" y="20.32" visible="pin" length="middle"/>
<pin name="PA14" x="-22.86" y="2.54" visible="pin" length="middle"/>
<pin name="CN7_+3V3" x="22.86" y="38.1" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="PA15" x="-22.86" y="0" visible="pin" length="middle"/>
<pin name="CN7_+5V" x="22.86" y="35.56" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="CN7_GND" x="22.86" y="-40.64" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="PB7" x="-22.86" y="-7.62" visible="pin" length="middle"/>
<pin name="PC13" x="-22.86" y="-30.48" visible="pin" length="middle"/>
<pin name="CN7_VIN" x="22.86" y="25.4" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="PC14" x="-22.86" y="-33.02" visible="pin" length="middle"/>
<pin name="PC15" x="-22.86" y="-35.56" visible="pin" length="middle"/>
<pin name="PA0" x="-22.86" y="12.7" visible="pin" length="middle"/>
<pin name="PH0" x="22.86" y="12.7" visible="pin" length="middle" rot="R180"/>
<pin name="PA1" x="-22.86" y="10.16" visible="pin" length="middle"/>
<pin name="PH1" x="22.86" y="10.16" visible="pin" length="middle" rot="R180"/>
<pin name="PA4" x="-22.86" y="7.62" visible="pin" length="middle"/>
<pin name="VBAT" x="22.86" y="30.48" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="PB0" x="-22.86" y="-5.08" visible="pin" length="middle"/>
<pin name="PC2" x="-22.86" y="-17.78" visible="pin" length="middle"/>
<pin name="PC1/PB9" x="-22.86" y="-15.24" visible="pin" length="middle"/>
<pin name="PC3" x="-22.86" y="-20.32" visible="pin" length="middle"/>
<pin name="PC0/PB8" x="-22.86" y="-12.7" visible="pin" length="middle"/>
</symbol>
<symbol name="NUCLEO-F446RE_IO-2">
<wire x1="-12.7" y1="38.1" x2="-12.7" y2="-40.64" width="0.254" layer="94"/>
<wire x1="-12.7" y1="-40.64" x2="12.7" y2="-40.64" width="0.254" layer="94"/>
<wire x1="12.7" y1="-40.64" x2="12.7" y2="38.1" width="0.254" layer="94"/>
<wire x1="12.7" y1="38.1" x2="-12.7" y2="38.1" width="0.254" layer="94"/>
<text x="-12.7" y="39.37" size="1.778" layer="95">&gt;NAME</text>
<text x="-12.7" y="-41.91" size="1.778" layer="96" rot="MR180">&gt;VALUE</text>
<pin name="PC9" x="17.78" y="15.24" visible="pin" length="middle" rot="R180"/>
<pin name="PC8" x="17.78" y="17.78" visible="pin" length="middle" rot="R180"/>
<pin name="PB8" x="-17.78" y="-15.24" visible="pin" length="middle"/>
<pin name="PC6" x="17.78" y="22.86" visible="pin" length="middle" rot="R180"/>
<pin name="PB9" x="-17.78" y="-17.78" visible="pin" length="middle"/>
<pin name="PC5" x="17.78" y="25.4" visible="pin" length="middle" rot="R180"/>
<pin name="AVDD" x="17.78" y="35.56" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="U5V" x="17.78" y="33.02" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="CN10_GND" x="17.78" y="-38.1" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="PA5" x="-17.78" y="22.86" visible="pin" length="middle"/>
<pin name="PA12" x="-17.78" y="5.08" visible="pin" length="middle"/>
<pin name="PA6" x="-17.78" y="20.32" visible="pin" length="middle"/>
<pin name="PA11" x="-17.78" y="7.62" visible="pin" length="middle"/>
<pin name="PA7" x="-17.78" y="17.78" visible="pin" length="middle"/>
<pin name="PB12" x="-17.78" y="-22.86" visible="pin" length="middle"/>
<pin name="PB6" x="-17.78" y="-12.7" visible="pin" length="middle"/>
<pin name="PC7" x="17.78" y="20.32" visible="pin" length="middle" rot="R180"/>
<pin name="PA9" x="-17.78" y="12.7" visible="pin" length="middle"/>
<pin name="PB2" x="-17.78" y="-2.54" visible="pin" length="middle"/>
<pin name="PA8" x="-17.78" y="15.24" visible="pin" length="middle"/>
<pin name="PB1" x="-17.78" y="0" visible="pin" length="middle"/>
<pin name="PB10" x="-17.78" y="-20.32" visible="pin" length="middle"/>
<pin name="PB15" x="-17.78" y="-30.48" visible="pin" length="middle"/>
<pin name="PB4" x="-17.78" y="-7.62" visible="pin" length="middle"/>
<pin name="PB14" x="-17.78" y="-27.94" visible="pin" length="middle"/>
<pin name="PB5" x="-17.78" y="-10.16" visible="pin" length="middle"/>
<pin name="PB13" x="-17.78" y="-25.4" visible="pin" length="middle"/>
<pin name="PB3" x="-17.78" y="-5.08" visible="pin" length="middle"/>
<pin name="AGND" x="17.78" y="-35.56" visible="pin" length="middle" direction="pwr" rot="R180"/>
<pin name="PA10" x="-17.78" y="10.16" visible="pin" length="middle"/>
<pin name="PC4" x="17.78" y="27.94" visible="pin" length="middle" rot="R180"/>
<pin name="PA2" x="-17.78" y="27.94" visible="pin" length="middle"/>
<pin name="PA3" x="-17.78" y="25.4" visible="pin" length="middle"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="NUCLEO-F446RE-IO" prefix="U">
<gates>
<gate name="G$1" symbol="NUCLEO-F446RE_IO-1" x="-73.66" y="0"/>
<gate name="G$2" symbol="NUCLEO-F446RE_IO-2" x="-12.7" y="0"/>
</gates>
<devices>
<device name="" package="MODULE_NUCLEO-F446RE">
<connects>
<connect gate="G$1" pin="BOOT0" pad="CN7_7"/>
<connect gate="G$1" pin="CN7_+3V3" pad="CN7_16"/>
<connect gate="G$1" pin="CN7_+5V" pad="CN7_18"/>
<connect gate="G$1" pin="CN7_GND" pad="CN7_8 CN7_19"/>
<connect gate="G$1" pin="CN7_IOREF" pad="CN7_12"/>
<connect gate="G$1" pin="CN7_RESET" pad="CN7_14"/>
<connect gate="G$1" pin="CN7_VIN" pad="CN7_24"/>
<connect gate="G$1" pin="E5V" pad="CN7_6"/>
<connect gate="G$1" pin="PA0" pad="CN7_28"/>
<connect gate="G$1" pin="PA1" pad="CN7_30"/>
<connect gate="G$1" pin="PA13" pad="CN7_13"/>
<connect gate="G$1" pin="PA14" pad="CN7_15"/>
<connect gate="G$1" pin="PA15" pad="CN7_17"/>
<connect gate="G$1" pin="PA4" pad="CN7_32"/>
<connect gate="G$1" pin="PB0" pad="CN7_34"/>
<connect gate="G$1" pin="PB7" pad="CN7_21"/>
<connect gate="G$1" pin="PC0/PB8" pad="CN7_38"/>
<connect gate="G$1" pin="PC1/PB9" pad="CN7_36"/>
<connect gate="G$1" pin="PC10" pad="CN7_1"/>
<connect gate="G$1" pin="PC11" pad="CN7_2"/>
<connect gate="G$1" pin="PC12" pad="CN7_3"/>
<connect gate="G$1" pin="PC13" pad="CN7_23"/>
<connect gate="G$1" pin="PC14" pad="CN7_25"/>
<connect gate="G$1" pin="PC15" pad="CN7_27"/>
<connect gate="G$1" pin="PC2" pad="CN7_35"/>
<connect gate="G$1" pin="PC3" pad="CN7_37"/>
<connect gate="G$1" pin="PD2" pad="CN7_4"/>
<connect gate="G$1" pin="PH0" pad="CN7_29"/>
<connect gate="G$1" pin="PH1" pad="CN7_31"/>
<connect gate="G$1" pin="VBAT" pad="CN7_33"/>
<connect gate="G$1" pin="VDD" pad="CN7_5"/>
<connect gate="G$2" pin="AGND" pad="CN10_32"/>
<connect gate="G$2" pin="AVDD" pad="CN10_7"/>
<connect gate="G$2" pin="CN10_GND" pad="CN10_9 CN10_20"/>
<connect gate="G$2" pin="PA10" pad="CN10_33"/>
<connect gate="G$2" pin="PA11" pad="CN10_14"/>
<connect gate="G$2" pin="PA12" pad="CN10_12"/>
<connect gate="G$2" pin="PA2" pad="CN10_35"/>
<connect gate="G$2" pin="PA3" pad="CN10_37"/>
<connect gate="G$2" pin="PA5" pad="CN10_11"/>
<connect gate="G$2" pin="PA6" pad="CN10_13"/>
<connect gate="G$2" pin="PA7" pad="CN10_15"/>
<connect gate="G$2" pin="PA8" pad="CN10_23"/>
<connect gate="G$2" pin="PA9" pad="CN10_21"/>
<connect gate="G$2" pin="PB1" pad="CN10_24"/>
<connect gate="G$2" pin="PB10" pad="CN10_25"/>
<connect gate="G$2" pin="PB12" pad="CN10_16"/>
<connect gate="G$2" pin="PB13" pad="CN10_30"/>
<connect gate="G$2" pin="PB14" pad="CN10_28"/>
<connect gate="G$2" pin="PB15" pad="CN10_26"/>
<connect gate="G$2" pin="PB2" pad="CN10_22"/>
<connect gate="G$2" pin="PB3" pad="CN10_31"/>
<connect gate="G$2" pin="PB4" pad="CN10_27"/>
<connect gate="G$2" pin="PB5" pad="CN10_29"/>
<connect gate="G$2" pin="PB6" pad="CN10_17"/>
<connect gate="G$2" pin="PB8" pad="CN10_3"/>
<connect gate="G$2" pin="PB9" pad="CN10_5"/>
<connect gate="G$2" pin="PC4" pad="CN10_34"/>
<connect gate="G$2" pin="PC5" pad="CN10_6"/>
<connect gate="G$2" pin="PC6" pad="CN10_4"/>
<connect gate="G$2" pin="PC7" pad="CN10_19"/>
<connect gate="G$2" pin="PC8" pad="CN10_2"/>
<connect gate="G$2" pin="PC9" pad="CN10_1"/>
<connect gate="G$2" pin="U5V" pad="CN10_8"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-molex" urn="urn:adsk.eagle:library:165">
<description>&lt;b&gt;Molex Connectors&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="0446200002" urn="urn:adsk.eagle:footprint:8078500/1" library_version="5">
<description>&lt;b&gt;RJ-45 INVERTED MODULAR JACK ASSEMBLY WITH RJ-11 KEEPOUT FEATURE&lt;/b&gt;&lt;p&gt;
Source: &lt;a href="http://www.farnell.com/datasheets/73207.pdf"&gt; Data sheet &lt;/a&gt;&lt;p&gt;
&lt;b&gt;MOLEX  0446200002  BUCHSE, RJ45, GESCHIRMT &lt;/b&gt;&lt;p&gt;
Source: &lt;a href=http://de.farnell.com/molex/0446200002/buchse-rj45-geschirmt/dp/9251910?ost=9251910"&gt; Farnell &lt;/a&gt;&lt;br&gt;</description>
<hole x="6.35" y="0" drill="3.2512"/>
<hole x="-6.35" y="0" drill="3.2512"/>
<pad name="1" x="3.556" y="2.54" drill="0.889" diameter="1.4224"/>
<pad name="2" x="2.54" y="4.318" drill="0.889" diameter="1.4224"/>
<pad name="3" x="1.524" y="2.54" drill="0.889" diameter="1.4224"/>
<pad name="4" x="0.508" y="4.318" drill="0.889" diameter="1.4224"/>
<pad name="5" x="-0.508" y="2.54" drill="0.889" diameter="1.4224"/>
<pad name="6" x="-1.524" y="4.318" drill="0.889" diameter="1.4224"/>
<pad name="7" x="-2.54" y="2.54" drill="0.889" diameter="1.4224"/>
<pad name="8" x="-3.556" y="4.318" drill="0.889" diameter="1.4224"/>
<pad name="S@1" x="-8.1026" y="3.429" drill="1.5748" diameter="2.286"/>
<pad name="S@2" x="8.1026" y="3.429" drill="1.5748" diameter="2.286"/>
<wire x1="-7.775" y1="-4.975" x2="7.775" y2="-4.975" width="0.2032" layer="21"/>
<wire x1="-7.775" y1="-4.975" x2="-7.775" y2="10.011" width="0.2032" layer="21"/>
<wire x1="7.775" y1="-4.975" x2="7.775" y2="10.011" width="0.2032" layer="21"/>
<wire x1="-7.775" y1="10.011" x2="7.775" y2="10.011" width="0.2032" layer="21"/>
<wire x1="-7.874" y1="-4.064" x2="-8.128" y2="-3.556" width="0.2032" layer="21"/>
<wire x1="-8.128" y1="-3.556" x2="-8.636" y2="1.27" width="0.2032" layer="21" curve="-53.130102"/>
<wire x1="-8.636" y1="1.27" x2="-8.89" y2="1.778" width="0.2032" layer="21" curve="106.260205"/>
<wire x1="7.874" y1="-4.064" x2="8.128" y2="-3.556" width="0.2032" layer="21"/>
<wire x1="8.128" y1="-3.556" x2="8.636" y2="1.27" width="0.2032" layer="21" curve="53.130102"/>
<wire x1="8.636" y1="1.27" x2="8.89" y2="1.778" width="0.2032" layer="21" curve="-106.260205"/>
<text x="-7.62" y="10.795" size="1.27" layer="25">&gt;NAME</text>
<text x="-6.35" y="7.62" size="1.27" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="0446200002" urn="urn:adsk.eagle:package:8078921/1" type="box" library_version="5">
<description>&lt;b&gt;RJ-45 INVERTED MODULAR JACK ASSEMBLY WITH RJ-11 KEEPOUT FEATURE&lt;/b&gt;&lt;p&gt;
Source: &lt;a href="http://www.farnell.com/datasheets/73207.pdf"&gt; Data sheet &lt;/a&gt;&lt;p&gt;
&lt;b&gt;MOLEX  0446200002  BUCHSE, RJ45, GESCHIRMT &lt;/b&gt;&lt;p&gt;
Source: &lt;a href=http://de.farnell.com/molex/0446200002/buchse-rj45-geschirmt/dp/9251910?ost=9251910"&gt; Farnell &lt;/a&gt;&lt;br&gt;</description>
<packageinstances>
<packageinstance name="0446200002"/>
</packageinstances>
</package3d>
</packages3d>
<symbols>
<symbol name="JACK8SH" urn="urn:adsk.eagle:symbol:7672/2" library_version="5">
<wire x1="1.524" y1="10.668" x2="0" y2="10.668" width="0.254" layer="94"/>
<wire x1="0" y1="10.668" x2="0" y2="9.652" width="0.254" layer="94"/>
<wire x1="0" y1="9.652" x2="1.524" y2="9.652" width="0.254" layer="94"/>
<wire x1="1.524" y1="8.128" x2="0" y2="8.128" width="0.254" layer="94"/>
<wire x1="0" y1="8.128" x2="0" y2="7.112" width="0.254" layer="94"/>
<wire x1="0" y1="7.112" x2="1.524" y2="7.112" width="0.254" layer="94"/>
<wire x1="1.524" y1="5.588" x2="0" y2="5.588" width="0.254" layer="94"/>
<wire x1="0" y1="5.588" x2="0" y2="4.572" width="0.254" layer="94"/>
<wire x1="0" y1="4.572" x2="1.524" y2="4.572" width="0.254" layer="94"/>
<wire x1="1.524" y1="3.048" x2="0" y2="3.048" width="0.254" layer="94"/>
<wire x1="0" y1="3.048" x2="0" y2="2.032" width="0.254" layer="94"/>
<wire x1="0" y1="2.032" x2="1.524" y2="2.032" width="0.254" layer="94"/>
<wire x1="1.524" y1="0.508" x2="0" y2="0.508" width="0.254" layer="94"/>
<wire x1="0" y1="0.508" x2="0" y2="-0.508" width="0.254" layer="94"/>
<wire x1="0" y1="-0.508" x2="1.524" y2="-0.508" width="0.254" layer="94"/>
<wire x1="1.524" y1="-2.032" x2="0" y2="-2.032" width="0.254" layer="94"/>
<wire x1="0" y1="-2.032" x2="0" y2="-3.048" width="0.254" layer="94"/>
<wire x1="0" y1="-3.048" x2="1.524" y2="-3.048" width="0.254" layer="94"/>
<wire x1="1.524" y1="-4.572" x2="0" y2="-4.572" width="0.254" layer="94"/>
<wire x1="0" y1="-4.572" x2="0" y2="-5.588" width="0.254" layer="94"/>
<wire x1="0" y1="-5.588" x2="1.524" y2="-5.588" width="0.254" layer="94"/>
<wire x1="1.524" y1="-7.112" x2="0" y2="-7.112" width="0.254" layer="94"/>
<wire x1="0" y1="-7.112" x2="0" y2="-8.128" width="0.254" layer="94"/>
<wire x1="0" y1="-8.128" x2="1.524" y2="-8.128" width="0.254" layer="94"/>
<wire x1="-0.381" y1="-10.16" x2="0.254" y2="-10.16" width="0.127" layer="94"/>
<wire x1="1.016" y1="-10.16" x2="1.524" y2="-10.16" width="0.127" layer="94"/>
<wire x1="2.286" y1="-10.16" x2="2.794" y2="-10.16" width="0.127" layer="94"/>
<wire x1="3.048" y1="-10.16" x2="3.302" y2="-10.16" width="0.127" layer="94"/>
<wire x1="3.302" y1="-10.16" x2="3.302" y2="-9.652" width="0.127" layer="94"/>
<wire x1="3.302" y1="9.906" x2="3.302" y2="10.414" width="0.127" layer="94"/>
<wire x1="3.302" y1="10.922" x2="3.302" y2="11.43" width="0.127" layer="94"/>
<wire x1="3.302" y1="11.43" x2="2.794" y2="11.43" width="0.127" layer="94"/>
<wire x1="2.286" y1="11.43" x2="1.778" y2="11.43" width="0.127" layer="94"/>
<wire x1="1.27" y1="11.43" x2="0.762" y2="11.43" width="0.127" layer="94"/>
<wire x1="0.254" y1="11.43" x2="-0.381" y2="11.43" width="0.127" layer="94"/>
<wire x1="-0.381" y1="11.43" x2="-0.381" y2="10.668" width="0.127" layer="94"/>
<wire x1="-0.381" y1="9.652" x2="-0.381" y2="8.128" width="0.127" layer="94"/>
<wire x1="-0.381" y1="7.112" x2="-0.381" y2="5.588" width="0.127" layer="94"/>
<wire x1="-0.381" y1="4.572" x2="-0.381" y2="3.048" width="0.127" layer="94"/>
<wire x1="-0.381" y1="2.032" x2="-0.381" y2="0.508" width="0.127" layer="94"/>
<wire x1="-0.381" y1="-0.508" x2="-0.381" y2="-2.032" width="0.127" layer="94"/>
<wire x1="-0.381" y1="-3.048" x2="-0.381" y2="-4.572" width="0.127" layer="94"/>
<wire x1="-0.381" y1="-5.588" x2="-0.381" y2="-7.112" width="0.127" layer="94"/>
<wire x1="-0.381" y1="-8.128" x2="-0.381" y2="-10.16" width="0.127" layer="94"/>
<wire x1="4.826" y1="4.064" x2="4.826" y2="3.048" width="0.1998" layer="94"/>
<wire x1="4.826" y1="3.048" x2="4.826" y2="2.54" width="0.1998" layer="94"/>
<wire x1="4.826" y1="2.54" x2="4.826" y2="2.032" width="0.1998" layer="94"/>
<wire x1="4.826" y1="2.032" x2="4.826" y2="1.524" width="0.1998" layer="94"/>
<wire x1="4.826" y1="1.524" x2="4.826" y2="1.016" width="0.1998" layer="94"/>
<wire x1="4.826" y1="1.016" x2="4.826" y2="0.508" width="0.1998" layer="94"/>
<wire x1="4.826" y1="0.508" x2="4.826" y2="0" width="0.1998" layer="94"/>
<wire x1="4.826" y1="0" x2="4.826" y2="-0.508" width="0.1998" layer="94"/>
<wire x1="4.826" y1="-0.508" x2="4.826" y2="-1.524" width="0.1998" layer="94"/>
<wire x1="4.826" y1="-1.524" x2="7.366" y2="-1.524" width="0.1998" layer="94"/>
<wire x1="7.366" y1="-1.524" x2="7.366" y2="-0.254" width="0.1998" layer="94"/>
<wire x1="7.366" y1="-0.254" x2="8.89" y2="-0.254" width="0.1998" layer="94"/>
<wire x1="8.89" y1="-0.254" x2="8.89" y2="2.794" width="0.1998" layer="94"/>
<wire x1="8.89" y1="2.794" x2="7.366" y2="2.794" width="0.1998" layer="94"/>
<wire x1="7.366" y1="2.794" x2="7.366" y2="4.064" width="0.1998" layer="94"/>
<wire x1="7.366" y1="4.064" x2="4.826" y2="4.064" width="0.1998" layer="94"/>
<wire x1="4.826" y1="3.048" x2="5.588" y2="3.048" width="0.1998" layer="94"/>
<wire x1="4.826" y1="2.54" x2="5.588" y2="2.54" width="0.1998" layer="94"/>
<wire x1="4.826" y1="2.032" x2="5.588" y2="2.032" width="0.1998" layer="94"/>
<wire x1="4.826" y1="1.524" x2="5.588" y2="1.524" width="0.1998" layer="94"/>
<wire x1="4.826" y1="1.016" x2="5.588" y2="1.016" width="0.1998" layer="94"/>
<wire x1="4.826" y1="0.508" x2="5.588" y2="0.508" width="0.1998" layer="94"/>
<wire x1="4.826" y1="0" x2="5.588" y2="0" width="0.1998" layer="94"/>
<wire x1="4.826" y1="-0.508" x2="5.588" y2="-0.508" width="0.1998" layer="94"/>
<wire x1="3.302" y1="8.636" x2="3.302" y2="9.144" width="0.127" layer="94"/>
<wire x1="3.302" y1="7.366" x2="3.302" y2="7.874" width="0.127" layer="94"/>
<wire x1="3.302" y1="6.096" x2="3.302" y2="6.604" width="0.127" layer="94"/>
<wire x1="3.302" y1="4.826" x2="3.302" y2="5.334" width="0.127" layer="94"/>
<wire x1="3.302" y1="3.556" x2="3.302" y2="4.064" width="0.127" layer="94"/>
<wire x1="3.302" y1="2.286" x2="3.302" y2="2.794" width="0.127" layer="94"/>
<wire x1="3.302" y1="1.016" x2="3.302" y2="1.524" width="0.127" layer="94"/>
<wire x1="3.302" y1="-0.254" x2="3.302" y2="0.254" width="0.127" layer="94"/>
<wire x1="3.302" y1="-1.524" x2="3.302" y2="-1.016" width="0.127" layer="94"/>
<wire x1="3.302" y1="-2.794" x2="3.302" y2="-2.286" width="0.127" layer="94"/>
<wire x1="3.302" y1="-4.064" x2="3.302" y2="-3.556" width="0.127" layer="94"/>
<wire x1="3.302" y1="-5.334" x2="3.302" y2="-4.826" width="0.127" layer="94"/>
<wire x1="3.302" y1="-6.604" x2="3.302" y2="-6.096" width="0.127" layer="94"/>
<wire x1="3.302" y1="-7.874" x2="3.302" y2="-7.366" width="0.127" layer="94"/>
<wire x1="3.302" y1="-9.144" x2="3.302" y2="-8.636" width="0.127" layer="94"/>
<text x="3.81" y="10.668" size="1.778" layer="95">&gt;NAME</text>
<text x="3.81" y="-10.922" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-2.54" y="10.16" visible="pad" length="short" direction="pas" swaplevel="1"/>
<pin name="2" x="-2.54" y="7.62" visible="pad" length="short" direction="pas" swaplevel="1"/>
<pin name="3" x="-2.54" y="5.08" visible="pad" length="short" direction="pas" swaplevel="1"/>
<pin name="4" x="-2.54" y="2.54" visible="pad" length="short" direction="pas" swaplevel="1"/>
<pin name="5" x="-2.54" y="0" visible="pad" length="short" direction="pas" swaplevel="1"/>
<pin name="6" x="-2.54" y="-2.54" visible="pad" length="short" direction="pas" swaplevel="1"/>
<pin name="7" x="-2.54" y="-5.08" visible="pad" length="short" direction="pas" swaplevel="1"/>
<pin name="8" x="-2.54" y="-7.62" visible="pad" length="short" direction="pas" swaplevel="1"/>
<pin name="S2" x="2.54" y="-12.7" visible="off" length="short" direction="pas" swaplevel="2" rot="R90"/>
<pin name="S1" x="0" y="-12.7" visible="off" length="short" direction="pas" swaplevel="2" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="0446200002" urn="urn:adsk.eagle:component:8079451/3" prefix="X" library_version="5">
<description>&lt;b&gt;RJ-45 INVERTED MODULAR JACK ASSEMBLY WITH RJ-11 KEEPOUT FEATURE&lt;/b&gt;&lt;p&gt;
Source: &lt;a href="http://www.farnell.com/datasheets/73207.pdf"&gt; Data sheet &lt;/a&gt;&lt;p&gt;
&lt;b&gt;MOLEX  0446200002  BUCHSE, RJ45, GESCHIRMT &lt;/b&gt;&lt;p&gt;
Source: &lt;a href=http://de.farnell.com/molex/0446200002/buchse-rj45-geschirmt/dp/9251910?ost=9251910"&gt; Farnell &lt;/a&gt;&lt;br&gt;</description>
<gates>
<gate name="P" symbol="JACK8SH" x="0" y="0"/>
</gates>
<devices>
<device name="" package="0446200002">
<connects>
<connect gate="P" pin="1" pad="1"/>
<connect gate="P" pin="2" pad="2"/>
<connect gate="P" pin="3" pad="3"/>
<connect gate="P" pin="4" pad="4"/>
<connect gate="P" pin="5" pad="5"/>
<connect gate="P" pin="6" pad="6"/>
<connect gate="P" pin="7" pad="7"/>
<connect gate="P" pin="8" pad="8"/>
<connect gate="P" pin="S1" pad="S@1"/>
<connect gate="P" pin="S2" pad="S@2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:8078921/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="MOLEX" constant="no"/>
<attribute name="MPN" value="0446200002" constant="no"/>
<attribute name="OC_FARNELL" value="9251910" constant="no"/>
<attribute name="POPULARITY" value="0" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="supply1" urn="urn:adsk.eagle:library:371">
<description>&lt;b&gt;Supply Symbols&lt;/b&gt;&lt;p&gt;
 GND, VCC, 0V, +5V, -5V, etc.&lt;p&gt;
 Please keep in mind, that these devices are necessary for the
 automatic wiring of the supply signals.&lt;p&gt;
 The pin name defined in the symbol is identical to the net which is to be wired automatically.&lt;p&gt;
 In this library the device names are the same as the pin names of the symbols, therefore the correct signal names appear next to the supply symbols in the schematic.&lt;p&gt;
 &lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
</packages>
<symbols>
<symbol name="GND" urn="urn:adsk.eagle:symbol:26925/1" library_version="1">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
</symbol>
<symbol name="+3V3" urn="urn:adsk.eagle:symbol:26950/1" library_version="1">
<wire x1="1.27" y1="-1.905" x2="0" y2="0" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="-1.27" y2="-1.905" width="0.254" layer="94"/>
<text x="-2.54" y="-5.08" size="1.778" layer="96" rot="R90">&gt;VALUE</text>
<pin name="+3V3" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GND" urn="urn:adsk.eagle:component:26954/1" prefix="GND" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="+3V3" urn="urn:adsk.eagle:component:26981/1" prefix="+3V3" library_version="1">
<description>&lt;b&gt;SUPPLY SYMBOL&lt;/b&gt;</description>
<gates>
<gate name="G$1" symbol="+3V3" x="0" y="0"/>
</gates>
<devices>
<device name="">
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
<part name="J1" library="AUDIO" deviceset="MJ-1835" device=""/>
<part name="J2" library="PINHEADER" deviceset="PINHEADER-8*1" device="8*1"/>
<part name="U1" library="NUCLEO-F446RE-IO" deviceset="NUCLEO-F446RE-IO" device=""/>
<part name="X1" library="con-molex" library_urn="urn:adsk.eagle:library:165" deviceset="0446200002" device="" package3d_urn="urn:adsk.eagle:package:8078921/1"/>
<part name="GND3" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="+3V1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="+3V3" device=""/>
<part name="GND1" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
<part name="GND2" library="supply1" library_urn="urn:adsk.eagle:library:371" deviceset="GND" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="J1" gate="G$1" x="-54.61" y="-7.62" smashed="yes">
<attribute name="NAME" x="-62.23" y="0.762" size="1.27" layer="95"/>
<attribute name="VALUE" x="-62.23" y="-17.018" size="1.27" layer="96"/>
</instance>
<instance part="J2" gate="G$1" x="38.1" y="20.32" smashed="yes">
<attribute name="NAME" x="35.56" y="31.75" size="1.778" layer="95"/>
</instance>
<instance part="U1" gate="G$1" x="-59.69" y="-91.44" smashed="yes">
<attribute name="NAME" x="-77.47" y="-49.53" size="1.778" layer="95"/>
<attribute name="VALUE" x="-77.47" y="-135.89" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="U1" gate="G$2" x="38.1" y="-92.71" smashed="yes">
<attribute name="NAME" x="25.4" y="-53.34" size="1.778" layer="95"/>
<attribute name="VALUE" x="25.4" y="-134.62" size="1.778" layer="96" rot="MR180"/>
</instance>
<instance part="X1" gate="P" x="-37.592" y="26.67" smashed="yes">
<attribute name="NAME" x="-33.782" y="37.338" size="1.778" layer="95"/>
<attribute name="VALUE" x="-33.782" y="15.748" size="1.778" layer="96"/>
</instance>
<instance part="GND3" gate="1" x="-59.69" y="11.43" smashed="yes">
<attribute name="VALUE" x="-62.23" y="8.89" size="1.778" layer="96"/>
</instance>
<instance part="+3V1" gate="G$1" x="-54.61" y="41.91" smashed="yes">
<attribute name="VALUE" x="-57.15" y="36.83" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND1" gate="1" x="-44.45" y="-19.05" smashed="yes">
<attribute name="VALUE" x="-46.99" y="-21.59" size="1.778" layer="96"/>
</instance>
<instance part="GND2" gate="1" x="30.48" y="5.08" smashed="yes">
<attribute name="VALUE" x="27.94" y="2.54" size="1.778" layer="96"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="+3V3" class="0">
<segment>
<pinref part="X1" gate="P" pin="3"/>
<wire x1="-40.132" y1="31.75" x2="-54.61" y2="31.75" width="0.1524" layer="91"/>
<pinref part="+3V1" gate="G$1" pin="+3V3"/>
<wire x1="-54.61" y1="39.37" x2="-54.61" y2="31.75" width="0.1524" layer="91"/>
<pinref part="X1" gate="P" pin="8"/>
<wire x1="-40.132" y1="19.05" x2="-54.61" y2="19.05" width="0.1524" layer="91"/>
<wire x1="-54.61" y1="19.05" x2="-54.61" y2="31.75" width="0.1524" layer="91"/>
<junction x="-54.61" y="31.75"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="X1" gate="P" pin="7"/>
<pinref part="GND3" gate="1" pin="GND"/>
<wire x1="-59.69" y1="13.97" x2="-59.69" y2="21.59" width="0.1524" layer="91"/>
<wire x1="-59.69" y1="21.59" x2="-40.132" y2="21.59" width="0.1524" layer="91"/>
<pinref part="X1" gate="P" pin="2"/>
<wire x1="-59.69" y1="34.29" x2="-40.132" y2="34.29" width="0.1524" layer="91"/>
<wire x1="-59.69" y1="34.29" x2="-59.69" y2="21.59" width="0.1524" layer="91"/>
<junction x="-59.69" y="21.59"/>
<pinref part="X1" gate="P" pin="S2"/>
<pinref part="X1" gate="P" pin="S1"/>
<wire x1="-35.052" y1="13.97" x2="-37.592" y2="13.97" width="0.1524" layer="91"/>
<wire x1="-37.592" y1="13.97" x2="-59.69" y2="13.97" width="0.1524" layer="91"/>
<junction x="-37.592" y="13.97"/>
<junction x="-59.69" y="13.97"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="GND"/>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="-44.45" y1="-16.51" x2="-44.45" y2="-12.7" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="J2" gate="G$1" pin="8"/>
<pinref part="GND2" gate="1" pin="GND"/>
<wire x1="30.48" y1="7.62" x2="30.48" y2="10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="GPIO" class="0">
<segment>
<pinref part="X1" gate="P" pin="6"/>
<wire x1="-40.132" y1="24.13" x2="-52.07" y2="24.13" width="0.1524" layer="91"/>
<label x="-52.07" y="24.13" size="1.778" layer="95"/>
</segment>
</net>
<net name="SCL" class="0">
<segment>
<pinref part="X1" gate="P" pin="5"/>
<wire x1="-40.132" y1="26.67" x2="-52.07" y2="26.67" width="0.1524" layer="91"/>
<label x="-52.07" y="26.67" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U1" gate="G$2" pin="PB8"/>
<wire x1="20.32" y1="-107.95" x2="3.81" y2="-107.95" width="0.1524" layer="91"/>
<label x="5.08" y="-107.95" size="1.778" layer="95"/>
</segment>
</net>
<net name="SDA" class="0">
<segment>
<pinref part="X1" gate="P" pin="4"/>
<wire x1="-40.132" y1="29.21" x2="-52.07" y2="29.21" width="0.1524" layer="91"/>
<label x="-52.07" y="29.21" size="1.778" layer="95"/>
</segment>
<segment>
<pinref part="U1" gate="G$2" pin="PB9"/>
<wire x1="20.32" y1="-110.49" x2="3.81" y2="-110.49" width="0.1524" layer="91"/>
<label x="5.08" y="-110.49" size="1.778" layer="95"/>
</segment>
</net>
<net name="SPEAKER" class="0">
<segment>
<pinref part="X1" gate="P" pin="1"/>
<wire x1="-40.132" y1="36.83" x2="-51.562" y2="36.83" width="0.1524" layer="91"/>
<label x="-52.07" y="36.83" size="1.778" layer="95"/>
</segment>
</net>
<net name="REFSOUND" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="R"/>
<pinref part="J1" gate="G$1" pin="L"/>
<wire x1="-44.45" y1="-7.62" x2="-44.45" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="-44.45" y1="-2.54" x2="-25.4" y2="-2.54" width="0.1524" layer="91"/>
<junction x="-44.45" y="-2.54"/>
<label x="-39.37" y="-2.54" size="1.778" layer="95"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
