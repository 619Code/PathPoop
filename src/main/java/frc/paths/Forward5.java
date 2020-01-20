package frc.paths;

import com.team319.trajectory.Path;

public class Forward5 extends Path {
   // dt,x,y,left.pos,left.vel,left.acc,left.jerk,center.pos,center.vel,center.acc,center.jerk,right.pos,right.vel,right.acc,right.jerk,heading
	private static final double[][] points = {
				{0.0200,6.0028,0.0000,0.0028,0.1400,7.0000,0.0000,0.0028,0.1400,7.0000,0.0000,0.0028,0.1400,7.0000,0.0000,0.0000},
				{0.0200,6.0084,0.0000,0.0084,0.2800,7.0000,0.0000,0.0084,0.2800,7.0000,0.0000,0.0084,0.2800,7.0000,0.0000,0.0000},
				{0.0200,6.0168,0.0000,0.0168,0.4200,7.0000,-0.0000,0.0168,0.4200,7.0000,0.0000,0.0168,0.4200,7.0000,-0.0000,0.0000},
				{0.0200,6.0280,0.0000,0.0280,0.5600,7.0000,0.0000,0.0280,0.5600,7.0000,0.0000,0.0280,0.5600,7.0000,0.0000,0.0000},
				{0.0200,6.0420,0.0000,0.0420,0.7000,7.0000,-0.0000,0.0420,0.7000,7.0000,0.0000,0.0420,0.7000,7.0000,-0.0000,0.0000},
				{0.0200,6.0588,0.0000,0.0588,0.8400,7.0000,0.0000,0.0588,0.8400,7.0000,0.0000,0.0588,0.8400,7.0000,0.0000,0.0000},
				{0.0200,6.0784,0.0000,0.0784,0.9800,7.0000,-0.0000,0.0784,0.9800,7.0000,0.0000,0.0784,0.9800,7.0000,-0.0000,0.0000},
				{0.0200,6.1008,0.0000,0.1008,1.1200,7.0000,0.0000,0.1008,1.1200,7.0000,0.0000,0.1008,1.1200,7.0000,0.0000,0.0000},
				{0.0200,6.1260,0.0000,0.1260,1.2600,7.0000,0.0000,0.1260,1.2600,7.0000,0.0000,0.1260,1.2600,7.0000,0.0000,0.0000},
				{0.0200,6.1540,0.0000,0.1540,1.4000,7.0000,-0.0000,0.1540,1.4000,7.0000,0.0000,0.1540,1.4000,7.0000,-0.0000,0.0000},
				{0.0200,6.1848,0.0000,0.1848,1.5400,7.0000,-0.0000,0.1848,1.5400,7.0000,0.0000,0.1848,1.5400,7.0000,-0.0000,0.0000},
				{0.0200,6.2184,0.0000,0.2184,1.6800,7.0000,0.0000,0.2184,1.6800,7.0000,0.0000,0.2184,1.6800,7.0000,0.0000,0.0000},
				{0.0200,6.2548,0.0000,0.2548,1.8200,7.0000,0.0000,0.2548,1.8200,7.0000,0.0000,0.2548,1.8200,7.0000,0.0000,0.0000},
				{0.0200,6.2940,0.0000,0.2940,1.9600,7.0000,0.0000,0.2940,1.9600,7.0000,0.0000,0.2940,1.9600,7.0000,0.0000,0.0000},
				{0.0200,6.3360,0.0000,0.3360,2.1000,7.0000,0.0000,0.3360,2.1000,7.0000,0.0000,0.3360,2.1000,7.0000,0.0000,0.0000},
				{0.0200,6.3808,0.0000,0.3808,2.2400,7.0000,-0.0000,0.3808,2.2400,7.0000,0.0000,0.3808,2.2400,7.0000,-0.0000,0.0000},
				{0.0200,6.4284,0.0000,0.4284,2.3800,7.0000,-0.0000,0.4284,2.3800,7.0000,0.0000,0.4284,2.3800,7.0000,-0.0000,0.0000},
				{0.0200,6.4788,0.0000,0.4788,2.5200,7.0000,0.0000,0.4788,2.5200,7.0000,0.0000,0.4788,2.5200,7.0000,0.0000,0.0000},
				{0.0200,6.5320,0.0000,0.5320,2.6600,7.0000,-0.0000,0.5320,2.6600,7.0000,0.0000,0.5320,2.6600,7.0000,-0.0000,0.0000},
				{0.0200,6.5880,0.0000,0.5880,2.8000,7.0000,0.0000,0.5880,2.8000,7.0000,0.0000,0.5880,2.8000,7.0000,0.0000,0.0000},
				{0.0200,6.6468,0.0000,0.6468,2.9400,7.0000,-0.0000,0.6468,2.9400,7.0000,0.0000,0.6468,2.9400,7.0000,-0.0000,0.0000},
				{0.0200,6.7084,0.0000,0.7084,3.0800,7.0000,-0.0000,0.7084,3.0800,7.0000,0.0000,0.7084,3.0800,7.0000,-0.0000,0.0000},
				{0.0200,6.7728,0.0000,0.7728,3.2200,7.0000,0.0000,0.7728,3.2200,7.0000,0.0000,0.7728,3.2200,7.0000,0.0000,0.0000},
				{0.0200,6.8400,0.0000,0.8400,3.3600,7.0000,-0.0000,0.8400,3.3600,7.0000,0.0000,0.8400,3.3600,7.0000,-0.0000,0.0000},
				{0.0200,6.9100,0.0000,0.9100,3.5000,7.0000,0.0000,0.9100,3.5000,7.0000,0.0000,0.9100,3.5000,7.0000,0.0000,0.0000},
				{0.0200,6.9828,0.0000,0.9828,3.6400,7.0000,0.0000,0.9828,3.6400,7.0000,0.0000,0.9828,3.6400,7.0000,0.0000,0.0000},
				{0.0200,7.0584,0.0000,1.0584,3.7800,7.0000,-0.0000,1.0584,3.7800,7.0000,0.0000,1.0584,3.7800,7.0000,-0.0000,0.0000},
				{0.0200,7.1368,0.0000,1.1368,3.9200,7.0000,0.0000,1.1368,3.9200,7.0000,0.0000,1.1368,3.9200,7.0000,0.0000,0.0000},
				{0.0200,7.2180,0.0000,1.2180,4.0600,7.0000,0.0000,1.2180,4.0600,7.0000,0.0000,1.2180,4.0600,7.0000,0.0000,0.0000},
				{0.0200,7.3020,0.0000,1.3020,4.2000,7.0000,-0.0000,1.3020,4.2000,7.0000,0.0000,1.3020,4.2000,7.0000,-0.0000,0.0000},
				{0.0200,7.3888,0.0000,1.3888,4.3400,7.0000,0.0000,1.3888,4.3400,7.0000,0.0000,1.3888,4.3400,7.0000,0.0000,0.0000},
				{0.0200,7.4784,0.0000,1.4784,4.4800,7.0000,0.0000,1.4784,4.4800,7.0000,0.0000,1.4784,4.4800,7.0000,0.0000,0.0000},
				{0.0200,7.5708,0.0000,1.5708,4.6200,7.0000,0.0000,1.5708,4.6200,7.0000,0.0000,1.5708,4.6200,7.0000,0.0000,0.0000},
				{0.0200,7.6660,0.0000,1.6660,4.7600,7.0000,-0.0000,1.6660,4.7600,7.0000,0.0000,1.6660,4.7600,7.0000,-0.0000,0.0000},
				{0.0200,7.7640,0.0000,1.7640,4.9000,7.0000,-0.0000,1.7640,4.9000,7.0000,0.0000,1.7640,4.9000,7.0000,-0.0000,0.0000},
				{0.0200,7.8648,0.0000,1.8648,5.0400,7.0000,0.0000,1.8648,5.0400,7.0000,0.0000,1.8648,5.0400,7.0000,0.0000,0.0000},
				{0.0200,7.9684,0.0000,1.9684,5.1800,7.0000,-0.0000,1.9684,5.1800,7.0000,0.0000,1.9684,5.1800,7.0000,-0.0000,0.0000},
				{0.0200,8.0748,0.0000,2.0748,5.3200,7.0000,0.0000,2.0748,5.3200,7.0000,0.0000,2.0748,5.3200,7.0000,0.0000,0.0000},
				{0.0200,8.1840,0.0000,2.1840,5.4600,7.0000,-0.0000,2.1840,5.4600,7.0000,0.0000,2.1840,5.4600,7.0000,-0.0000,0.0000},
				{0.0200,8.2960,0.0000,2.2960,5.6000,7.0000,-0.0000,2.2960,5.6000,7.0000,0.0000,2.2960,5.6000,7.0000,-0.0000,0.0000},
				{0.0200,8.4108,0.0000,2.4108,5.7400,7.0000,0.0000,2.4108,5.7400,7.0000,0.0000,2.4108,5.7400,7.0000,0.0000,0.0000},
				{0.0200,8.5284,0.0000,2.5284,5.8800,7.0000,0.0000,2.5284,5.8800,7.0000,0.0000,2.5284,5.8800,7.0000,0.0000,0.0000},
				{0.0200,8.6488,0.0000,2.6488,6.0200,7.0000,0.0000,2.6488,6.0200,7.0000,0.0000,2.6488,6.0200,7.0000,0.0000,0.0000},
				{0.0200,8.7720,0.0000,2.7720,6.1600,7.0000,-0.0000,2.7720,6.1600,7.0000,0.0000,2.7720,6.1600,7.0000,-0.0000,0.0000},
				{0.0200,8.8980,0.0000,2.8980,6.3000,7.0000,0.0000,2.8980,6.3000,7.0000,0.0000,2.8980,6.3000,7.0000,0.0000,0.0000},
				{0.0200,9.0268,0.0000,3.0268,6.4400,7.0000,0.0000,3.0268,6.4400,7.0000,0.0000,3.0268,6.4400,7.0000,0.0000,0.0000},
				{0.0200,9.1584,0.0000,3.1584,6.5800,7.0000,0.0000,3.1584,6.5800,7.0000,0.0000,3.1584,6.5800,7.0000,0.0000,0.0000},
				{0.0200,9.2928,0.0000,3.2928,6.7200,7.0000,-0.0000,3.2928,6.7200,7.0000,0.0000,3.2928,6.7200,7.0000,-0.0000,0.0000},
				{0.0200,9.4300,0.0000,3.4300,6.8600,7.0000,0.0000,3.4300,6.8600,7.0000,0.0000,3.4300,6.8600,7.0000,0.0000,0.0000},
				{0.0200,9.5700,0.0000,3.5700,7.0000,7.0000,-0.0000,3.5700,7.0000,7.0000,0.0000,3.5700,7.0000,7.0000,-0.0000,0.0000},
				{0.0200,9.7128,0.0000,3.7128,7.1400,7.0000,0.0000,3.7128,7.1400,7.0000,0.0000,3.7128,7.1400,7.0000,0.0000,0.0000},
				{0.0200,9.8584,0.0000,3.8584,7.2800,7.0000,0.0000,3.8584,7.2800,7.0000,0.0000,3.8584,7.2800,7.0000,0.0000,0.0000},
				{0.0200,10.0068,0.0000,4.0068,7.4200,7.0000,-0.0000,4.0068,7.4200,7.0000,0.0000,4.0068,7.4200,7.0000,-0.0000,0.0000},
				{0.0200,10.1580,0.0000,4.1580,7.5600,7.0000,0.0000,4.1580,7.5600,7.0000,0.0000,4.1580,7.5600,7.0000,0.0000,0.0000},
				{0.0200,10.3120,0.0000,4.3120,7.7000,7.0000,-0.0000,4.3120,7.7000,7.0000,0.0000,4.3120,7.7000,7.0000,-0.0000,0.0000},
				{0.0200,10.4632,0.0000,4.4632,7.5600,-7.0000,-700.0000,4.4632,7.5600,-7.0000,0.0000,4.4632,7.5600,-7.0000,-700.0000,0.0000},
				{0.0200,10.6116,0.0000,4.6116,7.4200,-7.0000,-0.0000,4.6116,7.4200,-7.0000,0.0000,4.6116,7.4200,-7.0000,-0.0000,0.0000},
				{0.0200,10.7572,0.0000,4.7572,7.2800,-7.0000,0.0000,4.7572,7.2800,-7.0000,0.0000,4.7572,7.2800,-7.0000,0.0000,0.0000},
				{0.0200,10.9000,0.0000,4.9000,7.1400,-7.0000,-0.0000,4.9000,7.1400,-7.0000,0.0000,4.9000,7.1400,-7.0000,-0.0000,0.0000},
				{0.0200,11.0000,0.0000,5.0000,5.0000,-107.0000,-5000.0000,5.0000,7.0000,-7.0000,0.0000,5.0000,5.0000,-107.0000,-5000.0000,0.0000},
				{0.0200,11.1428,0.0000,5.1428,7.1400,107.0000,10700.0000,5.1428,7.1400,7.0000,0.0000,5.1428,7.1400,107.0000,10700.0000,0.0000},
				{0.0200,11.2884,0.0000,5.2884,7.2800,7.0000,-5000.0000,5.2884,7.2800,7.0000,0.0000,5.2884,7.2800,7.0000,-5000.0000,0.0000},
				{0.0200,11.4312,0.0000,5.4312,7.1400,-7.0000,-700.0000,5.4312,7.1400,-7.0000,0.0000,5.4312,7.1400,-7.0000,-700.0000,0.0000},
				{0.0200,11.5712,0.0000,5.5712,7.0000,-7.0000,-0.0000,5.5712,7.0000,-7.0000,0.0000,5.5712,7.0000,-7.0000,-0.0000,0.0000},
				{0.0200,11.7084,0.0000,5.7084,6.8600,-7.0000,0.0000,5.7084,6.8600,-7.0000,0.0000,5.7084,6.8600,-7.0000,0.0000,0.0000},
				{0.0200,11.8428,0.0000,5.8428,6.7200,-7.0000,0.0000,5.8428,6.7200,-7.0000,0.0000,5.8428,6.7200,-7.0000,0.0000,0.0000},
				{0.0200,11.9744,0.0000,5.9744,6.5800,-7.0000,0.0000,5.9744,6.5800,-7.0000,0.0000,5.9744,6.5800,-7.0000,0.0000,0.0000},
				{0.0200,12.1032,0.0000,6.1032,6.4400,-7.0000,-0.0000,6.1032,6.4400,-7.0000,0.0000,6.1032,6.4400,-7.0000,-0.0000,0.0000},
				{0.0200,12.2292,0.0000,6.2292,6.3000,-7.0000,0.0000,6.2292,6.3000,-7.0000,0.0000,6.2292,6.3000,-7.0000,0.0000,0.0000},
				{0.0200,12.3524,0.0000,6.3524,6.1600,-7.0000,-0.0000,6.3524,6.1600,-7.0000,0.0000,6.3524,6.1600,-7.0000,-0.0000,0.0000},
				{0.0200,12.4728,0.0000,6.4728,6.0200,-7.0000,0.0000,6.4728,6.0200,-7.0000,0.0000,6.4728,6.0200,-7.0000,0.0000,0.0000},
				{0.0200,12.5904,0.0000,6.5904,5.8800,-7.0000,0.0000,6.5904,5.8800,-7.0000,0.0000,6.5904,5.8800,-7.0000,0.0000,0.0000},
				{0.0200,12.7052,0.0000,6.7052,5.7400,-7.0000,-0.0000,6.7052,5.7400,-7.0000,0.0000,6.7052,5.7400,-7.0000,-0.0000,0.0000},
				{0.0200,12.8172,0.0000,6.8172,5.6000,-7.0000,0.0000,6.8172,5.6000,-7.0000,0.0000,6.8172,5.6000,-7.0000,0.0000,0.0000},
				{0.0200,12.9264,0.0000,6.9264,5.4600,-7.0000,-0.0000,6.9264,5.4600,-7.0000,0.0000,6.9264,5.4600,-7.0000,-0.0000,0.0000},
				{0.0200,13.0328,0.0000,7.0328,5.3200,-7.0000,0.0000,7.0328,5.3200,-7.0000,0.0000,7.0328,5.3200,-7.0000,0.0000,0.0000},
				{0.0200,13.1364,0.0000,7.1364,5.1800,-7.0000,0.0000,7.1364,5.1800,-7.0000,0.0000,7.1364,5.1800,-7.0000,0.0000,0.0000},
				{0.0200,13.2372,0.0000,7.2372,5.0400,-7.0000,-0.0000,7.2372,5.0400,-7.0000,0.0000,7.2372,5.0400,-7.0000,-0.0000,0.0000},
				{0.0200,13.3352,0.0000,7.3352,4.9000,-7.0000,-0.0000,7.3352,4.9000,-7.0000,0.0000,7.3352,4.9000,-7.0000,-0.0000,0.0000},
				{0.0200,13.4304,0.0000,7.4304,4.7600,-7.0000,0.0000,7.4304,4.7600,-7.0000,0.0000,7.4304,4.7600,-7.0000,0.0000,0.0000},
				{0.0200,13.5228,0.0000,7.5228,4.6200,-7.0000,-0.0000,7.5228,4.6200,-7.0000,0.0000,7.5228,4.6200,-7.0000,-0.0000,0.0000},
				{0.0200,13.6124,0.0000,7.6124,4.4800,-7.0000,0.0000,7.6124,4.4800,-7.0000,0.0000,7.6124,4.4800,-7.0000,0.0000,0.0000},
				{0.0200,13.6992,0.0000,7.6992,4.3400,-7.0000,0.0000,7.6992,4.3400,-7.0000,0.0000,7.6992,4.3400,-7.0000,0.0000,0.0000},
				{0.0200,13.7832,0.0000,7.7832,4.2000,-7.0000,0.0000,7.7832,4.2000,-7.0000,0.0000,7.7832,4.2000,-7.0000,0.0000,0.0000},
				{0.0200,13.8644,0.0000,7.8644,4.0600,-7.0000,-0.0000,7.8644,4.0600,-7.0000,0.0000,7.8644,4.0600,-7.0000,-0.0000,0.0000},
				{0.0200,13.9428,0.0000,7.9428,3.9200,-7.0000,0.0000,7.9428,3.9200,-7.0000,0.0000,7.9428,3.9200,-7.0000,0.0000,0.0000},
				{0.0200,14.0184,0.0000,8.0184,3.7800,-7.0000,-0.0000,8.0184,3.7800,-7.0000,0.0000,8.0184,3.7800,-7.0000,-0.0000,0.0000},
				{0.0200,14.0912,0.0000,8.0912,3.6400,-7.0000,0.0000,8.0912,3.6400,-7.0000,0.0000,8.0912,3.6400,-7.0000,0.0000,0.0000},
				{0.0200,14.1612,0.0000,8.1612,3.5000,-7.0000,-0.0000,8.1612,3.5000,-7.0000,0.0000,8.1612,3.5000,-7.0000,-0.0000,0.0000},
				{0.0200,14.2284,0.0000,8.2284,3.3600,-7.0000,0.0000,8.2284,3.3600,-7.0000,0.0000,8.2284,3.3600,-7.0000,0.0000,0.0000},
				{0.0200,14.2928,0.0000,8.2928,3.2200,-7.0000,0.0000,8.2928,3.2200,-7.0000,0.0000,8.2928,3.2200,-7.0000,0.0000,0.0000},
				{0.0200,14.3544,0.0000,8.3544,3.0800,-7.0000,-0.0000,8.3544,3.0800,-7.0000,0.0000,8.3544,3.0800,-7.0000,-0.0000,0.0000},
				{0.0200,14.4132,0.0000,8.4132,2.9400,-7.0000,0.0000,8.4132,2.9400,-7.0000,0.0000,8.4132,2.9400,-7.0000,0.0000,0.0000},
				{0.0200,14.4692,0.0000,8.4692,2.8000,-7.0000,-0.0000,8.4692,2.8000,-7.0000,0.0000,8.4692,2.8000,-7.0000,-0.0000,0.0000},
				{0.0200,14.5224,0.0000,8.5224,2.6600,-7.0000,0.0000,8.5224,2.6600,-7.0000,0.0000,8.5224,2.6600,-7.0000,0.0000,0.0000},
				{0.0200,14.5728,0.0000,8.5728,2.5200,-7.0000,0.0000,8.5728,2.5200,-7.0000,0.0000,8.5728,2.5200,-7.0000,0.0000,0.0000},
				{0.0200,14.6204,0.0000,8.6204,2.3800,-7.0000,-0.0000,8.6204,2.3800,-7.0000,0.0000,8.6204,2.3800,-7.0000,-0.0000,0.0000},
				{0.0200,14.6652,0.0000,8.6652,2.2400,-7.0000,0.0000,8.6652,2.2400,-7.0000,0.0000,8.6652,2.2400,-7.0000,0.0000,0.0000},
				{0.0200,14.7072,0.0000,8.7072,2.1000,-7.0000,0.0000,8.7072,2.1000,-7.0000,0.0000,8.7072,2.1000,-7.0000,0.0000,0.0000},
				{0.0200,14.7464,0.0000,8.7464,1.9600,-7.0000,-0.0000,8.7464,1.9600,-7.0000,0.0000,8.7464,1.9600,-7.0000,-0.0000,0.0000},
				{0.0200,14.7828,0.0000,8.7828,1.8200,-7.0000,0.0000,8.7828,1.8200,-7.0000,0.0000,8.7828,1.8200,-7.0000,0.0000,0.0000},
				{0.0200,14.8164,0.0000,8.8164,1.6800,-7.0000,0.0000,8.8164,1.6800,-7.0000,0.0000,8.8164,1.6800,-7.0000,0.0000,0.0000},
				{0.0200,14.8472,0.0000,8.8472,1.5400,-7.0000,0.0000,8.8472,1.5400,-7.0000,0.0000,8.8472,1.5400,-7.0000,0.0000,0.0000},
				{0.0200,14.8752,0.0000,8.8752,1.4000,-7.0000,0.0000,8.8752,1.4000,-7.0000,0.0000,8.8752,1.4000,-7.0000,0.0000,0.0000},
				{0.0200,14.9004,0.0000,8.9004,1.2600,-7.0000,0.0000,8.9004,1.2600,-7.0000,0.0000,8.9004,1.2600,-7.0000,0.0000,0.0000},
				{0.0200,14.9228,0.0000,8.9228,1.1200,-7.0000,-0.0000,8.9228,1.1200,-7.0000,0.0000,8.9228,1.1200,-7.0000,-0.0000,0.0000},
				{0.0200,14.9424,0.0000,8.9424,0.9800,-7.0000,0.0000,8.9424,0.9800,-7.0000,0.0000,8.9424,0.9800,-7.0000,0.0000,0.0000},
				{0.0200,14.9592,0.0000,8.9592,0.8400,-7.0000,0.0000,8.9592,0.8400,-7.0000,0.0000,8.9592,0.8400,-7.0000,0.0000,0.0000},
				{0.0200,14.9732,0.0000,8.9732,0.7000,-7.0000,-0.0000,8.9732,0.7000,-7.0000,0.0000,8.9732,0.7000,-7.0000,-0.0000,0.0000},
				{0.0200,14.9844,0.0000,8.9844,0.5600,-7.0000,0.0000,8.9844,0.5600,-7.0000,0.0000,8.9844,0.5600,-7.0000,0.0000,0.0000},
				{0.0200,14.9928,0.0000,8.9928,0.4200,-7.0000,0.0000,8.9928,0.4200,-7.0000,0.0000,8.9928,0.4200,-7.0000,0.0000,0.0000},
				{0.0200,14.9984,0.0000,8.9984,0.2800,-7.0000,-0.0000,8.9984,0.2800,-7.0000,0.0000,8.9984,0.2800,-7.0000,-0.0000,0.0000},
				{0.0200,15.0000,0.0000,9.0000,0.0800,-10.0000,-150.0000,9.0000,0.1400,-7.0000,0.0000,9.0000,0.0800,-10.0000,-150.0000,0.0000},
				{0.0200,15.0000,0.0000,9.0000,0.0000,-4.0000,300.0000,9.0000,-0.0000,-7.0000,0.0000,9.0000,0.0000,-4.0000,300.0000,0.0000},

	    };

	@Override
	public double[][] getPath() {
	    return points;
	}
}