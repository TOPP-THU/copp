//! Franka Panda no-viscous RobotTorque integration model.

#![allow(clippy::float_cmp)]
#![allow(clippy::many_single_char_names)]
#![allow(clippy::similar_names)]
#![allow(clippy::too_many_lines)]
#![allow(unused_variables)]

use copp::diag::RobotDynamicsError;
use copp::path::{Path, SplineConfig};
use copp::robot::{Robot, RobotBasic, RobotTorque};
use copp::solver::copp3_socp::{
    ClarabelOptionsBuilder, Copp3ProblemBuilder, CoppObjective, Topp3Profile, copp3_socp,
    s_to_t_topp3,
};
use copp::solver::topp2_ra::{ReachSet2OptionsBuilder, Topp2ProblemBuilder, topp2_ra};
use nalgebra::DMatrix;

const FRANKA_DOF: usize = 7;
const FRANKA_DYN_PARAMS: usize = 84;
const REGRESSOR_LEN: usize = FRANKA_DOF * FRANKA_DYN_PARAMS;
const SIGN_ZERO: f64 = 1E-16;

const FRANKA_NONZERO_PARAMS: &[(usize, f64)] = &[
    (5, 0.035473883494381714),    // L_1zz
    (10, 0.30140334744298314),    // fc_1
    (11, -0.13815427970846927),   // fo_1
    (12, 0.018669319423928757),   // L_2xx
    (13, 0.07737916929335538),    // L_2xy
    (14, 0.041666286147674474),   // L_2xz
    (15, 0.016804561511051294),   // L_2yy
    (16, 0.014206431257405581),   // L_2yz
    (17, 0.003410423235356195),   // L_2zz
    (18, 0.025575261601316145),   // l_2x
    (19, -1.2086485340687692),    // l_2y
    (20, 3.232471231915263e-11),  // l_2z
    (22, 0.21778182206773414),    // fc_2
    (23, -0.29526747213479715),   // fo_2
    (24, -0.015804370065835363),  // L_3xx
    (25, -0.014858578946456787),  // L_3xy
    (26, -0.08327215408099714),   // L_3xz
    (27, 0.03788404357090989),    // L_3yy
    (28, -0.013340631361542118),  // L_3yz
    (29, 0.05098501548556614),    // L_3zz
    (30, 0.5587422316156684),     // l_3x
    (31, -5.512913734623079e-05), // l_3y
    (32, 1.2226029270025434),     // l_3z
    (33, 0.38413755079719464),    // m_3
    (34, 0.15604364055134876),    // fc_3
    (35, -0.42031007270432036),   // fo_3
    (36, 0.04229476153319518),    // L_4xx
    (37, 0.060018840051008805),   // L_4xy
    (38, -0.015172241103477887),  // L_4xz
    (39, -0.00711403401002513),   // L_4yy
    (40, -0.00727657515634638),   // L_4yz
    (41, -0.08530497463731969),   // L_4zz
    (42, -0.3668476813631923),    // l_4x
    (43, 0.6292650881566902),     // l_4y
    (44, -0.0011706396498078672), // l_4z
    (45, 0.4308386698670209),     // m_4
    (46, 0.3140099737804228),     // fc_4
    (47, 0.07035455327891861),    // fo_4
    (48, -0.013946364904734082),  // L_5xx
    (49, -0.018378282214343785),  // L_5xy
    (50, 0.008085152404524468),   // L_5xz
    (51, -0.029063834415448346),  // L_5yy
    (52, 0.004160134527148216),   // L_5yz
    (53, 0.01784555753591962),    // L_5zz
    (54, -0.0027902442480020662), // l_5x
    (55, 0.043590431640450227),   // l_5y
    (56, 0.6011850730515639),     // l_5z
    (57, 0.6976709371327539),     // m_5
    (58, 0.31187474515284813),    // fc_5
    (59, 0.07195963317228392),    // fo_5
    (60, -0.0034108003348159617), // L_6xx
    (61, 0.012160928474609482),   // L_6xy
    (62, -0.0006124231864168001), // L_6xz
    (63, 0.007310052353274886),   // L_6yy
    (64, 0.010196791712246097),   // L_6yz
    (65, 0.02347025059505411),    // L_6zz
    (66, 0.16017708326258945),    // l_6x
    (67, -0.0850020538209929),    // l_6y
    (68, -0.043590556598582256),  // l_6z
    (69, 0.6976709195639503),     // m_6
    (70, 0.18370860552173787),    // fc_6
    (71, 0.011213824433469872),   // fo_6
    (72, 0.010800684510719246),   // L_7xx
    (73, -0.00287984378506688),   // L_7xy
    (74, 0.000334284615285064),   // L_7xz
    (75, 0.009258656683121867),   // L_7yy
    (76, -0.005205308010135958),  // L_7yz
    (77, -0.0037129604059085246), // L_7zz
    (78, 0.005023149607552673),   // l_7x
    (79, 0.008432894116476412),   // l_7y
    (80, 0.0860722682536253),     // l_7z
    (81, 0.7120057271794441),     // m_7
    (82, 0.24968343971117432),    // fc_7
    (83, 0.0299872654965484),     // fo_7
];

#[derive(Clone, Copy, Debug, Default)]
struct FrankaNoViscousModel;

impl RobotBasic for FrankaNoViscousModel {
    #[inline(always)]
    fn dim(&self) -> usize {
        FRANKA_DOF
    }
}

impl RobotTorque for FrankaNoViscousModel {
    fn inverse_dynamics(
        &self,
        q: &[f64],
        dq: &[f64],
        ddq: &[f64],
        tau: &mut [f64],
    ) -> Result<(), RobotDynamicsError> {
        inverse_dynamics_franka_no_viscous(q, dq, ddq, tau);
        Ok(())
    }
}

#[inline(always)]
fn assert_dof_slice(name: &str, values: &[f64]) {
    assert_eq!(
        values.len(),
        FRANKA_DOF,
        "{name} must have length {FRANKA_DOF}"
    );
}

fn inverse_dynamics_franka_no_viscous(q: &[f64], dq: &[f64], ddq: &[f64], tau: &mut [f64]) {
    assert_dof_slice("q", q);
    assert_dof_slice("dq", dq);
    assert_dof_slice("ddq", ddq);
    assert_dof_slice("tau", tau);

    let h = franka_no_viscous_regressor(q, dq, ddq);
    for (joint, tau_joint) in tau.iter_mut().enumerate() {
        let row_start = joint * FRANKA_DYN_PARAMS;
        *tau_joint = FRANKA_NONZERO_PARAMS
            .iter()
            .map(|&(idx, param)| h[row_start + idx] * param)
            .sum();
    }
}
fn franka_no_viscous_regressor(q: &[f64], dq: &[f64], ddq: &[f64]) -> [f64; REGRESSOR_LEN] {
    assert_dof_slice("q", q);
    assert_dof_slice("dq", dq);
    assert_dof_slice("ddq", ddq);
    let mut h = [0.0_f64; REGRESSOR_LEN];
    let x0 = q[1].sin();
    let x1 = -ddq[0];
    let x2 = q[1].cos();
    let x3 = -dq[0];
    let x4 = x2 * x3;
    let x5 = dq[1] * x4;
    let x6 = x0 * x1 + x5;
    let x7 = -x6;
    let x8 = x0 * x3;
    let x9 = dq[1] * x8;
    let x10 = -x9;
    let x11 = x10 * x2;
    let x12 = dq[0] * dq[1] * x0 + x1 * x2;
    let x13 = -x0;
    let x14 = -x2;
    let x15 = x5 + x6;
    let x16 = x4 * x8;
    let x17 = dq[1] * dq[1];
    let x18 = x8 * x8;
    let x19 = -x18;
    let x20 = -x12;
    let x21 = -x5;
    let x22 = -x17;
    let x23 = x4 * x4;
    let x24 = -x16;
    let x25 = -9.81 * x0;
    let x26 = -9.81 * x2;
    let x27 = -x26;
    let x28 = q[2].cos();
    let x29 = q[2].sin();
    let x30 = -x29;
    let x31 = dq[1] * x28 + x30 * x8;
    let x32 = ddq[1] * x29 + dq[2] * x31 + x28 * x6;
    let x33 = dq[1] * x29 + x28 * x8;
    let x34 = dq[2] - x4;
    let x35 = x33 * x34;
    let x36 = x31 * x33;
    let x37 = -x36;
    let x38 = -x35;
    let x39 = -x33;
    let x40 = ddq[1] * x28 + dq[2] * x39 + x29 * x7;
    let x41 = x38 + x40;
    let x42 = x31 * x34;
    let x43 = x32 + x42;
    let x44 = x31 * x31;
    let x45 = -x44;
    let x46 = x33 * x33;
    let x47 = x45 + x46;
    let x48 = ddq[2] + x20;
    let x49 = x36 + x48;
    let x50 = x28 * x49;
    let x51 = x34 * x34;
    let x52 = -x46;
    let x53 = x51 + x52;
    let x54 = -x42;
    let x55 = x32 + x54;
    let x56 = -x51;
    let x57 = x44 + x56;
    let x58 = x37 + x48;
    let x59 = x35 + x40;
    let x60 = -x48;
    let x61 = 0.316 * ddq[1] - 0.316 * x16 + x25;
    let x62 = -0.316 * x15;
    let x63 = x28 * x62 + x30 * x61;
    let x64 = -x63;
    let x65 = 0.316 * x19 + 0.316 * x22 + x27;
    let x66 = -x65;
    let x67 = x45 + x56;
    let x68 = x36 + x60;
    let x69 = x52 + x56;
    let x70 = x28 * x61 + x29 * x62;
    let x71 = -x70;
    let x72 = -x32;
    let x73 = x42 + x72;
    let x74 = -0.316 * x29;
    let x75 = q[3].cos();
    let x76 = q[3].sin();
    let x77 = x34 * x75 + x39 * x76;
    let x78 = dq[3] * x77 + x32 * x75 + x48 * x76;
    let x79 = x33 * x75 + x34 * x76;
    let x80 = dq[3] - x31;
    let x81 = x79 * x80;
    let x82 = -x76;
    let x83 = x75 * x78 + x81 * x82;
    let x84 = x77 * x79;
    let x85 = -x84;
    let x86 = -x85;
    let x87 = x75 * x81 + x76 * x78;
    let x88 = -x79;
    let x89 = dq[3] * x88 + x48 * x75 + x72 * x76;
    let x90 = -x81;
    let x91 = x89 + x90;
    let x92 = x77 * x80;
    let x93 = x78 + x92;
    let x94 = x75 * x91 + x82 * x93;
    let x95 = x77 * x77;
    let x96 = -x95;
    let x97 = x79 * x79;
    let x98 = x96 + x97;
    let x99 = -x98;
    let x100 = x75 * x93 + x76 * x91;
    let x101 = -x40;
    let x102 = ddq[3] + x101;
    let x103 = x102 + x84;
    let x104 = x80 * x80;
    let x105 = -x97;
    let x106 = x104 + x105;
    let x107 = x103 * x75 + x106 * x82;
    let x108 = -x92;
    let x109 = x108 + x78;
    let x110 = -x109;
    let x111 = x103 * x76 + x106 * x75;
    let x112 = -x89;
    let x113 = x108 * x75 + x112 * x76;
    let x114 = x108 * x76 + x75 * x89;
    let x115 = -x104;
    let x116 = x115 + x95;
    let x117 = x102 + x85;
    let x118 = x116 * x75 + x117 * x82;
    let x119 = x81 + x89;
    let x120 = -x119;
    let x121 = x116 * x76 + x117 * x75;
    let x122 = x75 * x92 + x82 * x90;
    let x123 = -x102;
    let x124 = x75 * x90 + x76 * x92;
    let x125 = -0.0825 * x49 + x64;
    let x126 = -x125;
    let x127 = x126 * x82;
    let x128 = 0.0825 * x101 + 0.0825 * x35 + x65;
    let x129 = 0.0825 * x67 + x70;
    let x130 = x128 * x75 + x129 * x82;
    let x131 = -x130;
    let x132 = x115 + x96;
    let x133 = -0.0825 * x132;
    let x134 = -0.0825 * x103;
    let x135 = x131 + x133 * x76 + x134 * x75;
    let x136 = x103 * x82 + x132 * x75;
    let x137 = x112 + x81;
    let x138 = -x137;
    let x139 = -0.0825 * x137;
    let x140 = x126 * x75 + x139;
    let x141 = x125 * x75;
    let x142 = x128 * x76 + x129 * x75;
    let x143 = -x142;
    let x144 = x123 + x84;
    let x145 = x105 + x115;
    let x146 = -x143 - 0.0825 * x144 * x76 - 0.0825 * x145 * x75;
    let x147 = x144 * x75 + x145 * x82;
    let x148 = -x93;
    let x149 = x125 * x76 - 0.0825 * x93;
    let x150 = x131 * x75 + x142 * x82;
    let x151 = -x78;
    let x152 = x151 + x92;
    let x153 = -0.0825 * x119 * x76 - 0.0825 * x152 * x75;
    let x154 = x119 * x75 + x152 * x82;
    let x155 = x105 + x96;
    let x156 = -x155;
    let x157 = x142 * x75;
    let x158 = x131 * x76 - 0.0825 * x155 + x157;
    let x159 = -0.0825 * x130 * x75 - 0.0825 * x142 * x76;
    let x160 = x130 * x82 + x157;
    let x161 = -0.0825 * x125;
    let x162 = q[4].cos();
    let x163 = q[4].sin();
    let x164 = -x80;
    let x165 = x162 * x164 + x163 * x88;
    let x166 = dq[4] * x165 + x123 * x163 + x162 * x78;
    let x167 = -x163;
    let x168 = x162 * x79 + x163 * x164;
    let x169 = dq[4] + x77;
    let x170 = x168 * x169;
    let x171 = x162 * x166 + x167 * x170;
    let x172 = x165 * x168;
    let x173 = -x172;
    let x174 = x171 * x75 + x173 * x82;
    let x175 = -x162;
    let x176 = x166 * x167 + x170 * x175;
    let x177 = -x176;
    let x178 = x171 * x76 + x173 * x75;
    let x179 = -x168;
    let x180 = dq[4] * x179 + x123 * x162 + x151 * x163;
    let x181 = -x170;
    let x182 = x180 + x181;
    let x183 = x165 * x169;
    let x184 = x166 + x183;
    let x185 = x162 * x182 + x167 * x184;
    let x186 = x165 * x165;
    let x187 = -x186;
    let x188 = x168 * x168;
    let x189 = x187 + x188;
    let x190 = x185 * x75 + x189 * x82;
    let x191 = x167 * x182 + x175 * x184;
    let x192 = -x191;
    let x193 = x185 * x76 + x189 * x75;
    let x194 = ddq[4] + x89;
    let x195 = x172 + x194;
    let x196 = x162 * x195;
    let x197 = x169 * x169;
    let x198 = -x188;
    let x199 = x197 + x198;
    let x200 = x167 * x199 + x196;
    let x201 = -x183;
    let x202 = x166 + x201;
    let x203 = x200 * x75 + x202 * x82;
    let x204 = x167 * x195;
    let x205 = x175 * x199 + x204;
    let x206 = -x205;
    let x207 = x200 * x76 + x202 * x75;
    let x208 = x162 * x201 + x167 * x180;
    let x209 = x172 * x82 + x208 * x75;
    let x210 = -x180;
    let x211 = x162 * x210 + x167 * x201;
    let x212 = -x211;
    let x213 = x172 * x75 + x208 * x76;
    let x214 = -x197;
    let x215 = x186 + x214;
    let x216 = x173 + x194;
    let x217 = x162 * x215 + x167 * x216;
    let x218 = x170 + x180;
    let x219 = x217 * x75 + x218 * x82;
    let x220 = x167 * x215 + x175 * x216;
    let x221 = -x220;
    let x222 = x217 * x76 + x218 * x75;
    let x223 = x162 * x183 + x167 * x181;
    let x224 = -x194;
    let x225 = x223 * x75 + x224 * x76;
    let x226 = x167 * x183 + x175 * x181;
    let x227 = -x226;
    let x228 = x194 * x75 + x223 * x76;
    let x229 = x130 + x134 + 0.384 * x145;
    let x230 = -x229;
    let x231 = x187 + x214;
    let x232 = x163 * x231;
    let x233 = x167 * x230 - 0.384 * x196 - 0.384 * x232;
    let x234 = x133 + x142 + 0.384 * x144;
    let x235 = x125 + x139 + 0.384 * x93;
    let x236 = x167 * x234 + x175 * x235;
    let x237 = -0.0825 * x196 - 0.0825 * x232 + x236;
    let x238 = x233 * x75 + x237 * x82;
    let x239 = x170 + x210;
    let x240 = -0.0825 * x239;
    let x241 = 0.384 * x163;
    let x242 = x162 * x231;
    let x243 = x175 * x230 + x195 * x241 + x240 - 0.384 * x242;
    let x244 = x204 + x242;
    let x245 = x240 * x75 - x243 - 0.0825 * x244 * x76;
    let x246 = x239 * x82 + x244 * x75;
    let x247 = x167 * x231 + x175 * x195;
    let x248 = -x247;
    let x249 = x233 * x76 + x237 * x75 - 0.0825 * x247;
    let x250 = x172 + x224;
    let x251 = x163 * x250;
    let x252 = x198 + x214;
    let x253 = x162 * x252;
    let x254 = x162 * x229 - 0.384 * x251 - 0.384 * x253;
    let x255 = x162 * x234 + x167 * x235;
    let x256 = -x255;
    let x257 = -0.0825 * x251 - 0.0825 * x253 + x256;
    let x258 = x254 * x75 + x257 * x82;
    let x259 = -0.0825 * x184;
    let x260 = x162 * x250;
    let x261 = x167 * x229 + x241 * x252 + x259 - 0.384 * x260;
    let x262 = x167 * x252 + x260;
    let x263 = x259 * x75 - x261 - 0.0825 * x262 * x76;
    let x264 = x184 * x82 + x262 * x75;
    let x265 = x167 * x250 + x175 * x252;
    let x266 = -x265;
    let x267 = x254 * x76 + x257 * x75 - 0.0825 * x265;
    let x268 = x163 * x218;
    let x269 = -x166;
    let x270 = x183 + x269;
    let x271 = x162 * x270;
    let x272 = -x236;
    let x273 = x162 * x272 + x167 * x255;
    let x274 = -0.384 * x268 - 0.384 * x271 + x273;
    let x275 = -0.0825 * x268 - 0.0825 * x271;
    let x276 = x274 * x75 + x275 * x82;
    let x277 = x187 + x198;
    let x278 = -0.0825 * x277;
    let x279 = x162 * x218;
    let x280 = x167 * x272 + x175 * x255 + x241 * x270 + x278 - 0.384 * x279;
    let x281 = x167 * x270 + x279;
    let x282 = x278 * x75 - x280 - 0.0825 * x281 * x76;
    let x283 = x277 * x82 + x281 * x75;
    let x284 = x167 * x218 + x175 * x270;
    let x285 = -x284;
    let x286 = x274 * x76 + x275 * x75 - 0.0825 * x284;
    let x287 = x163 * x255;
    let x288 = x162 * x236;
    let x289 = -0.384 * x287 - 0.384 * x288;
    let x290 = -0.0825 * x287 - 0.0825 * x288;
    let x291 = x289 * x75 + x290 * x82;
    let x292 = -0.0825 * x229;
    let x293 = x162 * x255;
    let x294 = x236 * x241 + x292 - 0.384 * x293;
    let x295 = x167 * x236 + x293;
    let x296 = x292 * x75 - x294 - 0.0825 * x295 * x76;
    let x297 = x229 * x82 + x295 * x75;
    let x298 = -x273;
    let x299 = -0.0825 * x273 + x289 * x76 + x290 * x75;
    let x300 = q[5].cos();
    let x301 = q[5].sin();
    let x302 = x169 * x300 + x179 * x301;
    let x303 = dq[5] * x302 + x166 * x300 + x194 * x301;
    let x304 = -x301;
    let x305 = x168 * x300 + x169 * x301;
    let x306 = dq[5] - x165;
    let x307 = x305 * x306;
    let x308 = x300 * x303 + x304 * x307;
    let x309 = x302 * x305;
    let x310 = -x309;
    let x311 = -x310;
    let x312 = x162 * x308 + x167 * x311;
    let x313 = x300 * x307 + x301 * x303;
    let x314 = x312 * x75 + x313 * x82;
    let x315 = x167 * x308 + x175 * x311;
    let x316 = -x315;
    let x317 = x312 * x76 + x313 * x75;
    let x318 = -x307;
    let x319 = -x305;
    let x320 = dq[5] * x319 + x194 * x300 + x269 * x301;
    let x321 = x318 + x320;
    let x322 = x302 * x306;
    let x323 = x303 + x322;
    let x324 = x300 * x321 + x304 * x323;
    let x325 = x302 * x302;
    let x326 = -x325;
    let x327 = x305 * x305;
    let x328 = x326 + x327;
    let x329 = -x328;
    let x330 = x162 * x324 + x167 * x329;
    let x331 = x300 * x323 + x301 * x321;
    let x332 = x330 * x75 + x331 * x82;
    let x333 = x167 * x324 + x175 * x329;
    let x334 = -x333;
    let x335 = x330 * x76 + x331 * x75;
    let x336 = ddq[5] + x210;
    let x337 = x309 + x336;
    let x338 = x300 * x337;
    let x339 = x306 * x306;
    let x340 = -x327;
    let x341 = x339 + x340;
    let x342 = x304 * x341 + x338;
    let x343 = -x322;
    let x344 = x303 + x343;
    let x345 = -x344;
    let x346 = x162 * x342 + x167 * x345;
    let x347 = x300 * x341 + x301 * x337;
    let x348 = x346 * x75 + x347 * x82;
    let x349 = x167 * x342 + x175 * x345;
    let x350 = -x349;
    let x351 = x346 * x76 + x347 * x75;
    let x352 = -x320;
    let x353 = x300 * x343 + x301 * x352;
    let x354 = x162 * x353 + x167 * x310;
    let x355 = x300 * x320 + x301 * x343;
    let x356 = x354 * x75 + x355 * x82;
    let x357 = x167 * x353 + x175 * x310;
    let x358 = -x357;
    let x359 = x354 * x76 + x355 * x75;
    let x360 = -x339;
    let x361 = x325 + x360;
    let x362 = x310 + x336;
    let x363 = x300 * x361 + x304 * x362;
    let x364 = x307 + x320;
    let x365 = -x364;
    let x366 = x162 * x363 + x167 * x365;
    let x367 = x300 * x362 + x301 * x361;
    let x368 = x366 * x75 + x367 * x82;
    let x369 = x167 * x363 + x175 * x365;
    let x370 = -x369;
    let x371 = x366 * x76 + x367 * x75;
    let x372 = x300 * x322 + x304 * x318;
    let x373 = -x336;
    let x374 = x162 * x372 + x167 * x373;
    let x375 = x300 * x318 + x301 * x322;
    let x376 = x374 * x75 + x375 * x82;
    let x377 = x167 * x372 + x175 * x373;
    let x378 = -x377;
    let x379 = x374 * x76 + x375 * x75;
    let x380 = -x272;
    let x381 = x304 * x380;
    let x382 = x229 * x300 + x256 * x301;
    let x383 = -x382;
    let x384 = x326 + x360;
    let x385 = x300 * x384 + x304 * x337;
    let x386 = x163 * x385;
    let x387 = x307 + x352;
    let x388 = -x387;
    let x389 = x162 * x388;
    let x390 = x162 * x381 + x167 * x383 - 0.384 * x386 - 0.384 * x389;
    let x391 = x300 * x380;
    let x392 = -0.0825 * x386 - 0.0825 * x389 + x391;
    let x393 = x390 * x75 + x392 * x82;
    let x394 = x301 * x384 + x338;
    let x395 = -0.0825 * x394;
    let x396 = x162 * x385;
    let x397 = x167 * x381 + x175 * x383 + x241 * x388 + x395 - 0.384 * x396;
    let x398 = x167 * x388 + x396;
    let x399 = x395 * x75 - x397 - 0.0825 * x398 * x76;
    let x400 = x394 * x82 + x398 * x75;
    let x401 = x167 * x385 + x175 * x388;
    let x402 = -x401;
    let x403 = x390 * x76 + x392 * x75 - 0.0825 * x401;
    let x404 = x272 * x300;
    let x405 = x229 * x301 + x255 * x300;
    let x406 = -x405;
    let x407 = -x406;
    let x408 = x309 + x373;
    let x409 = x340 + x360;
    let x410 = x300 * x408 + x304 * x409;
    let x411 = x163 * x410;
    let x412 = -x323;
    let x413 = x162 * x412;
    let x414 = x162 * x404 + x167 * x407 - 0.384 * x411 - 0.384 * x413;
    let x415 = x272 * x301;
    let x416 = -0.0825 * x411 - 0.0825 * x413 + x415;
    let x417 = x414 * x75 + x416 * x82;
    let x418 = x300 * x409 + x301 * x408;
    let x419 = -0.0825 * x418;
    let x420 = x162 * x410;
    let x421 = x167 * x404 + x175 * x407 + x241 * x412 + x419 - 0.384 * x420;
    let x422 = x167 * x412 + x420;
    let x423 = x419 * x75 - x421 - 0.0825 * x422 * x76;
    let x424 = x418 * x82 + x422 * x75;
    let x425 = x167 * x410 + x175 * x412;
    let x426 = -x425;
    let x427 = x414 * x76 + x416 * x75 - 0.0825 * x425;
    let x428 = x300 * x383 + x304 * x405;
    let x429 = -x303;
    let x430 = x322 + x429;
    let x431 = x300 * x364 + x304 * x430;
    let x432 = x163 * x431;
    let x433 = -x326 - x340;
    let x434 = x162 * x433;
    let x435 = x162 * x428 - 0.384 * x432 - 0.384 * x434;
    let x436 = x300 * x405 + x301 * x383;
    let x437 = -0.0825 * x432 - 0.0825 * x434 + x436;
    let x438 = x435 * x75 + x437 * x82;
    let x439 = x300 * x430 + x301 * x364;
    let x440 = -0.0825 * x439;
    let x441 = x162 * x431;
    let x442 = x167 * x428 + x241 * x433 + x440 - 0.384 * x441;
    let x443 = x167 * x433 + x441;
    let x444 = x440 * x75 - x442 - 0.0825 * x443 * x76;
    let x445 = x439 * x82 + x443 * x75;
    let x446 = x167 * x431 + x175 * x433;
    let x447 = -x446;
    let x448 = x435 * x76 + x437 * x75 - 0.0825 * x446;
    let x449 = x162 * x380;
    let x450 = x163 * x436;
    let x451 = -0.384 * x449 - 0.384 * x450;
    let x452 = -0.0825 * x449 - 0.0825 * x450;
    let x453 = x451 * x75 + x452 * x82;
    let x454 = x300 * x382 + x301 * x405;
    let x455 = -0.0825 * x454;
    let x456 = x162 * x436;
    let x457 = x241 * x380 + x455 - 0.384 * x456;
    let x458 = x167 * x380 + x456;
    let x459 = x455 * x75 - x457 - 0.0825 * x458 * x76;
    let x460 = x454 * x82 + x458 * x75;
    let x461 = x167 * x436 + x175 * x380;
    let x462 = -x461;
    let x463 = x451 * x76 + x452 * x75 - 0.0825 * x461;
    let x464 = q[6].cos();
    let x465 = q[6].sin();
    let x466 = x306 * x464 + x319 * x465;
    let x467 = dq[6] * x466 + x303 * x464 + x336 * x465;
    let x468 = -x465;
    let x469 = x305 * x464 + x306 * x465;
    let x470 = dq[6] - x302;
    let x471 = x469 * x470;
    let x472 = x464 * x467 + x468 * x471;
    let x473 = x466 * x469;
    let x474 = -x473;
    let x475 = -x474;
    let x476 = x300 * x472 + x304 * x475;
    let x477 = x464 * x471 + x465 * x467;
    let x478 = -x477;
    let x479 = x162 * x476 + x167 * x478;
    let x480 = x300 * x475 + x301 * x472;
    let x481 = x479 * x75 + x480 * x82;
    let x482 = x167 * x476 + x175 * x478;
    let x483 = -x482;
    let x484 = x479 * x76 + x480 * x75;
    let x485 = -x471;
    let x486 = -dq[6] * x469 + x336 * x464 + x429 * x465;
    let x487 = x485 + x486;
    let x488 = x466 * x470;
    let x489 = x467 + x488;
    let x490 = x464 * x487 + x468 * x489;
    let x491 = x466 * x466;
    let x492 = -x491;
    let x493 = x469 * x469;
    let x494 = x492 + x493;
    let x495 = -x494;
    let x496 = x300 * x490 + x304 * x495;
    let x497 = x464 * x489 + x465 * x487;
    let x498 = -x497;
    let x499 = x162 * x496 + x167 * x498;
    let x500 = x300 * x495 + x301 * x490;
    let x501 = x499 * x75 + x500 * x82;
    let x502 = x167 * x496 + x175 * x498;
    let x503 = -x502;
    let x504 = x499 * x76 + x500 * x75;
    let x505 = ddq[6] + x352;
    let x506 = x473 + x505;
    let x507 = x464 * x506;
    let x508 = x470 * x470;
    let x509 = -x493;
    let x510 = x508 + x509;
    let x511 = x468 * x510 + x507;
    let x512 = -x488;
    let x513 = x467 + x512;
    let x514 = -x513;
    let x515 = x300 * x511 + x304 * x514;
    let x516 = x464 * x510 + x465 * x506;
    let x517 = -x516;
    let x518 = x162 * x515 + x167 * x517;
    let x519 = x300 * x514 + x301 * x511;
    let x520 = x518 * x75 + x519 * x82;
    let x521 = x167 * x515 + x175 * x517;
    let x522 = -x521;
    let x523 = x518 * x76 + x519 * x75;
    let x524 = -x486;
    let x525 = x464 * x512 + x465 * x524;
    let x526 = x300 * x525 + x304 * x474;
    let x527 = x464 * x486 + x465 * x512;
    let x528 = -x527;
    let x529 = x162 * x526 + x167 * x528;
    let x530 = x300 * x474 + x301 * x525;
    let x531 = x529 * x75 + x530 * x82;
    let x532 = x167 * x526 + x175 * x528;
    let x533 = -x532;
    let x534 = x529 * x76 + x530 * x75;
    let x535 = -x508;
    let x536 = x491 + x535;
    let x537 = x474 + x505;
    let x538 = x464 * x536 + x468 * x537;
    let x539 = x471 + x486;
    let x540 = -x539;
    let x541 = x300 * x538 + x304 * x540;
    let x542 = x464 * x537 + x465 * x536;
    let x543 = -x542;
    let x544 = x162 * x541 + x167 * x543;
    let x545 = x300 * x540 + x301 * x538;
    let x546 = x544 * x75 + x545 * x82;
    let x547 = x167 * x541 + x175 * x543;
    let x548 = -x547;
    let x549 = x544 * x76 + x545 * x75;
    let x550 = x464 * x488 + x468 * x485;
    let x551 = -x505;
    let x552 = x300 * x550 + x304 * x551;
    let x553 = x464 * x485 + x465 * x488;
    let x554 = -x553;
    let x555 = x162 * x552 + x167 * x554;
    let x556 = x300 * x551 + x301 * x550;
    let x557 = x555 * x75 + x556 * x82;
    let x558 = x167 * x552 + x175 * x554;
    let x559 = -x558;
    let x560 = x555 * x76 + x556 * x75;
    let x561 = -0.088 * x337 + x383;
    let x562 = -x561;
    let x563 = x468 * x562;
    let x564 = x272 + 0.088 * x387;
    let x565 = 0.088 * x384 + x405;
    let x566 = x464 * x564 + x468 * x565;
    let x567 = -x566;
    let x568 = x492 + x535;
    let x569 = x465 * x568;
    let x570 = -0.088 * x507 + x567 - 0.088 * x569;
    let x571 = x300 * x563 + x304 * x570;
    let x572 = x471 + x524;
    let x573 = x464 * x562 - 0.088 * x572;
    let x574 = -x573;
    let x575 = x464 * x568 + x468 * x506;
    let x576 = -x572;
    let x577 = x300 * x575 + x304 * x576;
    let x578 = x163 * x577;
    let x579 = -x507 - x569;
    let x580 = x162 * x579;
    let x581 = x162 * x571 + x167 * x574 - 0.384 * x578 - 0.384 * x580;
    let x582 = x300 * x570 + x301 * x563;
    let x583 = -0.0825 * x578 - 0.0825 * x580 + x582;
    let x584 = x581 * x75 + x583 * x82;
    let x585 = x300 * x576 + x301 * x575;
    let x586 = -0.0825 * x585;
    let x587 = x162 * x577;
    let x588 = x167 * x571 + x175 * x574 + x241 * x579 + x586 - 0.384 * x587;
    let x589 = x167 * x579 + x587;
    let x590 = x586 * x75 - x588 - 0.0825 * x589 * x76;
    let x591 = x585 * x82 + x589 * x75;
    let x592 = x167 * x577 + x175 * x579;
    let x593 = -x592;
    let x594 = x581 * x76 + x583 * x75 - 0.0825 * x592;
    let x595 = x464 * x561;
    let x596 = x464 * x565 + x465 * x564;
    let x597 = -x596;
    let x598 = x473 + x551;
    let x599 = x465 * x598;
    let x600 = x509 + x535;
    let x601 = x464 * x600;
    let x602 = -x597 - 0.088 * x599 - 0.088 * x601;
    let x603 = x300 * x595 + x304 * x602;
    let x604 = x465 * x561 - 0.088 * x489;
    let x605 = -x604;
    let x606 = x464 * x598 + x468 * x600;
    let x607 = -x489;
    let x608 = x300 * x606 + x304 * x607;
    let x609 = x163 * x608;
    let x610 = -x599 - x601;
    let x611 = x162 * x610;
    let x612 = x162 * x603 + x167 * x605 - 0.384 * x609 - 0.384 * x611;
    let x613 = x300 * x602 + x301 * x595;
    let x614 = -0.0825 * x609 - 0.0825 * x611 + x613;
    let x615 = x612 * x75 + x614 * x82;
    let x616 = x300 * x607 + x301 * x606;
    let x617 = -0.0825 * x616;
    let x618 = x162 * x608;
    let x619 = x167 * x603 + x175 * x605 + x241 * x610 + x617 - 0.384 * x618;
    let x620 = x167 * x610 + x618;
    let x621 = x617 * x75 - x619 - 0.0825 * x620 * x76;
    let x622 = x616 * x82 + x620 * x75;
    let x623 = x167 * x608 + x175 * x610;
    let x624 = -x623;
    let x625 = x612 * x76 + x614 * x75 - 0.0825 * x623;
    let x626 = x464 * x567 + x468 * x596;
    let x627 = x465 * x539;
    let x628 = -x467 + x488;
    let x629 = x464 * x628;
    let x630 = -0.088 * x627 - 0.088 * x629;
    let x631 = x300 * x626 + x304 * x630;
    let x632 = x492 + x509;
    let x633 = x464 * x596;
    let x634 = x465 * x567 - 0.088 * x632 + x633;
    let x635 = -x634;
    let x636 = x464 * x539 + x468 * x628;
    let x637 = -x632;
    let x638 = x300 * x636 + x304 * x637;
    let x639 = x163 * x638;
    let x640 = -x627 - x629;
    let x641 = x162 * x640;
    let x642 = x162 * x631 + x167 * x635 - 0.384 * x639 - 0.384 * x641;
    let x643 = x300 * x630 + x301 * x626;
    let x644 = -0.0825 * x639 - 0.0825 * x641 + x643;
    let x645 = x642 * x75 + x644 * x82;
    let x646 = x300 * x637 + x301 * x636;
    let x647 = -0.0825 * x646;
    let x648 = x162 * x638;
    let x649 = x167 * x631 + x175 * x635 + x241 * x640 + x647 - 0.384 * x648;
    let x650 = x167 * x640 + x648;
    let x651 = x647 * x75 - x649 - 0.0825 * x650 * x76;
    let x652 = x646 * x82 + x650 * x75;
    let x653 = x167 * x638 + x175 * x640;
    let x654 = -x653;
    let x655 = x642 * x76 + x644 * x75 - 0.0825 * x653;
    let x656 = x465 * x596;
    let x657 = x464 * x566;
    let x658 = -0.088 * x656 - 0.088 * x657;
    let x659 = x304 * x658;
    let x660 = -0.088 * x561;
    let x661 = -x660;
    let x662 = x468 * x566 + x633;
    let x663 = x300 * x662 + x304 * x562;
    let x664 = x163 * x663;
    let x665 = -x656 - x657;
    let x666 = x162 * x665;
    let x667 = x162 * x659 + x167 * x661 - 0.384 * x664 - 0.384 * x666;
    let x668 = x300 * x658;
    let x669 = -0.0825 * x664 - 0.0825 * x666 + x668;
    let x670 = x667 * x75 + x669 * x82;
    let x671 = x300 * x562 + x301 * x662;
    let x672 = -0.0825 * x671;
    let x673 = x162 * x663;
    let x674 = x167 * x659 + x175 * x661 + x241 * x665 + x672 - 0.384 * x673;
    let x675 = x167 * x665 + x673;
    let x676 = x672 * x75 - x674 - 0.0825 * x675 * x76;
    let x677 = x671 * x82 + x675 * x75;
    let x678 = x167 * x663 + x175 * x665;
    let x679 = -x678;
    let x680 = x667 * x76 + x669 * x75 - 0.0825 * x678;
    let x681 = x29 * x49;
    let x682 = x28 * x70;
    h[5] = ddq[0];
    h[10] = if dq[0].abs() < SIGN_ZERO {
        0.0
    } else {
        dq[0].signum()
    };
    h[11] = 1.0;
    h[12] = x0 * x7 + x11;
    h[13] = x13 * (x10 + x12) + x14 * x15;
    h[14] = x13 * (ddq[1] + x16) + x14 * (x17 + x19);
    h[15] = x13 * x21 + x2 * x20;
    h[16] = x13 * (x22 + x23) + x14 * (ddq[1] + x24);
    h[17] = -x11 + x13 * x5;
    h[20] = x13 * x27 + x14 * x25;
    h[24] = x13 * (x28 * x32 + x30 * x35) - x14 * x37;
    h[25] = x13 * (x28 * x41 + x30 * x43) - x14 * x47;
    h[26] = x13 * (x30 * x53 + x50) - x14 * x55;
    h[27] = x13 * (x28 * x54 + x30 * x40) + x14 * x37;
    h[28] = x13 * (x28 * x57 + x30 * x58) - x14 * x59;
    h[29] = x13 * (x28 * x42 + x30 * x38) + x14 * x60;
    h[30] = x13 * (-0.316 * x29 * x67 + x30 * x66 - 0.316 * x50) + x14 * x64;
    h[31] = x13 * (x28 * x65 - 0.316 * x28 * x69 - 0.316 * x29 * x68) - x14 * x71;
    h[32] = x13 * (x28 * x64 - 0.316 * x28 * x73 - 0.316 * x29 * x59 + x30 * x70);
    h[33] = x13 * (-0.316 * x28 * x63 + x70 * x74);
    h[36] = x13 * (x28 * x83 + x30 * x86) - x14 * x87;
    h[37] = -x100 * x14 + x13 * (x28 * x94 + x30 * x99);
    h[38] = -x111 * x14 + x13 * (x107 * x28 + x110 * x30);
    h[39] = -x114 * x14 + x13 * (x113 * x28 + x30 * x85);
    h[40] = -x121 * x14 + x13 * (x118 * x28 + x120 * x30);
    h[41] = -x124 * x14 + x13 * (x122 * x28 + x123 * x30);
    h[42] = x13 * (x127 * x28 + x135 * x30 + x136 * x74 - 0.316 * x138 * x28) - x14 * x140;
    h[43] = x13 * (x141 * x28 + x146 * x30 + x147 * x74 - 0.316 * x148 * x28) - x14 * x149;
    h[44] = x13 * (x150 * x28 + x153 * x30 + x154 * x74 - 0.316 * x156 * x28) - x14 * x158;
    h[45] = x13 * (-0.316 * x126 * x28 + x159 * x30 + x160 * x74) - x14 * x161;
    h[48] = x13 * (x174 * x28 + x177 * x30) - x14 * x178;
    h[49] = x13 * (x190 * x28 + x192 * x30) - x14 * x193;
    h[50] = x13 * (x203 * x28 + x206 * x30) - x14 * x207;
    h[51] = x13 * (x209 * x28 + x212 * x30) - x14 * x213;
    h[52] = x13 * (x219 * x28 + x221 * x30) - x14 * x222;
    h[53] = x13 * (x225 * x28 + x227 * x30) - x14 * x228;
    h[54] = x13 * (x238 * x28 + x245 * x30 + x246 * x74 - 0.316 * x248 * x28) - x14 * x249;
    h[55] = x13 * (x258 * x28 + x263 * x30 + x264 * x74 - 0.316 * x266 * x28) - x14 * x267;
    h[56] = x13 * (x276 * x28 - 0.316 * x28 * x285 + x282 * x30 + x283 * x74) - x14 * x286;
    h[57] = x13 * (x28 * x291 - 0.316 * x28 * x298 + x296 * x30 + x297 * x74) - x14 * x299;
    h[60] = x13 * (x28 * x314 + x30 * x316) - x14 * x317;
    h[61] = x13 * (x28 * x332 + x30 * x334) - x14 * x335;
    h[62] = x13 * (x28 * x348 + x30 * x350) - x14 * x351;
    h[63] = x13 * (x28 * x356 + x30 * x358) - x14 * x359;
    h[64] = x13 * (x28 * x368 + x30 * x370) - x14 * x371;
    h[65] = x13 * (x28 * x376 + x30 * x378) - x14 * x379;
    h[66] = x13 * (x28 * x393 - 0.316 * x28 * x402 + x30 * x399 + x400 * x74) - x14 * x403;
    h[67] = x13 * (x28 * x417 - 0.316 * x28 * x426 + x30 * x423 + x424 * x74) - x14 * x427;
    h[68] = x13 * (x28 * x438 - 0.316 * x28 * x447 + x30 * x444 + x445 * x74) - x14 * x448;
    h[69] = x13 * (x28 * x453 - 0.316 * x28 * x462 + x30 * x459 + x460 * x74) - x14 * x463;
    h[72] = x13 * (x28 * x481 + x30 * x483) - x14 * x484;
    h[73] = x13 * (x28 * x501 + x30 * x503) - x14 * x504;
    h[74] = x13 * (x28 * x520 + x30 * x522) - x14 * x523;
    h[75] = x13 * (x28 * x531 + x30 * x533) - x14 * x534;
    h[76] = x13 * (x28 * x546 + x30 * x548) - x14 * x549;
    h[77] = x13 * (x28 * x557 + x30 * x559) - x14 * x560;
    h[78] = x13 * (x28 * x584 - 0.316 * x28 * x593 + x30 * x590 + x591 * x74) - x14 * x594;
    h[79] = x13 * (x28 * x615 - 0.316 * x28 * x624 + x30 * x621 + x622 * x74) - x14 * x625;
    h[80] = x13 * (x28 * x645 - 0.316 * x28 * x654 + x30 * x651 + x652 * x74) - x14 * x655;
    h[81] = x13 * (x28 * x670 - 0.316 * x28 * x679 + x30 * x676 + x677 * x74) - x14 * x680;
    h[96] = x24;
    h[97] = x18 - x23;
    h[98] = x21 + x6;
    h[99] = x16;
    h[100] = x12 + x9;
    h[101] = ddq[1];
    h[102] = x26;
    h[103] = -x25;
    h[106] = if dq[1].abs() < SIGN_ZERO {
        0.0
    } else {
        dq[1].signum()
    };
    h[107] = 1.0;
    h[108] = x28 * x35 + x29 * x32;
    h[109] = x28 * x43 + x29 * x41;
    h[110] = x28 * x53 + x681;
    h[111] = x28 * x40 + x29 * x54;
    h[112] = x28 * x58 + x29 * x57;
    h[113] = x28 * x38 + x29 * x42;
    h[114] = x28 * x66 + 0.316 * x28 * x67 - 0.316 * x681;
    h[115] = 0.316 * x28 * x68 + x29 * x65 + x69 * x74;
    h[116] = 0.316 * x28 * x59 + x29 * x64 + x682 + x73 * x74;
    h[117] = x63 * x74 + 0.316 * x682;
    h[120] = x28 * x86 + x29 * x83;
    h[121] = x28 * x99 + x29 * x94;
    h[122] = x107 * x29 + x110 * x28;
    h[123] = x113 * x29 + x28 * x85;
    h[124] = x118 * x29 + x120 * x28;
    h[125] = x122 * x29 + x123 * x28;
    h[126] = x127 * x29 + x135 * x28 + 0.316 * x136 * x28 + x138 * x74;
    h[127] = x141 * x29 + x146 * x28 + 0.316 * x147 * x28 + x148 * x74;
    h[128] = x150 * x29 + x153 * x28 + 0.316 * x154 * x28 + x156 * x74;
    h[129] = x126 * x74 + x159 * x28 + 0.316 * x160 * x28;
    h[132] = x174 * x29 + x177 * x28;
    h[133] = x190 * x29 + x192 * x28;
    h[134] = x203 * x29 + x206 * x28;
    h[135] = x209 * x29 + x212 * x28;
    h[136] = x219 * x29 + x221 * x28;
    h[137] = x225 * x29 + x227 * x28;
    h[138] = x238 * x29 + x245 * x28 + 0.316 * x246 * x28 + x248 * x74;
    h[139] = x258 * x29 + x263 * x28 + 0.316 * x264 * x28 + x266 * x74;
    h[140] = x276 * x29 + x28 * x282 + 0.316 * x28 * x283 + x285 * x74;
    h[141] = x28 * x296 + 0.316 * x28 * x297 + x29 * x291 + x298 * x74;
    h[144] = x28 * x316 + x29 * x314;
    h[145] = x28 * x334 + x29 * x332;
    h[146] = x28 * x350 + x29 * x348;
    h[147] = x28 * x358 + x29 * x356;
    h[148] = x28 * x370 + x29 * x368;
    h[149] = x28 * x378 + x29 * x376;
    h[150] = x28 * x399 + 0.316 * x28 * x400 + x29 * x393 + x402 * x74;
    h[151] = x28 * x423 + 0.316 * x28 * x424 + x29 * x417 + x426 * x74;
    h[152] = x28 * x444 + 0.316 * x28 * x445 + x29 * x438 + x447 * x74;
    h[153] = x28 * x459 + 0.316 * x28 * x460 + x29 * x453 + x462 * x74;
    h[156] = x28 * x483 + x29 * x481;
    h[157] = x28 * x503 + x29 * x501;
    h[158] = x28 * x522 + x29 * x520;
    h[159] = x28 * x533 + x29 * x531;
    h[160] = x28 * x548 + x29 * x546;
    h[161] = x28 * x559 + x29 * x557;
    h[162] = x28 * x590 + 0.316 * x28 * x591 + x29 * x584 + x593 * x74;
    h[163] = x28 * x621 + 0.316 * x28 * x622 + x29 * x615 + x624 * x74;
    h[164] = x28 * x651 + 0.316 * x28 * x652 + x29 * x645 + x654 * x74;
    h[165] = x28 * x676 + 0.316 * x28 * x677 + x29 * x670 + x679 * x74;
    h[192] = x37;
    h[193] = x47;
    h[194] = x55;
    h[195] = x36;
    h[196] = x59;
    h[197] = x48;
    h[198] = x63;
    h[199] = x71;
    h[202] = if dq[2].abs() < SIGN_ZERO {
        0.0
    } else {
        dq[2].signum()
    };
    h[203] = 1.0;
    h[204] = x87;
    h[205] = x100;
    h[206] = x111;
    h[207] = x114;
    h[208] = x121;
    h[209] = x124;
    h[210] = x140;
    h[211] = x149;
    h[212] = x158;
    h[213] = x161;
    h[216] = x178;
    h[217] = x193;
    h[218] = x207;
    h[219] = x213;
    h[220] = x222;
    h[221] = x228;
    h[222] = x249;
    h[223] = x267;
    h[224] = x286;
    h[225] = x299;
    h[228] = x317;
    h[229] = x335;
    h[230] = x351;
    h[231] = x359;
    h[232] = x371;
    h[233] = x379;
    h[234] = x403;
    h[235] = x427;
    h[236] = x448;
    h[237] = x463;
    h[240] = x484;
    h[241] = x504;
    h[242] = x523;
    h[243] = x534;
    h[244] = x549;
    h[245] = x560;
    h[246] = x594;
    h[247] = x625;
    h[248] = x655;
    h[249] = x680;
    h[288] = x85;
    h[289] = x98;
    h[290] = x109;
    h[291] = x84;
    h[292] = x119;
    h[293] = x102;
    h[294] = x130;
    h[295] = x143;
    h[298] = if dq[3].abs() < SIGN_ZERO {
        0.0
    } else {
        dq[3].signum()
    };
    h[299] = 1.0;
    h[300] = x176;
    h[301] = x191;
    h[302] = x205;
    h[303] = x211;
    h[304] = x220;
    h[305] = x226;
    h[306] = x243;
    h[307] = x261;
    h[308] = x280;
    h[309] = x294;
    h[312] = x315;
    h[313] = x333;
    h[314] = x349;
    h[315] = x357;
    h[316] = x369;
    h[317] = x377;
    h[318] = x397;
    h[319] = x421;
    h[320] = x442;
    h[321] = x457;
    h[324] = x482;
    h[325] = x502;
    h[326] = x521;
    h[327] = x532;
    h[328] = x547;
    h[329] = x558;
    h[330] = x588;
    h[331] = x619;
    h[332] = x649;
    h[333] = x674;
    h[384] = x173;
    h[385] = x189;
    h[386] = x202;
    h[387] = x172;
    h[388] = x218;
    h[389] = x194;
    h[390] = x236;
    h[391] = x256;
    h[394] = if dq[4].abs() < SIGN_ZERO {
        0.0
    } else {
        dq[4].signum()
    };
    h[395] = 1.0;
    h[396] = x313;
    h[397] = x331;
    h[398] = x347;
    h[399] = x355;
    h[400] = x367;
    h[401] = x375;
    h[402] = x391;
    h[403] = x415;
    h[404] = x436;
    h[408] = x480;
    h[409] = x500;
    h[410] = x519;
    h[411] = x530;
    h[412] = x545;
    h[413] = x556;
    h[414] = x582;
    h[415] = x613;
    h[416] = x643;
    h[417] = x668;
    h[480] = x310;
    h[481] = x328;
    h[482] = x344;
    h[483] = x309;
    h[484] = x364;
    h[485] = x336;
    h[486] = x382;
    h[487] = x406;
    h[490] = if dq[5].abs() < SIGN_ZERO {
        0.0
    } else {
        dq[5].signum()
    };
    h[491] = 1.0;
    h[492] = x477;
    h[493] = x497;
    h[494] = x516;
    h[495] = x527;
    h[496] = x542;
    h[497] = x553;
    h[498] = x573;
    h[499] = x604;
    h[500] = x634;
    h[501] = x660;
    h[576] = x474;
    h[577] = x494;
    h[578] = x513;
    h[579] = x473;
    h[580] = x539;
    h[581] = x505;
    h[582] = x566;
    h[583] = x597;
    h[586] = if dq[6].abs() < SIGN_ZERO {
        0.0
    } else {
        dq[6].signum()
    };
    h[587] = 1.0;
    h
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solve_franka_carry_with_copp3_socp_time_and_hybrid() -> TestResult<()> {
        let (mut robot, path) = make_franka_carry_robot_and_path(PATH_SAMPLES)?;
        let s = robot.constraints.s_vec(0, robot.constraints.len())?;
        let idx_s_interval = (0, s.len() - 1);
        let a_boundary = (0.0, 0.0);
        let b_boundary = (0.0, 0.0);

        let a_seed = {
            let problem = Topp2ProblemBuilder::new(&robot, idx_s_interval, a_boundary).build()?;
            let options = ReachSet2OptionsBuilder::new().build()?;
            topp2_ra(&problem, &options)?
        };
        robot.constraints.amax_substitute(&a_seed, 0)?;

        let opts_socp = ClarabelOptionsBuilder::new()
            .allow_almost_solved(true)
            .build()?;

        let time_objectives = vec![CoppObjective::Time(1.0)];
        let profile_time = {
            let profile_first = solve_copp3_socp_iteration(
                &mut robot,
                &time_objectives,
                &a_seed,
                a_boundary,
                b_boundary,
                &opts_socp,
            )?;
            let profile_second = solve_copp3_socp_iteration(
                &mut robot,
                &time_objectives,
                &profile_first.a,
                a_boundary,
                b_boundary,
                &opts_socp,
            )?;
            solve_copp3_socp_iteration(
                &mut robot,
                &time_objectives,
                &profile_second.a,
                a_boundary,
                b_boundary,
                &opts_socp,
            )?
        };
        let (t_final_time, t_s_time) = s_to_t_topp3(&s, profile_time.as_parts(), 0.0)?;
        let energy_time =
            thermal_energy_on_station_grid(&path, &s, &profile_time.a, &profile_time.b, &t_s_time)?;

        let torque_normalize = TMAX.map(|tau_max| 1.0 / tau_max);
        let hybrid_objectives = vec![
            CoppObjective::Time(1.0),
            CoppObjective::ThermalEnergy(THERMAL_WEIGHT, &torque_normalize),
        ];
        let profile_hybrid = {
            let profile_first = solve_copp3_socp_iteration(
                &mut robot,
                &hybrid_objectives,
                &a_seed,
                a_boundary,
                b_boundary,
                &opts_socp,
            )?;
            let profile_second = solve_copp3_socp_iteration(
                &mut robot,
                &hybrid_objectives,
                &profile_first.a,
                a_boundary,
                b_boundary,
                &opts_socp,
            )?;
            solve_copp3_socp_iteration(
                &mut robot,
                &hybrid_objectives,
                &profile_second.a,
                a_boundary,
                b_boundary,
                &opts_socp,
            )?
        };
        let (t_final_hybrid, t_s_hybrid) = s_to_t_topp3(&s, profile_hybrid.as_parts(), 0.0)?;
        let energy_hybrid = thermal_energy_on_station_grid(
            &path,
            &s,
            &profile_hybrid.a,
            &profile_hybrid.b,
            &t_s_hybrid,
        )?;

        print_result(
            "SOCP time",
            t_final_time,
            energy_time,
            t_final_time,
            profile_time.num_stationary,
            &profile_time.a,
            &profile_time.b,
        );
        print_result(
            "SOCP hybrid",
            t_final_hybrid,
            energy_hybrid,
            t_final_hybrid + THERMAL_WEIGHT * energy_hybrid,
            profile_hybrid.num_stationary,
            &profile_hybrid.a,
            &profile_hybrid.b,
        );

        assert!(t_final_time.is_finite() && t_final_time > 0.0);
        assert!(t_final_hybrid.is_finite() && t_final_hybrid > 0.0);
        assert!(energy_time.is_finite() && energy_time >= 0.0);
        assert!(energy_hybrid.is_finite() && energy_hybrid >= 0.0);

        Ok(())
    }

    const PATH_SAMPLES: usize = 1001;
    const VMAX: [f64; FRANKA_DOF] = [1.1, 1.1, 2.0, 1.1, 1.305, 1.305, 1.305];
    const AMAX: [f64; FRANKA_DOF] = [6.0, 6.0, 8.0, 10.0, 12.0, 16.0, 16.0];
    const JMAX: [f64; FRANKA_DOF] = [350.0, 490.0, 560.0, 560.0, 560.0, 560.0, 560.0];
    const TMAX: [f64; FRANKA_DOF] = [52.2, 52.2, 52.2, 52.2, 7.2, 7.2, 7.2];
    const THERMAL_WEIGHT: f64 = 5.0;

    type TestResult<T> = Result<T, Box<dyn std::error::Error>>;

    fn solve_copp3_socp_iteration(
        robot: &mut Robot<FrankaNoViscousModel>,
        objectives: &[CoppObjective<'_>],
        a_seed: &[f64],
        a_boundary: (f64, f64),
        b_boundary: (f64, f64),
        opts_socp: &copp::solver::copp3_socp::ClarabelOptions,
    ) -> TestResult<Topp3Profile> {
        let problem =
            Copp3ProblemBuilder::new(robot, objectives, 0, a_seed, a_boundary, b_boundary)
                .build_with_linearization()?;
        Ok(copp3_socp(&problem, opts_socp)?)
    }

    fn make_franka_carry_robot_and_path(
        n: usize,
    ) -> TestResult<(Robot<FrankaNoViscousModel>, Path)> {
        let waypoints: &[[f64; FRANKA_DOF]] = &[
            [
                0.012153940129962843,
                -0.7754088995358044,
                0.006128888484063546,
                -2.3616255788586735,
                0.0010100371700716712,
                1.5718217522556466,
                0.8109682303319373,
            ],
            [
                -1.0280690113996205,
                0.5729317021470539,
                -0.7133851083456398,
                -1.6248129860012,
                0.3568181437253943,
                2.0949306228160856,
                0.4823342094471057,
            ],
            [
                -0.7311841840785785,
                0.04684228509605245,
                0.6302858664191684,
                -1.5334595664055826,
                -0.013256409841343929,
                1.6085592446042687,
                0.7541331845106661,
            ],
            [
                -1.0280690113996205,
                0.5729317021470539,
                -0.7133851083456398,
                -1.6248129860012,
                0.3568181437253943,
                2.0949306228160856,
                0.4823342094471057,
            ],
            [
                -0.7578486721013721,
                0.518776978258501,
                0.762822077899602,
                -1.760920539508491,
                -0.388982286271545,
                2.09698111672023,
                1.0986684727537102,
            ],
            [
                -1.0280690113996205,
                0.5729317021470539,
                -0.7133851083456398,
                -1.6248129860012,
                0.3568181437253943,
                2.0949306228160856,
                0.4823342094471057,
            ],
            [
                -0.6253049090213272,
                -0.8413715532696,
                0.7352110023524686,
                -2.9885874693362093,
                0.6295055113238361,
                2.132425441185634,
                0.36971949519571246,
            ],
            [
                -1.0280690113996205,
                0.5729317021470539,
                -0.7133851083456398,
                -1.6248129860012,
                0.3568181437253943,
                2.0949306228160856,
                0.4823342094471057,
            ],
            [
                -1.2428417805077736,
                0.9329576058806034,
                0.7336554984649933,
                -1.5817274529568839,
                -0.8051503452379202,
                2.225606162563603,
                0.5695070223868421,
            ],
            [
                -1.0280690113996205,
                0.5729317021470539,
                -0.7133851083456398,
                -1.6248129860012,
                0.3568181437253943,
                2.0949306228160856,
                0.4823342094471057,
            ],
            [
                -1.3484241422147956,
                0.8228130042993188,
                1.3242334300170395,
                -1.151040788134121,
                -0.835997090935781,
                1.4336946593124638,
                0.5430478815942863,
            ],
        ];

        let wp_mat = DMatrix::<f64>::from_fn(FRANKA_DOF, waypoints.len(), |i, j| waypoints[j][i]);
        let path = Path::from_waypoints(&wp_mat, SplineConfig::default())?;
        let s: Vec<f64> = (0..n).map(|j| j as f64 / (n - 1) as f64).collect();

        let mut robot = Robot::with_capacity(FrankaNoViscousModel, n);
        robot
            .with_s(s.as_slice())?
            .with_q_from_path_3rd(&path, 0, n)?
            .with_axial_velocity((VMAX.as_slice(), n), (negated(VMAX).as_slice(), n), 0)?
            .with_axial_acceleration((AMAX.as_slice(), n), (negated(AMAX).as_slice(), n), 0)?
            .with_axial_jerk((JMAX.as_slice(), n), (negated(JMAX).as_slice(), n), 0)?
            .with_axial_torque((TMAX.as_slice(), n), (negated(TMAX).as_slice(), n), 0)?;

        Ok((robot, path))
    }

    fn thermal_energy_on_station_grid(
        path: &Path,
        s: &[f64],
        a_profile: &[f64],
        b_profile: &[f64],
        t_s: &[f64],
    ) -> TestResult<f64> {
        let derivs = path.evaluate_up_to_2nd(s)?;
        let q_s = derivs.dq.as_ref().ok_or_else(|| missing_derivative("dq"))?;
        let q_ss = derivs
            .ddq
            .as_ref()
            .ok_or_else(|| missing_derivative("ddq"))?;
        let torque_normalize = TMAX.map(|tau_max| 1.0 / tau_max);
        let mut tau_left = [0.0; FRANKA_DOF];
        let mut tau_right = [0.0; FRANKA_DOF];
        let mut energy = 0.0;

        for k in 0..s.len() - 1 {
            let dt = t_s[k + 1] - t_s[k];
            if dt <= 0.0 {
                continue;
            }

            fill_time_domain_torque(
                &derivs.q,
                q_s,
                q_ss,
                k,
                a_profile[k],
                b_profile[k],
                &mut tau_left,
            );
            fill_time_domain_torque(
                &derivs.q,
                q_s,
                q_ss,
                k + 1,
                a_profile[k + 1],
                b_profile[k + 1],
                &mut tau_right,
            );

            for joint in 0..FRANKA_DOF {
                let left = tau_left[joint] * torque_normalize[joint];
                let right = tau_right[joint] * torque_normalize[joint];
                energy += (left * left + right * right + left * right) * dt / 3.0;
            }
        }

        Ok(energy)
    }

    fn fill_time_domain_torque(
        q_grid: &DMatrix<f64>,
        q_s_grid: &DMatrix<f64>,
        q_ss_grid: &DMatrix<f64>,
        idx: usize,
        a: f64,
        b: f64,
        tau: &mut [f64; FRANKA_DOF],
    ) {
        let s_dot = a.max(0.0).sqrt();
        let mut q = [0.0; FRANKA_DOF];
        let mut dq = [0.0; FRANKA_DOF];
        let mut ddq = [0.0; FRANKA_DOF];

        for joint in 0..FRANKA_DOF {
            q[joint] = q_grid[(joint, idx)];
            dq[joint] = q_s_grid[(joint, idx)] * s_dot;
            ddq[joint] = q_ss_grid[(joint, idx)] * a + q_s_grid[(joint, idx)] * b;
        }

        inverse_dynamics_franka_no_viscous(&q, &dq, &ddq, tau);
    }

    fn print_result(
        label: &str,
        t_final: f64,
        thermal_energy: f64,
        objective: f64,
        num_stationary: (usize, usize),
        a_profile: &[f64],
        b_profile: &[f64],
    ) {
        let (a_min, a_max) = min_max(a_profile);
        let (b_min, b_max) = min_max(b_profile);
        println!("{label:<11} t_final={t_final:.6}s thermal_energy={thermal_energy:.6}]");
    }

    fn min_max(values: &[f64]) -> (f64, f64) {
        values.iter().fold(
            (f64::INFINITY, f64::NEG_INFINITY),
            |(min_v, max_v), &value| (min_v.min(value), max_v.max(value)),
        )
    }

    fn negated<const N: usize>(values: [f64; N]) -> [f64; N] {
        values.map(|value| -value)
    }

    fn missing_derivative(name: &'static str) -> std::io::Error {
        std::io::Error::other(format!("missing {name} derivative"))
    }
}
