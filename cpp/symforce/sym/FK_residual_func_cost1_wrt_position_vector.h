// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <sym/rot3.h>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: FK_residual_func_cost1_wrt_position_vector
 *
 * Args:
 *     fh1: Scalar
 *     fv1: Scalar
 *     DeltaRot: Rot3
 *     position_vector: Matrix31
 *     Rot_init: Rot3
 *     lc0: Scalar
 *     lc1: Scalar
 *     lc2: Scalar
 *     lc3: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix43
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 3> FkResidualFuncCost1WrtPositionVector(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar lc0, const Scalar lc1, const Scalar lc2, const Scalar lc3, const Scalar epsilon) {
  // Total ops: 1579

  // Unused inputs
  (void)lc0;
  (void)lc1;
  (void)lc2;
  (void)lc3;
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (495)
  const Scalar _tmp0 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp1 = -2 * std::pow(_tmp0, Scalar(2));
  const Scalar _tmp2 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp3 = 1 - 2 * std::pow(_tmp2, Scalar(2));
  const Scalar _tmp4 = Scalar(0.20999999999999999) * _tmp1 + Scalar(0.20999999999999999) * _tmp3;
  const Scalar _tmp5 = -_tmp4;
  const Scalar _tmp6 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp7 = 2 * _tmp2;
  const Scalar _tmp8 = _tmp6 * _tmp7;
  const Scalar _tmp9 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                       2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp10 = _tmp0 * _tmp9;
  const Scalar _tmp11 = _tmp10 + _tmp8;
  const Scalar _tmp12 = -Scalar(0.010999999999999999) * _tmp11;
  const Scalar _tmp13 = 2 * _tmp0 * _tmp6;
  const Scalar _tmp14 = _tmp2 * _tmp9;
  const Scalar _tmp15 = Scalar(0.20999999999999999) * _tmp13 - Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp16 = _tmp12 - _tmp15;
  const Scalar _tmp17 = _tmp16 + _tmp5;
  const Scalar _tmp18 = _tmp17 + position_vector(0, 0);
  const Scalar _tmp19 = -_tmp18 + Scalar(-125.0);
  const Scalar _tmp20 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp21 = -_tmp20;
  const Scalar _tmp22 = _tmp0 * _tmp7;
  const Scalar _tmp23 = _tmp6 * _tmp9;
  const Scalar _tmp24 = _tmp22 - _tmp23;
  const Scalar _tmp25 = -Scalar(0.010999999999999999) * _tmp24;
  const Scalar _tmp26 = -2 * std::pow(_tmp6, Scalar(2));
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp26 + Scalar(0.20999999999999999) * _tmp3;
  const Scalar _tmp28 = _tmp25 - _tmp27;
  const Scalar _tmp29 = _tmp21 + _tmp28;
  const Scalar _tmp30 = _tmp29 + position_vector(1, 0);
  const Scalar _tmp31 = -_tmp30 + Scalar(-110.0);
  const Scalar _tmp32 =
      std::sqrt(Scalar(std::pow(_tmp19, Scalar(2)) + std::pow(_tmp31, Scalar(2))));
  const Scalar _tmp33 = Scalar(1.0) / (fh1);
  const Scalar _tmp34 =
      Scalar(1.0) *
      std::sinh(Scalar(0.71007031138673404) * _tmp33 *
                (-_tmp32 - Scalar(1.4083112389913199) * fh1 * std::asinh(_tmp33 * fv1))) /
      _tmp32;
  const Scalar _tmp35 = _tmp16 + _tmp4;
  const Scalar _tmp36 = _tmp35 + position_vector(0, 0);
  const Scalar _tmp37 = _tmp36 + Scalar(-125.0);
  const Scalar _tmp38 = Scalar(1.0) / (_tmp37);
  const Scalar _tmp39 = _tmp20 + _tmp28;
  const Scalar _tmp40 = _tmp39 + position_vector(1, 0);
  const Scalar _tmp41 = _tmp40 + Scalar(110.0);
  const Scalar _tmp42 = std::pow(_tmp41, Scalar(2));
  const Scalar _tmp43 = std::pow(_tmp37, Scalar(2));
  const Scalar _tmp44 = _tmp42 + _tmp43;
  const Scalar _tmp45 = std::sqrt(_tmp44);
  const Scalar _tmp46 = _tmp38 * _tmp45;
  const Scalar _tmp47 = _tmp18 + Scalar(125.0);
  const Scalar _tmp48 = _tmp12 + _tmp15;
  const Scalar _tmp49 = _tmp48 + _tmp5;
  const Scalar _tmp50 = _tmp49 + position_vector(0, 0);
  const Scalar _tmp51 = _tmp50 + Scalar(125.0);
  const Scalar _tmp52 = std::pow(_tmp51, Scalar(2));
  const Scalar _tmp53 = _tmp25 + _tmp27;
  const Scalar _tmp54 = _tmp21 + _tmp53;
  const Scalar _tmp55 = _tmp54 + position_vector(1, 0);
  const Scalar _tmp56 = _tmp55 + Scalar(-110.0);
  const Scalar _tmp57 = std::pow(_tmp56, Scalar(2));
  const Scalar _tmp58 = _tmp52 + _tmp57;
  const Scalar _tmp59 = std::pow(_tmp58, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp60 = -Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp61 = -_tmp60;
  const Scalar _tmp62 = -Scalar(0.010999999999999999) * _tmp1 -
                        Scalar(0.010999999999999999) * _tmp26 + Scalar(-0.010999999999999999);
  const Scalar _tmp63 = Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp64 = _tmp62 + _tmp63;
  const Scalar _tmp65 = _tmp61 + _tmp64;
  const Scalar _tmp66 = _tmp59 * _tmp65;
  const Scalar _tmp67 = _tmp62 - _tmp63;
  const Scalar _tmp68 = _tmp60 + _tmp67;
  const Scalar _tmp69 = _tmp59 * _tmp68;
  const Scalar _tmp70 = _tmp51 * _tmp69;
  const Scalar _tmp71 = _tmp38 * _tmp41;
  const Scalar _tmp72 = _tmp51 * _tmp59;
  const Scalar _tmp73 = _tmp71 * _tmp72;
  const Scalar _tmp74 = -_tmp56 * _tmp59 + _tmp73;
  const Scalar _tmp75 = _tmp4 + _tmp48;
  const Scalar _tmp76 = _tmp75 + position_vector(0, 0);
  const Scalar _tmp77 = _tmp76 + Scalar(-125.0);
  const Scalar _tmp78 = _tmp20 + _tmp53;
  const Scalar _tmp79 = _tmp78 + position_vector(1, 0);
  const Scalar _tmp80 = _tmp79 + Scalar(-110.0);
  const Scalar _tmp81 = std::pow(_tmp80, Scalar(2));
  const Scalar _tmp82 = std::pow(_tmp77, Scalar(2));
  const Scalar _tmp83 = _tmp81 + _tmp82;
  const Scalar _tmp84 = std::pow(_tmp83, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp85 = _tmp71 * _tmp84;
  const Scalar _tmp86 = _tmp77 * _tmp85;
  const Scalar _tmp87 = -_tmp80 * _tmp84 + _tmp86;
  const Scalar _tmp88 = Scalar(1.0) / (_tmp87);
  const Scalar _tmp89 = _tmp60 + _tmp64;
  const Scalar _tmp90 = _tmp84 * _tmp89;
  const Scalar _tmp91 = _tmp68 * _tmp84;
  const Scalar _tmp92 = _tmp77 * _tmp91;
  const Scalar _tmp93 = -_tmp77 * _tmp90 + _tmp92;
  const Scalar _tmp94 = _tmp88 * _tmp93;
  const Scalar _tmp95 = -_tmp71 * _tmp92 + _tmp80 * _tmp90;
  const Scalar _tmp96 = _tmp74 * _tmp88;
  const Scalar _tmp97 = _tmp56 * _tmp66 - _tmp70 * _tmp71 - _tmp95 * _tmp96;
  const Scalar _tmp98 = Scalar(1.0) * _tmp39;
  const Scalar _tmp99 = -_tmp98;
  const Scalar _tmp100 = Scalar(1.0) / (_tmp78 + _tmp99);
  const Scalar _tmp101 = Scalar(1.0) * _tmp35;
  const Scalar _tmp102 = _tmp101 - _tmp75;
  const Scalar _tmp103 = _tmp100 * _tmp102;
  const Scalar _tmp104 = -_tmp103 * _tmp97 - _tmp51 * _tmp66 + _tmp70 - _tmp74 * _tmp94;
  const Scalar _tmp105 = Scalar(1.0) / (_tmp104);
  const Scalar _tmp106 = _tmp49 * _tmp59;
  const Scalar _tmp107 = _tmp54 * _tmp59;
  const Scalar _tmp108 = _tmp78 * _tmp84;
  const Scalar _tmp109 = _tmp75 * _tmp84;
  const Scalar _tmp110 = Scalar(1.0) / (_tmp45);
  const Scalar _tmp111 = _tmp110 * _tmp35;
  const Scalar _tmp112 = _tmp110 * _tmp39;
  const Scalar _tmp113 = _tmp111 * _tmp41 - _tmp112 * _tmp37;
  const Scalar _tmp114 = _tmp113 * _tmp45;
  const Scalar _tmp115 = _tmp114 * _tmp38;
  const Scalar _tmp116 = _tmp77 * _tmp84;
  const Scalar _tmp117 = _tmp108 * _tmp77 - _tmp109 * _tmp80 + _tmp115 * _tmp116;
  const Scalar _tmp118 = -_tmp106 * _tmp56 + _tmp107 * _tmp51 + _tmp115 * _tmp72 - _tmp117 * _tmp96;
  const Scalar _tmp119 = _tmp71 * _tmp88;
  const Scalar _tmp120 = _tmp68 * _tmp71;
  const Scalar _tmp121 = _tmp119 * _tmp95 + _tmp120;
  const Scalar _tmp122 = -_tmp103 * _tmp121 - _tmp68 + _tmp71 * _tmp94;
  const Scalar _tmp123 = _tmp105 * _tmp122;
  const Scalar _tmp124 = _tmp117 * _tmp88;
  const Scalar _tmp125 = -_tmp115 - _tmp118 * _tmp123 + _tmp124 * _tmp71;
  const Scalar _tmp126 = Scalar(1.0) / (_tmp118);
  const Scalar _tmp127 = _tmp104 * _tmp126;
  const Scalar _tmp128 = _tmp125 * _tmp127;
  const Scalar _tmp129 = _tmp122 + _tmp128;
  const Scalar _tmp130 = _tmp105 * _tmp129;
  const Scalar _tmp131 = _tmp130 * _tmp59;
  const Scalar _tmp132 = -_tmp130 * _tmp74 - _tmp71;
  const Scalar _tmp133 = _tmp84 * _tmp88;
  const Scalar _tmp134 = _tmp132 * _tmp133;
  const Scalar _tmp135 = _tmp131 * _tmp51 + _tmp134 * _tmp77 + Scalar(1.0);
  const Scalar _tmp136 = _tmp30 + Scalar(110.0);
  const Scalar _tmp137 = std::pow(_tmp136, Scalar(2));
  const Scalar _tmp138 = std::pow(_tmp47, Scalar(2));
  const Scalar _tmp139 = _tmp137 + _tmp138;
  const Scalar _tmp140 = std::pow(_tmp139, Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp141 = _tmp140 * fh1;
  const Scalar _tmp142 = _tmp135 * _tmp141;
  const Scalar _tmp143 = _tmp142 * _tmp47;
  const Scalar _tmp144 = Scalar(1.0) * _tmp126;
  const Scalar _tmp145 = _tmp133 * _tmp77;
  const Scalar _tmp146 = _tmp145 * _tmp74;
  const Scalar _tmp147 = _tmp144 * _tmp59;
  const Scalar _tmp148 = -_tmp144 * _tmp146 + _tmp147 * _tmp51;
  const Scalar _tmp149 = _tmp140 * _tmp17;
  const Scalar _tmp150 = _tmp140 * _tmp29;
  const Scalar _tmp151 = fh1 * (_tmp136 * _tmp149 - _tmp150 * _tmp47);
  const Scalar _tmp152 = _tmp148 * _tmp151;
  const Scalar _tmp153 = _tmp88 * _tmp95;
  const Scalar _tmp154 = Scalar(1.0) * _tmp100;
  const Scalar _tmp155 = _tmp102 * _tmp154;
  const Scalar _tmp156 = _tmp153 * _tmp155 - Scalar(1.0) * _tmp94;
  const Scalar _tmp157 = _tmp105 * _tmp118;
  const Scalar _tmp158 = Scalar(1.0) * _tmp88;
  const Scalar _tmp159 = -_tmp117 * _tmp158 - _tmp156 * _tmp157;
  const Scalar _tmp160 = _tmp127 * _tmp159;
  const Scalar _tmp161 = _tmp156 + _tmp160;
  const Scalar _tmp162 = _tmp105 * _tmp161;
  const Scalar _tmp163 = _tmp162 * _tmp59;
  const Scalar _tmp164 = -_tmp162 * _tmp74 + Scalar(1.0);
  const Scalar _tmp165 = _tmp133 * _tmp164;
  const Scalar _tmp166 = _tmp163 * _tmp51 + _tmp165 * _tmp77;
  const Scalar _tmp167 = _tmp141 * _tmp166;
  const Scalar _tmp168 = _tmp167 * _tmp46;
  const Scalar _tmp169 = Scalar(333.54000000000002) - fv1;
  const Scalar _tmp170 = _tmp101 + _tmp103 * _tmp98;
  const Scalar _tmp171 = 0;
  const Scalar _tmp172 = _tmp105 * _tmp171;
  const Scalar _tmp173 = _tmp169 * (-_tmp146 * _tmp172 + _tmp172 * _tmp72);
  const Scalar _tmp174 =
      -_tmp136 * _tmp168 - _tmp143 * _tmp46 - _tmp152 * _tmp46 - _tmp173 * _tmp46;
  const Scalar _tmp175 = Scalar(1.0) / (_tmp174);
  const Scalar _tmp176 = _tmp54 + _tmp99;
  const Scalar _tmp177 = _tmp103 * _tmp176;
  const Scalar _tmp178 = Scalar(1.0) / (_tmp101 - _tmp177 - _tmp49);
  const Scalar _tmp179 = _tmp170 * _tmp178;
  const Scalar _tmp180 = -_tmp172 * _tmp97 - _tmp176 * _tmp179 + _tmp99;
  const Scalar _tmp181 = Scalar(1.0) * _tmp178;
  const Scalar _tmp182 = _tmp127 * _tmp181;
  const Scalar _tmp183 = -_tmp144 * _tmp97 + _tmp176 * _tmp182;
  const Scalar _tmp184 = -_tmp154 * _tmp183 + _tmp182;
  const Scalar _tmp185 = Scalar(1.0) * _tmp151;
  const Scalar _tmp186 = _tmp176 * _tmp178;
  const Scalar _tmp187 = -_tmp158 * _tmp95 + _tmp160 * _tmp186 - _tmp162 * _tmp97;
  const Scalar _tmp188 = -_tmp154 * _tmp187 + _tmp160 * _tmp181;
  const Scalar _tmp189 = Scalar(1.0) * fh1;
  const Scalar _tmp190 = _tmp140 * _tmp189;
  const Scalar _tmp191 = _tmp188 * _tmp190;
  const Scalar _tmp192 = _tmp121 + _tmp128 * _tmp186 - _tmp130 * _tmp97;
  const Scalar _tmp193 = _tmp128 * _tmp181 - _tmp154 * _tmp192;
  const Scalar _tmp194 = _tmp190 * _tmp193;
  const Scalar _tmp195 = fh1 * (_tmp61 + _tmp67);
  const Scalar _tmp196 = _tmp140 * _tmp195;
  const Scalar _tmp197 = -_tmp136 * _tmp196 - Scalar(40.024799999999999) * _tmp24 - _tmp29 * fv1;
  const Scalar _tmp198 = _tmp177 * _tmp181 + Scalar(1.0);
  const Scalar _tmp199 = _tmp103 * _tmp181;
  const Scalar _tmp200 = -Scalar(1.0) * _tmp154 * _tmp198 + Scalar(1.0) * _tmp199;
  const Scalar _tmp201 = Scalar(40.024799999999999) * _tmp11 + _tmp17 * fv1 + _tmp196 * _tmp47;
  const Scalar _tmp202 = _tmp176 * _tmp181;
  const Scalar _tmp203 = _tmp100 * _tmp202;
  const Scalar _tmp204 = -Scalar(1.0) * _tmp181 + Scalar(1.0) * _tmp203;
  const Scalar _tmp205 =
      _tmp136 * _tmp191 +
      Scalar(1.0) * _tmp169 * (-_tmp154 * _tmp180 - _tmp170 * _tmp181 + Scalar(1.0)) +
      _tmp184 * _tmp185 + _tmp194 * _tmp47 + _tmp197 * _tmp200 + _tmp201 * _tmp204;
  const Scalar _tmp206 = std::asinh(_tmp175 * _tmp205);
  const Scalar _tmp207 = Scalar(1.4083112389913199) * _tmp174;
  const Scalar _tmp208 = -_tmp40 + Scalar(-110.0);
  const Scalar _tmp209 = Scalar(125.0) - _tmp36;
  const Scalar _tmp210 =
      std::sqrt(Scalar(std::pow(_tmp208, Scalar(2)) + std::pow(_tmp209, Scalar(2))));
  const Scalar _tmp211 = -_tmp206 * _tmp207 - _tmp210;
  const Scalar _tmp212 = Scalar(0.71007031138673404) * _tmp175;
  const Scalar _tmp213 = _tmp211 * _tmp212;
  const Scalar _tmp214 = std::sinh(_tmp213);
  const Scalar _tmp215 = std::pow(_tmp174, Scalar(-2));
  const Scalar _tmp216 = std::pow(_tmp87, Scalar(-2));
  const Scalar _tmp217 = Scalar(1.0) / (_tmp43);
  const Scalar _tmp218 = _tmp217 * _tmp41;
  const Scalar _tmp219 = std::pow(_tmp83, Scalar(Scalar(-3) / Scalar(2)));
  const Scalar _tmp220 = _tmp219 * _tmp82;
  const Scalar _tmp221 = _tmp219 * _tmp77 * _tmp80;
  const Scalar _tmp222 = _tmp216 * (-_tmp116 * _tmp218 - _tmp220 * _tmp71 + _tmp221 + _tmp85);
  const Scalar _tmp223 = _tmp222 * _tmp71;
  const Scalar _tmp224 = _tmp220 * _tmp68;
  const Scalar _tmp225 = _tmp221 * _tmp89;
  const Scalar _tmp226 = _tmp218 * _tmp92 + _tmp224 * _tmp71 - _tmp225 - _tmp71 * _tmp91;
  const Scalar _tmp227 =
      _tmp119 * _tmp226 - _tmp153 * _tmp218 - _tmp218 * _tmp68 - _tmp223 * _tmp95;
  const Scalar _tmp228 = _tmp220 * _tmp89 - _tmp224 - _tmp90 + _tmp91;
  const Scalar _tmp229 =
      -_tmp103 * _tmp227 + _tmp119 * _tmp228 - _tmp218 * _tmp94 - _tmp223 * _tmp93;
  const Scalar _tmp230 = std::pow(_tmp58, Scalar(Scalar(-3) / Scalar(2)));
  const Scalar _tmp231 = _tmp230 * _tmp52;
  const Scalar _tmp232 = _tmp230 * _tmp51 * _tmp56;
  const Scalar _tmp233 = -_tmp218 * _tmp72 - _tmp231 * _tmp71 + _tmp232 + _tmp59 * _tmp71;
  const Scalar _tmp234 = _tmp233 * _tmp88;
  const Scalar _tmp235 = _tmp222 * _tmp74;
  const Scalar _tmp236 = _tmp232 * _tmp65;
  const Scalar _tmp237 = _tmp120 * _tmp231 + _tmp218 * _tmp70 - _tmp226 * _tmp96 -
                         _tmp234 * _tmp95 + _tmp235 * _tmp95 - _tmp236 - _tmp69 * _tmp71;
  const Scalar _tmp238 = -_tmp103 * _tmp237 - _tmp228 * _tmp96 + _tmp231 * _tmp65 -
                         _tmp231 * _tmp68 - _tmp233 * _tmp94 + _tmp235 * _tmp93 - _tmp66 + _tmp69;
  const Scalar _tmp239 = std::pow(_tmp104, Scalar(-2));
  const Scalar _tmp240 = _tmp238 * _tmp239;
  const Scalar _tmp241 = _tmp118 * _tmp240;
  const Scalar _tmp242 = std::pow(_tmp44, Scalar(Scalar(-3) / Scalar(2)));
  const Scalar _tmp243 = _tmp242 * _tmp35;
  const Scalar _tmp244 = _tmp37 * _tmp41;
  const Scalar _tmp245 = _tmp242 * _tmp39;
  const Scalar _tmp246 = -_tmp112 - _tmp243 * _tmp244 + _tmp245 * _tmp43;
  const Scalar _tmp247 = _tmp246 * _tmp46;
  const Scalar _tmp248 = _tmp110 * _tmp113;
  const Scalar _tmp249 = _tmp116 * _tmp38;
  const Scalar _tmp250 = _tmp249 * _tmp45;
  const Scalar _tmp251 = _tmp114 * _tmp217;
  const Scalar _tmp252 = _tmp108 - _tmp115 * _tmp220 + _tmp115 * _tmp84 + _tmp116 * _tmp248 -
                         _tmp116 * _tmp251 - _tmp220 * _tmp78 + _tmp221 * _tmp75 +
                         _tmp246 * _tmp250;
  const Scalar _tmp253 = _tmp107 - _tmp115 * _tmp231 + _tmp115 * _tmp59 - _tmp117 * _tmp234 +
                         _tmp117 * _tmp235 - _tmp231 * _tmp54 + _tmp232 * _tmp49 +
                         _tmp247 * _tmp72 + _tmp248 * _tmp72 - _tmp251 * _tmp72 - _tmp252 * _tmp96;
  const Scalar _tmp254 =
      _tmp127 * (-_tmp117 * _tmp223 + _tmp119 * _tmp252 + _tmp122 * _tmp241 - _tmp123 * _tmp253 -
                 _tmp124 * _tmp218 - _tmp157 * _tmp229 - _tmp247 - _tmp248 + _tmp251);
  const Scalar _tmp255 = std::pow(_tmp118, Scalar(-2));
  const Scalar _tmp256 = _tmp104 * _tmp255;
  const Scalar _tmp257 = _tmp125 * _tmp256;
  const Scalar _tmp258 = _tmp253 * _tmp257;
  const Scalar _tmp259 = _tmp126 * _tmp238;
  const Scalar _tmp260 = _tmp125 * _tmp259;
  const Scalar _tmp261 = _tmp229 + _tmp254 - _tmp258 + _tmp260;
  const Scalar _tmp262 = _tmp105 * _tmp74;
  const Scalar _tmp263 = _tmp240 * _tmp74;
  const Scalar _tmp264 = _tmp129 * _tmp263 - _tmp130 * _tmp233 + _tmp218 - _tmp261 * _tmp262;
  const Scalar _tmp265 = _tmp105 * _tmp72;
  const Scalar _tmp266 = _tmp240 * _tmp72;
  const Scalar _tmp267 = _tmp116 * _tmp132;
  const Scalar _tmp268 = _tmp132 * _tmp88;
  const Scalar _tmp269 = _tmp141 * _tmp47;
  const Scalar _tmp270 = _tmp269 * _tmp46;
  const Scalar _tmp271 = _tmp110 * _tmp143;
  const Scalar _tmp272 = _tmp217 * _tmp45;
  const Scalar _tmp273 = std::pow(_tmp139, Scalar(Scalar(-3) / Scalar(2)));
  const Scalar _tmp274 = _tmp138 * _tmp273;
  const Scalar _tmp275 = _tmp274 * fh1;
  const Scalar _tmp276 = _tmp135 * _tmp46;
  const Scalar _tmp277 = Scalar(1.0) * _tmp222;
  const Scalar _tmp278 = -_tmp155 * _tmp222 * _tmp95 + _tmp155 * _tmp226 * _tmp88 -
                         _tmp158 * _tmp228 + _tmp277 * _tmp93;
  const Scalar _tmp279 = _tmp105 * _tmp156;
  const Scalar _tmp280 = _tmp127 * (_tmp117 * _tmp277 + _tmp156 * _tmp241 - _tmp157 * _tmp278 -
                                    _tmp158 * _tmp252 - _tmp253 * _tmp279);
  const Scalar _tmp281 = _tmp159 * _tmp256;
  const Scalar _tmp282 = _tmp253 * _tmp281;
  const Scalar _tmp283 = _tmp159 * _tmp259;
  const Scalar _tmp284 = _tmp278 + _tmp280 - _tmp282 + _tmp283;
  const Scalar _tmp285 = _tmp164 * _tmp88;
  const Scalar _tmp286 = _tmp161 * _tmp263 - _tmp162 * _tmp233 - _tmp262 * _tmp284;
  const Scalar _tmp287 = _tmp116 * _tmp164;
  const Scalar _tmp288 = _tmp136 * _tmp141;
  const Scalar _tmp289 = _tmp288 * _tmp46;
  const Scalar _tmp290 = _tmp17 * _tmp273;
  const Scalar _tmp291 = _tmp136 * _tmp47;
  const Scalar _tmp292 = -_tmp150 + _tmp274 * _tmp29 - _tmp290 * _tmp291;
  const Scalar _tmp293 = _tmp292 * fh1;
  const Scalar _tmp294 = _tmp148 * _tmp46;
  const Scalar _tmp295 = _tmp136 * _tmp167;
  const Scalar _tmp296 = _tmp110 * _tmp295;
  const Scalar _tmp297 = _tmp110 * _tmp173;
  const Scalar _tmp298 = _tmp253 * _tmp255;
  const Scalar _tmp299 = Scalar(1.0) * _tmp72;
  const Scalar _tmp300 = _tmp116 * _tmp235;
  const Scalar _tmp301 = _tmp220 * _tmp96;
  const Scalar _tmp302 = _tmp144 * _tmp145;
  const Scalar _tmp303 = _tmp133 * _tmp74;
  const Scalar _tmp304 = Scalar(1.0) * _tmp146;
  const Scalar _tmp305 = _tmp151 * _tmp46;
  const Scalar _tmp306 = _tmp273 * _tmp291;
  const Scalar _tmp307 = _tmp306 * fh1;
  const Scalar _tmp308 = _tmp166 * _tmp46;
  const Scalar _tmp309 = _tmp145 * _tmp172;
  const Scalar _tmp310 = _tmp145 * _tmp171;
  const Scalar _tmp311 = _tmp169 * _tmp46;
  const Scalar _tmp312 = _tmp110 * _tmp152;
  const Scalar _tmp313 =
      -_tmp142 * _tmp46 + _tmp143 * _tmp272 + _tmp152 * _tmp272 + _tmp173 * _tmp272 -
      _tmp270 * (-_tmp129 * _tmp266 - _tmp130 * _tmp231 + _tmp131 + _tmp134 + _tmp145 * _tmp264 -
                 _tmp220 * _tmp268 - _tmp222 * _tmp267 + _tmp261 * _tmp265) -
      _tmp271 + _tmp272 * _tmp295 + _tmp275 * _tmp276 -
      _tmp289 * (_tmp145 * _tmp286 - _tmp161 * _tmp266 - _tmp162 * _tmp231 + _tmp163 + _tmp165 -
                 _tmp220 * _tmp285 - _tmp222 * _tmp287 + _tmp265 * _tmp284) -
      _tmp293 * _tmp294 - _tmp296 - _tmp297 -
      _tmp305 * (-_tmp144 * _tmp231 + _tmp144 * _tmp300 + _tmp144 * _tmp301 - _tmp144 * _tmp303 +
                 _tmp147 - _tmp233 * _tmp302 - _tmp298 * _tmp299 + _tmp298 * _tmp304) +
      _tmp307 * _tmp308 -
      _tmp311 * (-_tmp171 * _tmp266 - _tmp172 * _tmp231 + _tmp172 * _tmp300 + _tmp172 * _tmp301 -
                 _tmp172 * _tmp303 + _tmp172 * _tmp59 - _tmp233 * _tmp309 + _tmp263 * _tmp310) -
      _tmp312;
  const Scalar _tmp314 = _tmp215 * _tmp313;
  const Scalar _tmp315 = Scalar(0.71007031138673404) * _tmp211;
  const Scalar _tmp316 = Scalar(1.0) / (_tmp210);
  const Scalar _tmp317 = _tmp188 * _tmp189;
  const Scalar _tmp318 = _tmp171 * _tmp97;
  const Scalar _tmp319 = -_tmp172 * _tmp237 + _tmp240 * _tmp318;
  const Scalar _tmp320 = _tmp154 * _tmp169;
  const Scalar _tmp321 = _tmp161 * _tmp97;
  const Scalar _tmp322 = _tmp105 * _tmp97;
  const Scalar _tmp323 = -_tmp158 * _tmp226 - _tmp162 * _tmp237 + _tmp186 * _tmp280 -
                         _tmp186 * _tmp282 + _tmp186 * _tmp283 + _tmp240 * _tmp321 +
                         _tmp277 * _tmp95 - _tmp284 * _tmp322;
  const Scalar _tmp324 = _tmp136 * _tmp190;
  const Scalar _tmp325 = _tmp184 * _tmp189;
  const Scalar _tmp326 = -_tmp195 * _tmp274 + _tmp196;
  const Scalar _tmp327 = _tmp189 * _tmp193;
  const Scalar _tmp328 = _tmp195 * _tmp306;
  const Scalar _tmp329 = _tmp181 * _tmp256;
  const Scalar _tmp330 = _tmp253 * _tmp329;
  const Scalar _tmp331 = _tmp181 * _tmp259;
  const Scalar _tmp332 = Scalar(1.0) * _tmp97;
  const Scalar _tmp333 = _tmp202 * _tmp256;
  const Scalar _tmp334 =
      -_tmp144 * _tmp237 + _tmp202 * _tmp259 - _tmp253 * _tmp333 + _tmp298 * _tmp332;
  const Scalar _tmp335 = _tmp129 * _tmp97;
  const Scalar _tmp336 = -_tmp130 * _tmp237 + _tmp186 * _tmp254 - _tmp186 * _tmp258 +
                         _tmp186 * _tmp260 + _tmp227 + _tmp240 * _tmp335 - _tmp261 * _tmp322;
  const Scalar _tmp337 = _tmp190 * _tmp47;
  const Scalar _tmp338 =
      std::pow(Scalar(std::pow(_tmp205, Scalar(2)) * _tmp215 + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp339 =
      _tmp338 * (_tmp175 * (_tmp185 * (-_tmp154 * _tmp334 - _tmp330 + _tmp331) + _tmp194 +
                            _tmp200 * _tmp328 + _tmp204 * _tmp326 - _tmp274 * _tmp327 +
                            _tmp292 * _tmp325 - _tmp306 * _tmp317 - _tmp319 * _tmp320 +
                            _tmp324 * (-_tmp154 * _tmp323 + _tmp181 * _tmp280 - _tmp181 * _tmp282 +
                                       _tmp181 * _tmp283) +
                            _tmp337 * (-_tmp154 * _tmp336 + _tmp181 * _tmp254 - _tmp181 * _tmp258 +
                                       _tmp181 * _tmp260)) -
                 _tmp205 * _tmp314);
  const Scalar _tmp340 = Scalar(1.4083112389913199) * _tmp313;
  const Scalar _tmp341 = Scalar(1.0) * _tmp206;
  const Scalar _tmp342 = Scalar(1.0) * std::sinh(_tmp341);
  const Scalar _tmp343 =
      Scalar(34.083374946563197) * _tmp175 - std::cosh(_tmp213) + std::cosh(_tmp341);
  const Scalar _tmp344 = _tmp141 * _tmp88;
  const Scalar _tmp345 = _tmp136 * _tmp344;
  const Scalar _tmp346 = _tmp169 * _tmp172;
  const Scalar _tmp347 = _tmp144 * _tmp151;
  const Scalar _tmp348 = _tmp169 * _tmp171;
  const Scalar _tmp349 = _tmp240 * _tmp348;
  const Scalar _tmp350 = _tmp141 * _tmp268;
  const Scalar _tmp351 = _tmp344 * _tmp47;
  const Scalar _tmp352 = _tmp132 * _tmp269;
  const Scalar _tmp353 = _tmp144 * _tmp293;
  const Scalar _tmp354 = _tmp164 * _tmp288;
  const Scalar _tmp355 = _tmp185 * _tmp298;
  const Scalar _tmp356 = -_tmp222 * _tmp352 - _tmp222 * _tmp354 - _tmp234 * _tmp346 -
                         _tmp234 * _tmp347 + _tmp235 * _tmp346 + _tmp235 * _tmp347 +
                         _tmp264 * _tmp351 - _tmp268 * _tmp275 - _tmp285 * _tmp307 +
                         _tmp286 * _tmp345 + _tmp349 * _tmp96 + _tmp350 - _tmp353 * _tmp96 +
                         _tmp355 * _tmp96;
  const Scalar _tmp357 = Scalar(125.0) - _tmp76;
  const Scalar _tmp358 = Scalar(110.0) - _tmp79;
  const Scalar _tmp359 =
      std::sqrt(Scalar(std::pow(_tmp357, Scalar(2)) + std::pow(_tmp358, Scalar(2))));
  const Scalar _tmp360 = _tmp141 * _tmp285;
  const Scalar _tmp361 = _tmp136 * _tmp360 - _tmp346 * _tmp96 - _tmp347 * _tmp96 + _tmp350 * _tmp47;
  const Scalar _tmp362 = Scalar(1.0) / (_tmp361);
  const Scalar _tmp363 = _tmp181 * _tmp201;
  const Scalar _tmp364 = _tmp100 * _tmp176;
  const Scalar _tmp365 = _tmp100 * _tmp169;
  const Scalar _tmp366 = _tmp100 * _tmp198;
  const Scalar _tmp367 = _tmp100 * _tmp141;
  const Scalar _tmp368 = _tmp192 * _tmp367;
  const Scalar _tmp369 = _tmp100 * _tmp151;
  const Scalar _tmp370 = _tmp187 * _tmp367;
  const Scalar _tmp371 = _tmp136 * _tmp370 + _tmp180 * _tmp365 + _tmp183 * _tmp369 +
                         _tmp197 * _tmp366 - _tmp363 * _tmp364 + _tmp368 * _tmp47;
  const Scalar _tmp372 = std::asinh(_tmp362 * _tmp371);
  const Scalar _tmp373 = Scalar(1.4083112389913199) * _tmp372;
  const Scalar _tmp374 = -_tmp359 - _tmp361 * _tmp373;
  const Scalar _tmp375 = Scalar(0.71007031138673404) * _tmp362;
  const Scalar _tmp376 = _tmp374 * _tmp375;
  const Scalar _tmp377 = Scalar(1.0) * _tmp372;
  const Scalar _tmp378 = Scalar(48.000000000000128) * _tmp362 -
                         Scalar(1.4083112389913199) * std::cosh(_tmp376) +
                         Scalar(1.4083112389913199) * std::cosh(_tmp377);
  const Scalar _tmp379 = std::pow(_tmp361, Scalar(-2));
  const Scalar _tmp380 = Scalar(34.083374946563197) * _tmp379;
  const Scalar _tmp381 = std::sinh(_tmp376);
  const Scalar _tmp382 = Scalar(0.71007031138673404) * _tmp374 * _tmp379;
  const Scalar _tmp383 = Scalar(1.0) / (_tmp359);
  const Scalar _tmp384 = Scalar(1.4083112389913199) * _tmp361;
  const Scalar _tmp385 = _tmp100 * _tmp183;
  const Scalar _tmp386 = _tmp100 * _tmp187;
  const Scalar _tmp387 = _tmp181 * _tmp326;
  const Scalar _tmp388 = _tmp100 * _tmp192;
  const Scalar _tmp389 = _tmp367 * _tmp47;
  const Scalar _tmp390 = _tmp136 * _tmp367;
  const Scalar _tmp391 = _tmp371 * _tmp379;
  const Scalar _tmp392 =
      std::pow(Scalar(std::pow(_tmp371, Scalar(2)) * _tmp379 + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp393 =
      _tmp392 * (-_tmp356 * _tmp391 +
                 _tmp362 * (-_tmp275 * _tmp388 + _tmp293 * _tmp385 - _tmp307 * _tmp386 +
                            _tmp319 * _tmp365 + _tmp323 * _tmp390 + _tmp328 * _tmp366 +
                            _tmp334 * _tmp369 + _tmp336 * _tmp389 - _tmp364 * _tmp387 + _tmp368));
  const Scalar _tmp394 = Scalar(1.0) * std::sinh(_tmp377);
  const Scalar _tmp395 = _tmp178 * _tmp288;
  const Scalar _tmp396 = _tmp178 * _tmp269;
  const Scalar _tmp397 = -_tmp128 * _tmp396 - _tmp151 * _tmp182 - _tmp160 * _tmp395 +
                         _tmp169 * _tmp179 - _tmp197 * _tmp199 + _tmp363;
  const Scalar _tmp398 = _tmp130 * _tmp141;
  const Scalar _tmp399 = _tmp141 * _tmp162;
  const Scalar _tmp400 = _tmp136 * _tmp399 + _tmp346 + _tmp347 + _tmp398 * _tmp47;
  const Scalar _tmp401 = Scalar(1.0) / (_tmp400);
  const Scalar _tmp402 = std::asinh(_tmp397 * _tmp401);
  const Scalar _tmp403 = Scalar(1.0) * _tmp402;
  const Scalar _tmp404 = -_tmp50 + Scalar(-125.0);
  const Scalar _tmp405 = Scalar(110.0) - _tmp55;
  const Scalar _tmp406 =
      std::sqrt(Scalar(std::pow(_tmp404, Scalar(2)) + std::pow(_tmp405, Scalar(2))));
  const Scalar _tmp407 = Scalar(1.4083112389913199) * _tmp400;
  const Scalar _tmp408 = -_tmp402 * _tmp407 - _tmp406;
  const Scalar _tmp409 = Scalar(0.71007031138673404) * _tmp401;
  const Scalar _tmp410 = _tmp408 * _tmp409;
  const Scalar _tmp411 =
      Scalar(34.083374946563197) * _tmp401 + std::cosh(_tmp403) - std::cosh(_tmp410);
  const Scalar _tmp412 = _tmp161 * _tmp288;
  const Scalar _tmp413 = _tmp105 * _tmp141;
  const Scalar _tmp414 = _tmp413 * _tmp47;
  const Scalar _tmp415 = _tmp129 * _tmp269;
  const Scalar _tmp416 = _tmp136 * _tmp413;
  const Scalar _tmp417 = -_tmp130 * _tmp275 - _tmp162 * _tmp307 - _tmp240 * _tmp412 -
                         _tmp240 * _tmp415 + _tmp261 * _tmp414 + _tmp284 * _tmp416 - _tmp349 +
                         _tmp353 - _tmp355 + _tmp398;
  const Scalar _tmp418 = Scalar(1.4083112389913199) * _tmp417;
  const Scalar _tmp419 = _tmp141 * _tmp178;
  const Scalar _tmp420 = _tmp160 * _tmp178;
  const Scalar _tmp421 = _tmp128 * _tmp178;
  const Scalar _tmp422 = std::pow(_tmp400, Scalar(-2));
  const Scalar _tmp423 = _tmp397 * _tmp422;
  const Scalar _tmp424 =
      _tmp401 * (-_tmp128 * _tmp419 + _tmp151 * _tmp330 - _tmp151 * _tmp331 - _tmp182 * _tmp293 -
                 _tmp199 * _tmp328 - _tmp254 * _tmp396 + _tmp258 * _tmp396 - _tmp260 * _tmp396 +
                 _tmp275 * _tmp421 - _tmp280 * _tmp395 + _tmp282 * _tmp395 - _tmp283 * _tmp395 +
                 _tmp307 * _tmp420 + _tmp387) -
      _tmp417 * _tmp423;
  const Scalar _tmp425 =
      std::pow(Scalar(std::pow(_tmp397, Scalar(2)) * _tmp422 + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp426 = Scalar(1.0) * _tmp425 * std::sinh(_tmp403);
  const Scalar _tmp427 = Scalar(0.71007031138673404) * _tmp408 * _tmp422;
  const Scalar _tmp428 = Scalar(1.0) / (_tmp406);
  const Scalar _tmp429 = _tmp407 * _tmp425;
  const Scalar _tmp430 = std::sinh(_tmp410);
  const Scalar _tmp431 = Scalar(34.083374946563197) * _tmp422;
  const Scalar _tmp432 = _tmp219 * _tmp81;
  const Scalar _tmp433 = _tmp216 * (-_tmp221 * _tmp71 + _tmp249 + _tmp432 - _tmp84);
  const Scalar _tmp434 = _tmp433 * _tmp74;
  const Scalar _tmp435 = _tmp116 * _tmp434;
  const Scalar _tmp436 = _tmp230 * _tmp57;
  const Scalar _tmp437 = -_tmp232 * _tmp71 + _tmp38 * _tmp72 + _tmp436 - _tmp59;
  const Scalar _tmp438 = _tmp221 * _tmp96;
  const Scalar _tmp439 = _tmp437 * _tmp88;
  const Scalar _tmp440 = _tmp88 * (_tmp120 * _tmp221 - _tmp38 * _tmp92 - _tmp432 * _tmp89 + _tmp90);
  const Scalar _tmp441 = _tmp433 * _tmp95;
  const Scalar _tmp442 = _tmp120 * _tmp232 - _tmp38 * _tmp70 - _tmp436 * _tmp65 - _tmp439 * _tmp95 -
                         _tmp440 * _tmp74 + _tmp441 * _tmp74 + _tmp66;
  const Scalar _tmp443 = _tmp433 * _tmp93;
  const Scalar _tmp444 = -_tmp221 * _tmp68 + _tmp225;
  const Scalar _tmp445 = -_tmp103 * _tmp442 - _tmp232 * _tmp68 + _tmp236 - _tmp437 * _tmp94 +
                         _tmp443 * _tmp74 - _tmp444 * _tmp96;
  const Scalar _tmp446 = _tmp239 * _tmp445;
  const Scalar _tmp447 = _tmp446 * _tmp72;
  const Scalar _tmp448 = _tmp446 * _tmp74;
  const Scalar _tmp449 = Scalar(1.0) * _tmp440;
  const Scalar _tmp450 =
      _tmp103 * _tmp449 - _tmp155 * _tmp441 - _tmp158 * _tmp444 + Scalar(1.0) * _tmp443;
  const Scalar _tmp451 = _tmp111 - _tmp243 * _tmp42 + _tmp244 * _tmp245;
  const Scalar _tmp452 = -_tmp109 - _tmp115 * _tmp221 - _tmp221 * _tmp78 + _tmp248 * _tmp86 +
                         _tmp250 * _tmp451 + _tmp432 * _tmp75;
  const Scalar _tmp453 = _tmp118 * _tmp446;
  const Scalar _tmp454 = _tmp117 * _tmp433;
  const Scalar _tmp455 = _tmp451 * _tmp46;
  const Scalar _tmp456 = -_tmp106 - _tmp115 * _tmp232 - _tmp117 * _tmp439 - _tmp232 * _tmp54 +
                         _tmp248 * _tmp73 + _tmp436 * _tmp49 - _tmp452 * _tmp96 + _tmp454 * _tmp74 +
                         _tmp455 * _tmp72;
  const Scalar _tmp457 = _tmp127 * (_tmp156 * _tmp453 - _tmp157 * _tmp450 - _tmp158 * _tmp452 -
                                    _tmp279 * _tmp456 + Scalar(1.0) * _tmp454);
  const Scalar _tmp458 = _tmp281 * _tmp456;
  const Scalar _tmp459 = _tmp126 * _tmp445;
  const Scalar _tmp460 = _tmp159 * _tmp459;
  const Scalar _tmp461 = _tmp450 + _tmp457 - _tmp458 + _tmp460;
  const Scalar _tmp462 = _tmp161 * _tmp448 - _tmp162 * _tmp437 - _tmp262 * _tmp461;
  const Scalar _tmp463 = _tmp125 * _tmp459;
  const Scalar _tmp464 = _tmp257 * _tmp456;
  const Scalar _tmp465 = _tmp38 * _tmp88;
  const Scalar _tmp466 = _tmp38 * _tmp68 + _tmp440 * _tmp71 - _tmp441 * _tmp71 + _tmp465 * _tmp95;
  const Scalar _tmp467 =
      -_tmp103 * _tmp466 + _tmp119 * _tmp444 - _tmp443 * _tmp71 + _tmp465 * _tmp93;
  const Scalar _tmp468 =
      _tmp127 * (_tmp117 * _tmp465 + _tmp119 * _tmp452 + _tmp122 * _tmp453 - _tmp123 * _tmp456 -
                 _tmp157 * _tmp467 - _tmp248 * _tmp71 - _tmp454 * _tmp71 - _tmp455);
  const Scalar _tmp469 = _tmp463 - _tmp464 + _tmp467 + _tmp468;
  const Scalar _tmp470 = _tmp129 * _tmp448 - _tmp130 * _tmp437 - _tmp262 * _tmp469 - _tmp38;
  const Scalar _tmp471 = -_tmp137 * _tmp290 + _tmp149 + _tmp29 * _tmp306;
  const Scalar _tmp472 = _tmp471 * fh1;
  const Scalar _tmp473 = _tmp255 * _tmp456;
  const Scalar _tmp474 = _tmp137 * _tmp273;
  const Scalar _tmp475 = _tmp474 * fh1;
  const Scalar _tmp476 = -_tmp168 -
                         _tmp270 * (-_tmp129 * _tmp447 - _tmp130 * _tmp232 + _tmp145 * _tmp470 -
                                    _tmp221 * _tmp268 + _tmp265 * _tmp469 - _tmp267 * _tmp433) -
                         _tmp271 * _tmp71 + _tmp276 * _tmp307 -
                         _tmp289 * (_tmp145 * _tmp462 - _tmp161 * _tmp447 - _tmp162 * _tmp232 -
                                    _tmp221 * _tmp285 + _tmp265 * _tmp461 - _tmp287 * _tmp433) -
                         _tmp294 * _tmp472 - _tmp296 * _tmp71 - _tmp297 * _tmp71 -
                         _tmp305 * (-_tmp144 * _tmp232 + _tmp144 * _tmp435 + _tmp144 * _tmp438 -
                                    _tmp299 * _tmp473 - _tmp302 * _tmp437 + _tmp304 * _tmp473) +
                         _tmp308 * _tmp475 -
                         _tmp311 * (-_tmp171 * _tmp447 - _tmp172 * _tmp232 + _tmp172 * _tmp435 +
                                    _tmp172 * _tmp438 - _tmp309 * _tmp437 + _tmp310 * _tmp448) -
                         _tmp312 * _tmp71;
  const Scalar _tmp477 = Scalar(1.4083112389913199) * _tmp476;
  const Scalar _tmp478 = -_tmp162 * _tmp442 + _tmp186 * _tmp457 - _tmp186 * _tmp458 +
                         _tmp186 * _tmp460 + _tmp321 * _tmp446 - _tmp322 * _tmp461 +
                         Scalar(1.0) * _tmp441 - _tmp449;
  const Scalar _tmp479 = -_tmp130 * _tmp442 + _tmp186 * _tmp463 - _tmp186 * _tmp464 +
                         _tmp186 * _tmp468 - _tmp322 * _tmp469 + _tmp335 * _tmp446 + _tmp466;
  const Scalar _tmp480 = _tmp195 * _tmp474 - _tmp196;
  const Scalar _tmp481 = _tmp329 * _tmp456;
  const Scalar _tmp482 = _tmp181 * _tmp459;
  const Scalar _tmp483 =
      -_tmp144 * _tmp442 + _tmp176 * _tmp482 + _tmp332 * _tmp473 - _tmp333 * _tmp456;
  const Scalar _tmp484 = -_tmp172 * _tmp442 + _tmp318 * _tmp446;
  const Scalar _tmp485 = _tmp215 * _tmp476;
  const Scalar _tmp486 =
      _tmp338 *
      (_tmp175 * (_tmp185 * (-_tmp154 * _tmp483 - _tmp481 + _tmp482) + _tmp191 + _tmp200 * _tmp480 -
                  _tmp204 * _tmp328 - _tmp306 * _tmp327 - _tmp317 * _tmp474 - _tmp320 * _tmp484 +
                  _tmp324 * (-_tmp154 * _tmp478 + _tmp181 * _tmp457 - _tmp181 * _tmp458 +
                             _tmp181 * _tmp460) +
                  _tmp325 * _tmp471 +
                  _tmp337 * (-_tmp154 * _tmp479 + _tmp181 * _tmp463 - _tmp181 * _tmp464 +
                             _tmp181 * _tmp468)) -
       _tmp205 * _tmp485);
  const Scalar _tmp487 = _tmp348 * _tmp446;
  const Scalar _tmp488 = _tmp185 * _tmp473;
  const Scalar _tmp489 = _tmp144 * _tmp472;
  const Scalar _tmp490 = -_tmp268 * _tmp307 - _tmp285 * _tmp475 + _tmp345 * _tmp462 +
                         _tmp346 * _tmp434 - _tmp346 * _tmp439 + _tmp347 * _tmp434 -
                         _tmp347 * _tmp439 + _tmp351 * _tmp470 - _tmp352 * _tmp433 -
                         _tmp354 * _tmp433 + _tmp360 + _tmp487 * _tmp96 + _tmp488 * _tmp96 -
                         _tmp489 * _tmp96;
  const Scalar _tmp491 =
      _tmp392 * (_tmp362 * (_tmp203 * _tmp328 - _tmp307 * _tmp388 + _tmp365 * _tmp484 +
                            _tmp366 * _tmp480 + _tmp369 * _tmp483 + _tmp370 + _tmp385 * _tmp472 -
                            _tmp386 * _tmp475 + _tmp389 * _tmp479 + _tmp390 * _tmp478) -
                 _tmp391 * _tmp490);
  const Scalar _tmp492 = -_tmp130 * _tmp307 - _tmp162 * _tmp475 + _tmp399 - _tmp412 * _tmp446 +
                         _tmp414 * _tmp469 - _tmp415 * _tmp446 + _tmp416 * _tmp461 - _tmp487 -
                         _tmp488 + _tmp489;
  const Scalar _tmp493 = Scalar(1.4083112389913199) * _tmp492;
  const Scalar _tmp494 =
      _tmp401 * (_tmp151 * _tmp481 - _tmp151 * _tmp482 - _tmp160 * _tmp419 - _tmp181 * _tmp328 -
                 _tmp182 * _tmp472 - _tmp199 * _tmp480 + _tmp307 * _tmp421 - _tmp395 * _tmp457 +
                 _tmp395 * _tmp458 - _tmp395 * _tmp460 - _tmp396 * _tmp463 + _tmp396 * _tmp464 -
                 _tmp396 * _tmp468 + _tmp420 * _tmp475) -
      _tmp423 * _tmp492;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 3> _res;

  _res(0, 0) = _tmp19 * _tmp34;
  _res(1, 0) =
      -_tmp207 *
          (-_tmp214 * (_tmp212 * (-_tmp206 * _tmp340 - _tmp207 * _tmp339 + _tmp209 * _tmp316) -
                       _tmp314 * _tmp315) -
           Scalar(34.083374946563197) * _tmp314 + _tmp339 * _tmp342) -
      _tmp340 * _tmp343;
  _res(2, 0) =
      -_tmp356 * _tmp378 -
      _tmp384 * (-_tmp356 * _tmp380 -
                 _tmp381 * (-_tmp356 * _tmp382 + _tmp375 * (-_tmp356 * _tmp373 + _tmp357 * _tmp383 -
                                                            _tmp384 * _tmp393)) +
                 _tmp393 * _tmp394);
  _res(3, 0) =
      -_tmp407 *
          (-_tmp417 * _tmp431 + _tmp424 * _tmp426 -
           _tmp430 * (_tmp409 * (-_tmp402 * _tmp418 + _tmp404 * _tmp428 - _tmp424 * _tmp429) -
                      _tmp417 * _tmp427)) -
      _tmp411 * _tmp418;
  _res(0, 1) = _tmp31 * _tmp34;
  _res(1, 1) =
      -_tmp207 *
          (-_tmp214 * (_tmp212 * (-_tmp206 * _tmp477 - _tmp207 * _tmp486 + _tmp208 * _tmp316) -
                       _tmp315 * _tmp485) +
           _tmp342 * _tmp486 - Scalar(34.083374946563197) * _tmp485) -
      _tmp343 * _tmp477;
  _res(2, 1) =
      -_tmp378 * _tmp490 -
      _tmp384 * (-_tmp380 * _tmp490 -
                 _tmp381 * (_tmp375 * (_tmp358 * _tmp383 - _tmp373 * _tmp490 - _tmp384 * _tmp491) -
                            _tmp382 * _tmp490) +
                 _tmp394 * _tmp491);
  _res(3, 1) =
      -_tmp407 *
          (_tmp426 * _tmp494 -
           _tmp430 * (_tmp409 * (-_tmp402 * _tmp493 + _tmp405 * _tmp428 - _tmp429 * _tmp494) -
                      _tmp427 * _tmp492) -
           _tmp431 * _tmp492) -
      _tmp411 * _tmp493;
  _res(0, 2) = 1;
  _res(1, 2) = 1;
  _res(2, 2) = 1;
  _res(3, 2) = 1;

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
