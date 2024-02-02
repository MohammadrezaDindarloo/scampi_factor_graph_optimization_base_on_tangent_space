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
 * Symbolic function: IK_residual_func_cost1_wrt_fh1_Nl2
 *
 * Args:
 *     fh1: Scalar
 *     fv1: Scalar
 *     DeltaRot: Rot3
 *     position_vector: Matrix31
 *     Rot_init: Rot3
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix41
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFh1Nl2(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 657

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (212)
  const Scalar _tmp0 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp1 = Scalar(1.0) / (fh1);
  const Scalar _tmp2 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp3 = -2 * std::pow(_tmp2, Scalar(2));
  const Scalar _tmp4 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp5 = -2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp6 = Scalar(0.20999999999999999) * _tmp3 + Scalar(0.20999999999999999) * _tmp5 +
                       Scalar(0.20999999999999999);
  const Scalar _tmp7 = -_tmp6;
  const Scalar _tmp8 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp9 = 2 * _tmp2;
  const Scalar _tmp10 = _tmp8 * _tmp9;
  const Scalar _tmp11 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                        2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp12 = _tmp11 * _tmp4;
  const Scalar _tmp13 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp12;
  const Scalar _tmp14 = 2 * _tmp4 * _tmp8;
  const Scalar _tmp15 = _tmp11 * _tmp2;
  const Scalar _tmp16 = _tmp14 - _tmp15;
  const Scalar _tmp17 = -Scalar(0.010999999999999999) * _tmp16;
  const Scalar _tmp18 = -_tmp13 + _tmp17;
  const Scalar _tmp19 = _tmp18 + _tmp7;
  const Scalar _tmp20 = _tmp19 + position_vector(1, 0);
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp10 - Scalar(0.20999999999999999) * _tmp12;
  const Scalar _tmp22 = -_tmp21;
  const Scalar _tmp23 = 1 - 2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp24 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp5;
  const Scalar _tmp25 = _tmp4 * _tmp9;
  const Scalar _tmp26 = _tmp11 * _tmp8;
  const Scalar _tmp27 = _tmp25 + _tmp26;
  const Scalar _tmp28 = -Scalar(0.010999999999999999) * _tmp27;
  const Scalar _tmp29 = -_tmp24 + _tmp28;
  const Scalar _tmp30 = _tmp22 + _tmp29;
  const Scalar _tmp31 = _tmp30 + position_vector(0, 0);
  const Scalar _tmp32 = _tmp1 * fv1;
  const Scalar _tmp33 = std::asinh(_tmp32);
  const Scalar _tmp34 = Scalar(9.6622558468725703) * _tmp33;
  const Scalar _tmp35 =
      -Scalar(0.1034955) * _tmp34 * fh1 -
      Scalar(0.86104699584133515) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp20 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp31 - 1), Scalar(2))));
  const Scalar _tmp36 = _tmp1 * _tmp35;
  const Scalar _tmp37 =
      std::pow(Scalar(_tmp0 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp38 = Scalar(1.0) * _tmp33;
  const Scalar _tmp39 = _tmp31 + Scalar(1.9874742000000001);
  const Scalar _tmp40 = _tmp20 + Scalar(8.3196563700000006);
  const Scalar _tmp41 = std::pow(Scalar(std::pow(_tmp39, Scalar(2)) + std::pow(_tmp40, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp42 = _tmp40 * _tmp41;
  const Scalar _tmp43 = _tmp39 * _tmp41;
  const Scalar _tmp44 = -_tmp19 * _tmp43 + _tmp30 * _tmp42;
  const Scalar _tmp45 = _tmp21 + _tmp29;
  const Scalar _tmp46 = _tmp45 + position_vector(0, 0);
  const Scalar _tmp47 = _tmp46 + Scalar(1.79662371);
  const Scalar _tmp48 = _tmp18 + _tmp6;
  const Scalar _tmp49 = _tmp48 + position_vector(1, 0);
  const Scalar _tmp50 = _tmp49 + Scalar(-4.8333311099999996);
  const Scalar _tmp51 = std::pow(Scalar(std::pow(_tmp47, Scalar(2)) + std::pow(_tmp50, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp52 = _tmp47 * _tmp51;
  const Scalar _tmp53 = _tmp13 + _tmp17;
  const Scalar _tmp54 = _tmp53 + _tmp6;
  const Scalar _tmp55 = _tmp54 + position_vector(1, 0);
  const Scalar _tmp56 = _tmp55 + Scalar(-4.7752063900000001);
  const Scalar _tmp57 = _tmp24 + _tmp28;
  const Scalar _tmp58 = _tmp21 + _tmp57;
  const Scalar _tmp59 = _tmp58 + position_vector(0, 0);
  const Scalar _tmp60 = _tmp59 + Scalar(-2.71799795);
  const Scalar _tmp61 =
      std::sqrt(Scalar(std::pow(_tmp56, Scalar(2)) + std::pow(_tmp60, Scalar(2))));
  const Scalar _tmp62 = Scalar(1.0) / (_tmp61);
  const Scalar _tmp63 = Scalar(1.0) / (_tmp60);
  const Scalar _tmp64 = _tmp61 * _tmp63;
  const Scalar _tmp65 = _tmp64 * (-_tmp54 * _tmp60 * _tmp62 + _tmp56 * _tmp58 * _tmp62);
  const Scalar _tmp66 = _tmp22 + _tmp57;
  const Scalar _tmp67 = _tmp53 + _tmp7;
  const Scalar _tmp68 = _tmp67 + position_vector(1, 0);
  const Scalar _tmp69 = _tmp68 + Scalar(8.3888750099999996);
  const Scalar _tmp70 = _tmp66 + position_vector(0, 0);
  const Scalar _tmp71 = _tmp70 + Scalar(-2.5202214700000001);
  const Scalar _tmp72 = std::pow(Scalar(std::pow(_tmp69, Scalar(2)) + std::pow(_tmp71, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp73 = _tmp69 * _tmp72;
  const Scalar _tmp74 = _tmp71 * _tmp72;
  const Scalar _tmp75 = _tmp65 * _tmp74 - _tmp66 * _tmp73 + _tmp67 * _tmp74;
  const Scalar _tmp76 = _tmp56 * _tmp63;
  const Scalar _tmp77 = Scalar(1.0) / (-_tmp73 + _tmp74 * _tmp76);
  const Scalar _tmp78 = _tmp50 * _tmp51;
  const Scalar _tmp79 = _tmp52 * _tmp76 - _tmp78;
  const Scalar _tmp80 = _tmp77 * _tmp79;
  const Scalar _tmp81 = -_tmp45 * _tmp78 + _tmp48 * _tmp52 + _tmp52 * _tmp65 - _tmp75 * _tmp80;
  const Scalar _tmp82 = Scalar(1.0) / (_tmp81);
  const Scalar _tmp83 = Scalar(1.0) * _tmp82;
  const Scalar _tmp84 = _tmp44 * _tmp64 * (_tmp52 * _tmp83 - _tmp74 * _tmp80 * _tmp83);
  const Scalar _tmp85 = Scalar(0.20999999999999999) * _tmp25 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp86 =
      -Scalar(0.010999999999999999) * _tmp23 - Scalar(0.010999999999999999) * _tmp3;
  const Scalar _tmp87 = Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp88 = _tmp86 + _tmp87;
  const Scalar _tmp89 = _tmp85 + _tmp88;
  const Scalar _tmp90 = _tmp76 * _tmp89;
  const Scalar _tmp91 = _tmp86 - _tmp87;
  const Scalar _tmp92 = _tmp85 + _tmp91;
  const Scalar _tmp93 = _tmp73 * _tmp92 - _tmp74 * _tmp90;
  const Scalar _tmp94 = _tmp76 * _tmp77;
  const Scalar _tmp95 = _tmp90 + _tmp93 * _tmp94;
  const Scalar _tmp96 = Scalar(1.0) * _tmp54;
  const Scalar _tmp97 = -_tmp96;
  const Scalar _tmp98 = Scalar(1.0) / (_tmp67 + _tmp97);
  const Scalar _tmp99 = Scalar(1.0) * _tmp58;
  const Scalar _tmp100 = _tmp98 * (-_tmp66 + _tmp99);
  const Scalar _tmp101 = _tmp77 * (_tmp74 * _tmp89 - _tmp74 * _tmp92);
  const Scalar _tmp102 = -_tmp100 * _tmp95 + _tmp101 * _tmp76 - _tmp89;
  const Scalar _tmp103 = -_tmp85;
  const Scalar _tmp104 = _tmp103 + _tmp88;
  const Scalar _tmp105 = _tmp104 * _tmp78 - _tmp52 * _tmp90 - _tmp80 * _tmp93;
  const Scalar _tmp106 = -_tmp100 * _tmp105 - _tmp101 * _tmp79 - _tmp104 * _tmp52 + _tmp52 * _tmp89;
  const Scalar _tmp107 = Scalar(1.0) / (_tmp106);
  const Scalar _tmp108 = _tmp107 * _tmp81;
  const Scalar _tmp109 = _tmp106 * _tmp82;
  const Scalar _tmp110 = _tmp109 * (-_tmp102 * _tmp108 - _tmp65 + _tmp75 * _tmp94);
  const Scalar _tmp111 = _tmp102 + _tmp110;
  const Scalar _tmp112 = _tmp107 * _tmp79;
  const Scalar _tmp113 = _tmp77 * (-_tmp111 * _tmp112 - _tmp76);
  const Scalar _tmp114 = _tmp107 * _tmp111;
  const Scalar _tmp115 = _tmp43 * _tmp64 * (_tmp113 * _tmp74 + _tmp114 * _tmp52 + Scalar(1.0));
  const Scalar _tmp116 = Scalar(1.0) * _tmp77;
  const Scalar _tmp117 = _tmp116 * _tmp93;
  const Scalar _tmp118 = _tmp100 * _tmp117 - Scalar(1.0) * _tmp101;
  const Scalar _tmp119 = _tmp109 * (-_tmp108 * _tmp118 - _tmp116 * _tmp75);
  const Scalar _tmp120 = _tmp118 + _tmp119;
  const Scalar _tmp121 = _tmp77 * (-_tmp112 * _tmp120 + Scalar(1.0));
  const Scalar _tmp122 = _tmp107 * _tmp120;
  const Scalar _tmp123 = _tmp42 * _tmp64 * (_tmp121 * _tmp74 + _tmp122 * _tmp52);
  const Scalar _tmp124 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp125 = _tmp100 * _tmp96 + _tmp99;
  const Scalar _tmp126 = 0;
  const Scalar _tmp127 = _tmp107 * _tmp126;
  const Scalar _tmp128 = _tmp112 * _tmp126 * _tmp77;
  const Scalar _tmp129 = -_tmp115 * fh1 - _tmp123 * fh1 -
                         _tmp124 * _tmp64 * (_tmp127 * _tmp52 - _tmp128 * _tmp74) - _tmp84 * fh1;
  const Scalar _tmp130 = Scalar(1.0) / (_tmp129);
  const Scalar _tmp131 = _tmp48 + _tmp97;
  const Scalar _tmp132 = _tmp100 * _tmp131;
  const Scalar _tmp133 = Scalar(1.0) / (-_tmp132 - _tmp45 + _tmp99);
  const Scalar _tmp134 = Scalar(1.0) * _tmp133;
  const Scalar _tmp135 = _tmp131 * _tmp134;
  const Scalar _tmp136 = -_tmp105 * _tmp83 + _tmp109 * _tmp135;
  const Scalar _tmp137 = Scalar(1.0) * _tmp98;
  const Scalar _tmp138 = _tmp109 * _tmp134;
  const Scalar _tmp139 = Scalar(1.0) * _tmp44 * (-_tmp136 * _tmp137 + _tmp138);
  const Scalar _tmp140 = _tmp105 * _tmp107;
  const Scalar _tmp141 = _tmp125 * _tmp133;
  const Scalar _tmp142 = _tmp98 * (-_tmp126 * _tmp140 - _tmp131 * _tmp141 + _tmp97);
  const Scalar _tmp143 = _tmp103 + _tmp91;
  const Scalar _tmp144 = _tmp143 * fh1;
  const Scalar _tmp145 = -_tmp144 * _tmp42 - Scalar(5.1796800000000003) * _tmp16 - _tmp19 * fv1;
  const Scalar _tmp146 = _tmp100 * _tmp134;
  const Scalar _tmp147 = _tmp98 * (_tmp132 * _tmp134 + Scalar(1.0));
  const Scalar _tmp148 = Scalar(1.0) * _tmp146 - Scalar(1.0) * _tmp147;
  const Scalar _tmp149 = _tmp144 * _tmp43 + Scalar(5.1796800000000003) * _tmp27 + _tmp30 * fv1;
  const Scalar _tmp150 = _tmp135 * _tmp98;
  const Scalar _tmp151 = -Scalar(1.0) * _tmp134 + Scalar(1.0) * _tmp150;
  const Scalar _tmp152 = _tmp131 * _tmp133;
  const Scalar _tmp153 = _tmp110 * _tmp152 - _tmp111 * _tmp140 + _tmp95;
  const Scalar _tmp154 = Scalar(1.0) * _tmp43 * (_tmp110 * _tmp134 - _tmp137 * _tmp153);
  const Scalar _tmp155 = -_tmp117 + _tmp119 * _tmp152 - _tmp120 * _tmp140;
  const Scalar _tmp156 = Scalar(1.0) * _tmp42 * (_tmp119 * _tmp134 - _tmp137 * _tmp155);
  const Scalar _tmp157 =
      Scalar(1.0) * _tmp124 * (-_tmp125 * _tmp134 - Scalar(1.0) * _tmp142 + Scalar(1.0)) +
      _tmp139 * fh1 + _tmp145 * _tmp148 + _tmp149 * _tmp151 + _tmp154 * fh1 + _tmp156 * fh1;
  const Scalar _tmp158 = std::asinh(_tmp130 * _tmp157);
  const Scalar _tmp159 = Scalar(1.0) * _tmp158;
  const Scalar _tmp160 = Scalar(9.6622558468725703) * _tmp129;
  const Scalar _tmp161 =
      -_tmp158 * _tmp160 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp55), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp59), Scalar(2))));
  const Scalar _tmp162 = Scalar(0.1034955) * _tmp130;
  const Scalar _tmp163 = _tmp161 * _tmp162;
  const Scalar _tmp164 = -_tmp115 - _tmp123 - _tmp84;
  const Scalar _tmp165 = Scalar(9.6622558468725703) * _tmp164;
  const Scalar _tmp166 = std::pow(_tmp129, Scalar(-2));
  const Scalar _tmp167 = _tmp164 * _tmp166;
  const Scalar _tmp168 = _tmp143 * _tmp42;
  const Scalar _tmp169 = _tmp143 * _tmp43;
  const Scalar _tmp170 =
      (_tmp130 * (_tmp139 - _tmp148 * _tmp168 + _tmp151 * _tmp169 + _tmp154 + _tmp156) -
       _tmp157 * _tmp167) /
      std::sqrt(Scalar(std::pow(_tmp157, Scalar(2)) * _tmp166 + 1));
  const Scalar _tmp171 = _tmp113 * _tmp43;
  const Scalar _tmp172 = _tmp44 * _tmp83;
  const Scalar _tmp173 = _tmp172 * fh1;
  const Scalar _tmp174 = _tmp121 * _tmp42;
  const Scalar _tmp175 = -_tmp124 * _tmp128 + _tmp171 * fh1 - _tmp173 * _tmp80 + _tmp174 * fh1;
  const Scalar _tmp176 = _tmp134 * _tmp149;
  const Scalar _tmp177 = _tmp136 * _tmp44 * _tmp98;
  const Scalar _tmp178 = _tmp153 * _tmp43 * _tmp98;
  const Scalar _tmp179 = _tmp155 * _tmp42 * _tmp98;
  const Scalar _tmp180 = _tmp124 * _tmp142 - _tmp131 * _tmp176 * _tmp98 + _tmp145 * _tmp147 +
                         _tmp177 * fh1 + _tmp178 * fh1 + _tmp179 * fh1;
  const Scalar _tmp181 = Scalar(1.0) / (_tmp175);
  const Scalar _tmp182 = std::asinh(_tmp180 * _tmp181);
  const Scalar _tmp183 = Scalar(9.6622558468725703) * _tmp182;
  const Scalar _tmp184 =
      -_tmp175 * _tmp183 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp70), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp68 - 1), Scalar(2))));
  const Scalar _tmp185 = Scalar(0.1034955) * _tmp181;
  const Scalar _tmp186 = _tmp184 * _tmp185;
  const Scalar _tmp187 = _tmp171 - _tmp172 * _tmp80 + _tmp174;
  const Scalar _tmp188 = Scalar(9.6622558468725703) * _tmp175;
  const Scalar _tmp189 = std::pow(_tmp175, Scalar(-2));
  const Scalar _tmp190 = _tmp187 * _tmp189;
  const Scalar _tmp191 = (-_tmp180 * _tmp190 + _tmp181 * (-_tmp147 * _tmp168 - _tmp150 * _tmp169 +
                                                          _tmp177 + _tmp178 + _tmp179)) /
                         std::sqrt(Scalar(std::pow(_tmp180, Scalar(2)) * _tmp189 + 1));
  const Scalar _tmp192 = Scalar(1.0) * _tmp182;
  const Scalar _tmp193 = _tmp114 * _tmp43;
  const Scalar _tmp194 = _tmp122 * _tmp42;
  const Scalar _tmp195 = _tmp124 * _tmp127 + _tmp173 + _tmp193 * fh1 + _tmp194 * fh1;
  const Scalar _tmp196 = Scalar(1.0) / (_tmp195);
  const Scalar _tmp197 = _tmp138 * _tmp44;
  const Scalar _tmp198 = _tmp110 * _tmp133 * _tmp43;
  const Scalar _tmp199 = _tmp119 * _tmp133 * _tmp42;
  const Scalar _tmp200 = _tmp124 * _tmp141 - _tmp145 * _tmp146 + _tmp176 - _tmp197 * fh1 -
                         _tmp198 * fh1 - _tmp199 * fh1;
  const Scalar _tmp201 = std::asinh(_tmp196 * _tmp200);
  const Scalar _tmp202 = Scalar(9.6622558468725703) * _tmp195;
  const Scalar _tmp203 =
      -_tmp201 * _tmp202 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp49), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp46 - 1), Scalar(2))));
  const Scalar _tmp204 = Scalar(0.1034955) * _tmp196;
  const Scalar _tmp205 = _tmp203 * _tmp204;
  const Scalar _tmp206 = Scalar(1.0) * _tmp201;
  const Scalar _tmp207 = _tmp172 + _tmp193 + _tmp194;
  const Scalar _tmp208 = Scalar(9.6622558468725703) * _tmp207;
  const Scalar _tmp209 = std::pow(_tmp195, Scalar(-2));
  const Scalar _tmp210 = _tmp207 * _tmp209;
  const Scalar _tmp211 =
      (_tmp196 * (_tmp134 * _tmp169 + _tmp146 * _tmp168 - _tmp197 - _tmp198 - _tmp199) -
       _tmp200 * _tmp210) /
      std::sqrt(Scalar(std::pow(_tmp200, Scalar(2)) * _tmp209 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -Scalar(8.4718465800000011) * _tmp1 -
      Scalar(9.6622558468725703) * fh1 *
          (-Scalar(1.0) * _tmp0 * _tmp37 * fv1 * std::sinh(_tmp38) -
           Scalar(0.87679799772039002) * _tmp0 -
           (-_tmp0 * _tmp35 +
            Scalar(0.1034955) * _tmp1 * (Scalar(9.6622558468725703) * _tmp32 * _tmp37 - _tmp34)) *
               std::sinh(_tmp36)) +
      Scalar(9.6622558468725703) * std::cosh(_tmp36) -
      Scalar(9.6622558468725703) * std::cosh(_tmp38);
  _res(1, 0) =
      -_tmp160 *
          (-Scalar(0.86565325453551001) * _tmp167 + Scalar(1.0) * _tmp170 * std::sinh(_tmp159) -
           (-Scalar(0.1034955) * _tmp161 * _tmp167 +
            _tmp162 * (-_tmp158 * _tmp165 - _tmp160 * _tmp170)) *
               std::sinh(_tmp163)) -
      _tmp165 * (Scalar(0.86565325453551001) * _tmp130 + std::cosh(_tmp159) - std::cosh(_tmp163));
  _res(2, 0) =
      -Scalar(9.6622558468725703) * _tmp187 *
          (Scalar(0.87653584775870996) * _tmp181 - std::cosh(_tmp186) + std::cosh(_tmp192)) -
      _tmp188 *
          (-Scalar(0.87653584775870996) * _tmp190 + Scalar(1.0) * _tmp191 * std::sinh(_tmp192) -
           (-Scalar(0.1034955) * _tmp184 * _tmp190 +
            _tmp185 * (-_tmp183 * _tmp187 - _tmp188 * _tmp191)) *
               std::sinh(_tmp186));
  _res(3, 0) =
      -_tmp202 *
          (-Scalar(0.86625939559540499) * _tmp210 + Scalar(1.0) * _tmp211 * std::sinh(_tmp206) -
           (-Scalar(0.1034955) * _tmp203 * _tmp210 +
            _tmp204 * (-_tmp201 * _tmp208 - _tmp202 * _tmp211)) *
               std::sinh(_tmp205)) -
      _tmp208 * (Scalar(0.86625939559540499) * _tmp196 - std::cosh(_tmp205) + std::cosh(_tmp206));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
