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
 * Symbolic function: IK_residual_func_cost1_wrt_fh1_Nl6
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFh1Nl6(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 663

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (216)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = _tmp0 * fv1;
  const Scalar _tmp2 = std::asinh(_tmp1);
  const Scalar _tmp3 = Scalar(1.0) * _tmp2;
  const Scalar _tmp4 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp5 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp6 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp7 = 2 * _tmp6;
  const Scalar _tmp8 = _tmp5 * _tmp7;
  const Scalar _tmp9 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp10 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                        2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp11 = _tmp10 * _tmp9;
  const Scalar _tmp12 = Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp13 = -2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp14 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp15 = Scalar(0.20999999999999999) * _tmp13 +
                        Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999);
  const Scalar _tmp16 = _tmp7 * _tmp9;
  const Scalar _tmp17 = _tmp10 * _tmp5;
  const Scalar _tmp18 = _tmp16 - _tmp17;
  const Scalar _tmp19 = -Scalar(0.010999999999999999) * _tmp18;
  const Scalar _tmp20 = -_tmp15 + _tmp19;
  const Scalar _tmp21 = _tmp12 + _tmp20;
  const Scalar _tmp22 = _tmp21 + position_vector(1, 0);
  const Scalar _tmp23 = -Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp24 = -_tmp23;
  const Scalar _tmp25 = 2 * _tmp5 * _tmp9;
  const Scalar _tmp26 = _tmp10 * _tmp6;
  const Scalar _tmp27 = _tmp25 + _tmp26;
  const Scalar _tmp28 = -Scalar(0.010999999999999999) * _tmp27;
  const Scalar _tmp29 = 1 - 2 * std::pow(_tmp6, Scalar(2));
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp31 = _tmp28 + _tmp30;
  const Scalar _tmp32 = _tmp24 + _tmp31;
  const Scalar _tmp33 = _tmp32 + position_vector(0, 0);
  const Scalar _tmp34 = Scalar(9.6622558468725703) * _tmp2;
  const Scalar _tmp35 =
      -Scalar(0.1034955) * _tmp34 * fh1 -
      Scalar(0.868210813597455) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp33), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp22 - 1), Scalar(2))));
  const Scalar _tmp36 =
      std::pow(Scalar(_tmp4 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp37 = _tmp0 * _tmp35;
  const Scalar _tmp38 = _tmp28 - _tmp30;
  const Scalar _tmp39 = _tmp23 + _tmp38;
  const Scalar _tmp40 = _tmp39 + position_vector(0, 0);
  const Scalar _tmp41 = _tmp40 + Scalar(1.79662371);
  const Scalar _tmp42 = -_tmp12;
  const Scalar _tmp43 = _tmp15 + _tmp19;
  const Scalar _tmp44 = _tmp42 + _tmp43;
  const Scalar _tmp45 = _tmp44 + position_vector(1, 0);
  const Scalar _tmp46 = _tmp45 + Scalar(-4.8333311099999996);
  const Scalar _tmp47 = std::pow(Scalar(std::pow(_tmp41, Scalar(2)) + std::pow(_tmp46, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp48 = _tmp41 * _tmp47;
  const Scalar _tmp49 = _tmp20 + _tmp42;
  const Scalar _tmp50 = _tmp49 + position_vector(1, 0);
  const Scalar _tmp51 = _tmp50 + Scalar(8.3196563700000006);
  const Scalar _tmp52 = _tmp24 + _tmp38;
  const Scalar _tmp53 = _tmp52 + position_vector(0, 0);
  const Scalar _tmp54 = _tmp53 + Scalar(1.9874742000000001);
  const Scalar _tmp55 = Scalar(1.0) / (_tmp54);
  const Scalar _tmp56 = _tmp51 * _tmp55;
  const Scalar _tmp57 = Scalar(0.20999999999999999) * _tmp25 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp58 = -_tmp57;
  const Scalar _tmp59 =
      -Scalar(0.010999999999999999) * _tmp13 - Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp60 = Scalar(0.20999999999999999) * _tmp16 + Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp61 = _tmp59 - _tmp60;
  const Scalar _tmp62 = _tmp58 + _tmp61;
  const Scalar _tmp63 = _tmp12 + _tmp43;
  const Scalar _tmp64 = _tmp63 + position_vector(1, 0);
  const Scalar _tmp65 = _tmp64 + Scalar(-4.7752063900000001);
  const Scalar _tmp66 = _tmp23 + _tmp31;
  const Scalar _tmp67 = _tmp66 + position_vector(0, 0);
  const Scalar _tmp68 = _tmp67 + Scalar(-2.71799795);
  const Scalar _tmp69 = std::pow(Scalar(std::pow(_tmp65, Scalar(2)) + std::pow(_tmp68, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp70 = _tmp68 * _tmp69;
  const Scalar _tmp71 = _tmp62 * _tmp70;
  const Scalar _tmp72 = _tmp59 + _tmp60;
  const Scalar _tmp73 = _tmp57 + _tmp72;
  const Scalar _tmp74 = _tmp65 * _tmp69;
  const Scalar _tmp75 = Scalar(1.0) / (_tmp56 * _tmp70 - _tmp74);
  const Scalar _tmp76 = _tmp75 * (-_tmp70 * _tmp73 + _tmp71);
  const Scalar _tmp77 = -_tmp56 * _tmp71 + _tmp73 * _tmp74;
  const Scalar _tmp78 = _tmp56 * _tmp75;
  const Scalar _tmp79 = _tmp56 * _tmp62;
  const Scalar _tmp80 = _tmp77 * _tmp78 + _tmp79;
  const Scalar _tmp81 = Scalar(1.0) * _tmp49;
  const Scalar _tmp82 = -_tmp81;
  const Scalar _tmp83 = Scalar(1.0) / (_tmp63 + _tmp82);
  const Scalar _tmp84 = Scalar(1.0) * _tmp52;
  const Scalar _tmp85 = -_tmp66 + _tmp84;
  const Scalar _tmp86 = _tmp83 * _tmp85;
  const Scalar _tmp87 = _tmp56 * _tmp76 - _tmp62 - _tmp80 * _tmp86;
  const Scalar _tmp88 = _tmp58 + _tmp72;
  const Scalar _tmp89 = _tmp46 * _tmp47;
  const Scalar _tmp90 = _tmp48 * _tmp56 - _tmp89;
  const Scalar _tmp91 = _tmp75 * _tmp90;
  const Scalar _tmp92 = -_tmp48 * _tmp79 - _tmp77 * _tmp91 + _tmp88 * _tmp89;
  const Scalar _tmp93 = _tmp48 * _tmp62 - _tmp48 * _tmp88 - _tmp76 * _tmp90 - _tmp86 * _tmp92;
  const Scalar _tmp94 = Scalar(1.0) / (_tmp93);
  const Scalar _tmp95 =
      std::sqrt(Scalar(std::pow(_tmp51, Scalar(2)) + std::pow(_tmp54, Scalar(2))));
  const Scalar _tmp96 = Scalar(1.0) / (_tmp95);
  const Scalar _tmp97 = _tmp55 * _tmp95;
  const Scalar _tmp98 = _tmp97 * (-_tmp49 * _tmp54 * _tmp96 + _tmp51 * _tmp52 * _tmp96);
  const Scalar _tmp99 = _tmp63 * _tmp70 - _tmp66 * _tmp74 + _tmp70 * _tmp98;
  const Scalar _tmp100 = -_tmp39 * _tmp89 + _tmp44 * _tmp48 + _tmp48 * _tmp98 - _tmp91 * _tmp99;
  const Scalar _tmp101 = _tmp100 * _tmp94;
  const Scalar _tmp102 = Scalar(1.0) / (_tmp100);
  const Scalar _tmp103 = _tmp102 * _tmp93;
  const Scalar _tmp104 = _tmp103 * (-_tmp101 * _tmp87 + _tmp78 * _tmp99 - _tmp98);
  const Scalar _tmp105 = _tmp104 + _tmp87;
  const Scalar _tmp106 = _tmp105 * _tmp94;
  const Scalar _tmp107 = _tmp90 * _tmp94;
  const Scalar _tmp108 = _tmp75 * (-_tmp105 * _tmp107 - _tmp56);
  const Scalar _tmp109 = _tmp33 + Scalar(-2.5202214700000001);
  const Scalar _tmp110 = _tmp22 + Scalar(8.3888750099999996);
  const Scalar _tmp111 =
      std::pow(Scalar(std::pow(_tmp109, Scalar(2)) + std::pow(_tmp110, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp112 = _tmp109 * _tmp111;
  const Scalar _tmp113 = _tmp112 * _tmp97 * (_tmp106 * _tmp48 + _tmp108 * _tmp70 + Scalar(1.0));
  const Scalar _tmp114 = _tmp110 * _tmp111;
  const Scalar _tmp115 = -_tmp112 * _tmp21 + _tmp114 * _tmp32;
  const Scalar _tmp116 = Scalar(1.0) * _tmp102;
  const Scalar _tmp117 = Scalar(1.0) * _tmp75;
  const Scalar _tmp118 = _tmp102 * _tmp117 * _tmp90;
  const Scalar _tmp119 = _tmp115 * _tmp97 * (_tmp116 * _tmp48 - _tmp118 * _tmp70);
  const Scalar _tmp120 = Scalar(1.0) * _tmp83;
  const Scalar _tmp121 = _tmp120 * _tmp75 * _tmp77 * _tmp85 - Scalar(1.0) * _tmp76;
  const Scalar _tmp122 = _tmp103 * (-_tmp101 * _tmp121 - _tmp117 * _tmp99);
  const Scalar _tmp123 = _tmp121 + _tmp122;
  const Scalar _tmp124 = _tmp123 * _tmp94;
  const Scalar _tmp125 = _tmp75 * (-_tmp107 * _tmp123 + Scalar(1.0));
  const Scalar _tmp126 = _tmp114 * _tmp97 * (_tmp124 * _tmp48 + _tmp125 * _tmp70);
  const Scalar _tmp127 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp128 = _tmp81 * _tmp86 + _tmp84;
  const Scalar _tmp129 = 0;
  const Scalar _tmp130 = _tmp107 * _tmp129 * _tmp75;
  const Scalar _tmp131 = _tmp129 * _tmp94;
  const Scalar _tmp132 = -_tmp113 * fh1 - _tmp119 * fh1 - _tmp126 * fh1 -
                         _tmp127 * _tmp97 * (-_tmp130 * _tmp70 + _tmp131 * _tmp48);
  const Scalar _tmp133 = Scalar(1.0) / (_tmp132);
  const Scalar _tmp134 = _tmp57 + _tmp61;
  const Scalar _tmp135 = _tmp134 * fh1;
  const Scalar _tmp136 = _tmp112 * _tmp135 + Scalar(5.1796800000000003) * _tmp27 + _tmp32 * fv1;
  const Scalar _tmp137 = _tmp44 + _tmp82;
  const Scalar _tmp138 = _tmp137 * _tmp86;
  const Scalar _tmp139 = Scalar(1.0) / (-_tmp138 - _tmp39 + _tmp84);
  const Scalar _tmp140 = Scalar(1.0) * _tmp139;
  const Scalar _tmp141 = _tmp137 * _tmp140;
  const Scalar _tmp142 = -Scalar(1.0) * _tmp140 + Scalar(1.0) * _tmp141 * _tmp83;
  const Scalar _tmp143 = -_tmp114 * _tmp135 - Scalar(5.1796800000000003) * _tmp18 - _tmp21 * fv1;
  const Scalar _tmp144 = _tmp138 * _tmp140 + Scalar(1.0);
  const Scalar _tmp145 = _tmp140 * _tmp86;
  const Scalar _tmp146 = -Scalar(1.0) * _tmp120 * _tmp144 + Scalar(1.0) * _tmp145;
  const Scalar _tmp147 = _tmp92 * _tmp94;
  const Scalar _tmp148 = _tmp137 * _tmp139;
  const Scalar _tmp149 = _tmp104 * _tmp148 - _tmp105 * _tmp147 + _tmp80;
  const Scalar _tmp150 = Scalar(1.0) * _tmp112 * (_tmp104 * _tmp140 - _tmp120 * _tmp149);
  const Scalar _tmp151 = _tmp83 * (_tmp103 * _tmp141 - _tmp116 * _tmp92);
  const Scalar _tmp152 = _tmp103 * _tmp140;
  const Scalar _tmp153 = Scalar(1.0) * _tmp115 * (-Scalar(1.0) * _tmp151 + _tmp152);
  const Scalar _tmp154 = _tmp128 * _tmp139;
  const Scalar _tmp155 = -_tmp129 * _tmp147 - _tmp137 * _tmp154 + _tmp82;
  const Scalar _tmp156 = -_tmp117 * _tmp77 + _tmp122 * _tmp148 - _tmp123 * _tmp147;
  const Scalar _tmp157 = Scalar(1.0) * _tmp114 * (-_tmp120 * _tmp156 + _tmp122 * _tmp140);
  const Scalar _tmp158 =
      Scalar(1.0) * _tmp127 * (-_tmp120 * _tmp155 - _tmp128 * _tmp140 + Scalar(1.0)) +
      _tmp136 * _tmp142 + _tmp143 * _tmp146 + _tmp150 * fh1 + _tmp153 * fh1 + _tmp157 * fh1;
  const Scalar _tmp159 = std::asinh(_tmp133 * _tmp158);
  const Scalar _tmp160 = -_tmp113 - _tmp119 - _tmp126;
  const Scalar _tmp161 = Scalar(9.6622558468725703) * _tmp160;
  const Scalar _tmp162 = Scalar(9.6622558468725703) * _tmp132;
  const Scalar _tmp163 = _tmp112 * _tmp134;
  const Scalar _tmp164 = _tmp114 * _tmp134;
  const Scalar _tmp165 = std::pow(_tmp132, Scalar(-2));
  const Scalar _tmp166 = _tmp160 * _tmp165;
  const Scalar _tmp167 =
      (_tmp133 * (_tmp142 * _tmp163 - _tmp146 * _tmp164 + _tmp150 + _tmp153 + _tmp157) -
       _tmp158 * _tmp166) /
      std::sqrt(Scalar(std::pow(_tmp158, Scalar(2)) * _tmp165 + 1));
  const Scalar _tmp168 = Scalar(0.1034955) * _tmp133;
  const Scalar _tmp169 =
      -_tmp159 * _tmp162 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp50 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp53 - 1), Scalar(2))));
  const Scalar _tmp170 = _tmp168 * _tmp169;
  const Scalar _tmp171 = Scalar(1.0) * _tmp159;
  const Scalar _tmp172 = _tmp114 * _tmp125;
  const Scalar _tmp173 = _tmp108 * _tmp112;
  const Scalar _tmp174 = _tmp115 * _tmp118;
  const Scalar _tmp175 = -_tmp127 * _tmp130 + _tmp172 * fh1 + _tmp173 * fh1 - _tmp174 * fh1;
  const Scalar _tmp176 = Scalar(1.0) / (_tmp175);
  const Scalar _tmp177 = _tmp114 * _tmp83;
  const Scalar _tmp178 = _tmp156 * _tmp177;
  const Scalar _tmp179 = _tmp112 * _tmp83;
  const Scalar _tmp180 = _tmp149 * _tmp179;
  const Scalar _tmp181 = _tmp143 * _tmp83;
  const Scalar _tmp182 = _tmp136 * _tmp140;
  const Scalar _tmp183 = _tmp115 * _tmp151;
  const Scalar _tmp184 = _tmp127 * _tmp155 * _tmp83 - _tmp137 * _tmp182 * _tmp83 +
                         _tmp144 * _tmp181 + _tmp178 * fh1 + _tmp180 * fh1 + _tmp183 * fh1;
  const Scalar _tmp185 = std::asinh(_tmp176 * _tmp184);
  const Scalar _tmp186 = Scalar(9.6622558468725703) * _tmp175;
  const Scalar _tmp187 =
      -_tmp185 * _tmp186 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp64), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp67), Scalar(2))));
  const Scalar _tmp188 = Scalar(0.1034955) * _tmp176;
  const Scalar _tmp189 = _tmp187 * _tmp188;
  const Scalar _tmp190 = Scalar(1.0) * _tmp185;
  const Scalar _tmp191 = _tmp172 + _tmp173 - _tmp174;
  const Scalar _tmp192 = Scalar(9.6622558468725703) * _tmp191;
  const Scalar _tmp193 = std::pow(_tmp175, Scalar(-2));
  const Scalar _tmp194 = _tmp191 * _tmp193;
  const Scalar _tmp195 = (_tmp176 * (-_tmp134 * _tmp141 * _tmp179 - _tmp134 * _tmp144 * _tmp177 +
                                     _tmp178 + _tmp180 + _tmp183) -
                          _tmp184 * _tmp194) /
                         std::sqrt(Scalar(std::pow(_tmp184, Scalar(2)) * _tmp193 + 1));
  const Scalar _tmp196 = _tmp106 * _tmp112;
  const Scalar _tmp197 = _tmp115 * _tmp116;
  const Scalar _tmp198 = _tmp114 * _tmp124;
  const Scalar _tmp199 = _tmp127 * _tmp131 + _tmp196 * fh1 + _tmp197 * fh1 + _tmp198 * fh1;
  const Scalar _tmp200 = Scalar(1.0) / (_tmp199);
  const Scalar _tmp201 = _tmp114 * _tmp122 * _tmp139;
  const Scalar _tmp202 = _tmp115 * _tmp152;
  const Scalar _tmp203 = _tmp104 * _tmp112 * _tmp139;
  const Scalar _tmp204 = _tmp127 * _tmp154 - _tmp140 * _tmp181 * _tmp85 + _tmp182 - _tmp201 * fh1 -
                         _tmp202 * fh1 - _tmp203 * fh1;
  const Scalar _tmp205 = std::asinh(_tmp200 * _tmp204);
  const Scalar _tmp206 = Scalar(9.6622558468725703) * _tmp199;
  const Scalar _tmp207 =
      -_tmp205 * _tmp206 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp45), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp40 - 1), Scalar(2))));
  const Scalar _tmp208 = Scalar(0.1034955) * _tmp200;
  const Scalar _tmp209 = _tmp207 * _tmp208;
  const Scalar _tmp210 = Scalar(1.0) * _tmp205;
  const Scalar _tmp211 = _tmp196 + _tmp197 + _tmp198;
  const Scalar _tmp212 = Scalar(9.6622558468725703) * _tmp211;
  const Scalar _tmp213 = std::pow(_tmp199, Scalar(-2));
  const Scalar _tmp214 = _tmp211 * _tmp213;
  const Scalar _tmp215 =
      (_tmp200 * (_tmp140 * _tmp163 + _tmp145 * _tmp164 - _tmp201 - _tmp202 - _tmp203) -
       _tmp204 * _tmp214) /
      std::sqrt(Scalar(std::pow(_tmp204, Scalar(2)) * _tmp213 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -Scalar(8.4693136200000012) * _tmp0 -
      Scalar(9.6622558468725703) * fh1 *
          (-Scalar(1.0) * _tmp36 * _tmp4 * fv1 * std::sinh(_tmp3) -
           Scalar(0.87653584775870996) * _tmp4 -
           (Scalar(0.1034955) * _tmp0 * (Scalar(9.6622558468725703) * _tmp1 * _tmp36 - _tmp34) -
            _tmp35 * _tmp4) *
               std::sinh(_tmp37)) -
      Scalar(9.6622558468725703) * std::cosh(_tmp3) +
      Scalar(9.6622558468725703) * std::cosh(_tmp37);
  _res(1, 0) =
      -_tmp161 * (Scalar(0.87679799772039002) * _tmp133 - std::cosh(_tmp170) + std::cosh(_tmp171)) -
      _tmp162 *
          (-Scalar(0.87679799772039002) * _tmp166 + Scalar(1.0) * _tmp167 * std::sinh(_tmp171) -
           (-Scalar(0.1034955) * _tmp166 * _tmp169 +
            _tmp168 * (-_tmp159 * _tmp161 - _tmp162 * _tmp167)) *
               std::sinh(_tmp170));
  _res(2, 0) =
      -_tmp186 *
          (-Scalar(0.86565325453551001) * _tmp194 + Scalar(1.0) * _tmp195 * std::sinh(_tmp190) -
           (-Scalar(0.1034955) * _tmp187 * _tmp194 +
            _tmp188 * (-_tmp185 * _tmp192 - _tmp186 * _tmp195)) *
               std::sinh(_tmp189)) -
      _tmp192 * (Scalar(0.86565325453551001) * _tmp176 - std::cosh(_tmp189) + std::cosh(_tmp190));
  _res(3, 0) =
      -_tmp206 *
          (-Scalar(0.86625939559540499) * _tmp214 + Scalar(1.0) * _tmp215 * std::sinh(_tmp210) -
           (-Scalar(0.1034955) * _tmp207 * _tmp214 +
            _tmp208 * (-_tmp205 * _tmp212 - _tmp206 * _tmp215)) *
               std::sinh(_tmp209)) -
      _tmp212 * (Scalar(0.86625939559540499) * _tmp200 - std::cosh(_tmp209) + std::cosh(_tmp210));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
