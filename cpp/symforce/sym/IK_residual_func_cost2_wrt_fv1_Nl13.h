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
 * Symbolic function: IK_residual_func_cost2_wrt_fv1_Nl13
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFv1Nl13(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 598

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (191)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(1.0) * _tmp0 /
                       std::sqrt(Scalar(1 + std::pow(fv1, Scalar(2)) / std::pow(fh1, Scalar(2))));
  const Scalar _tmp3 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp4 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp5 = -2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp6 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp7 = 1 - 2 * std::pow(_tmp6, Scalar(2));
  const Scalar _tmp8 = Scalar(0.20999999999999999) * _tmp5 + Scalar(0.20999999999999999) * _tmp7;
  const Scalar _tmp9 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp10 = 2 * _tmp4;
  const Scalar _tmp11 = _tmp10 * _tmp9;
  const Scalar _tmp12 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                        2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp13 = _tmp12 * _tmp6;
  const Scalar _tmp14 = _tmp11 + _tmp13;
  const Scalar _tmp15 = -Scalar(0.010999999999999999) * _tmp14;
  const Scalar _tmp16 = 2 * _tmp6 * _tmp9;
  const Scalar _tmp17 = _tmp12 * _tmp4;
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp16 - Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp19 = _tmp15 + _tmp18;
  const Scalar _tmp20 = _tmp19 + _tmp8;
  const Scalar _tmp21 = _tmp20 + position_vector(0, 0);
  const Scalar _tmp22 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp23 = Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999) * _tmp5 +
                        Scalar(0.20999999999999999);
  const Scalar _tmp24 = _tmp10 * _tmp6;
  const Scalar _tmp25 = _tmp12 * _tmp9;
  const Scalar _tmp26 = _tmp24 - _tmp25;
  const Scalar _tmp27 = Scalar(0.010999999999999999) * _tmp26;
  const Scalar _tmp28 = -_tmp27;
  const Scalar _tmp29 = Scalar(0.20999999999999999) * _tmp16 + Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp30 = _tmp28 + _tmp29;
  const Scalar _tmp31 = _tmp23 + _tmp30;
  const Scalar _tmp32 = _tmp31 + position_vector(1, 0);
  const Scalar _tmp33 = _tmp21 + Scalar(-2.71799795);
  const Scalar _tmp34 = _tmp32 + Scalar(-4.7752063900000001);
  const Scalar _tmp35 = std::pow(Scalar(std::pow(_tmp33, Scalar(2)) + std::pow(_tmp34, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp36 = _tmp33 * _tmp35;
  const Scalar _tmp37 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp38 =
      -Scalar(0.010999999999999999) * _tmp22 - Scalar(0.010999999999999999) * _tmp7;
  const Scalar _tmp39 = Scalar(0.20999999999999999) * _tmp11 - Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp40 = _tmp38 + _tmp39;
  const Scalar _tmp41 = fh1 * (_tmp37 + _tmp40);
  const Scalar _tmp42 = Scalar(5.1796800000000003) * _tmp14 + _tmp20 * fv1 + _tmp36 * _tmp41;
  const Scalar _tmp43 = _tmp15 - _tmp18;
  const Scalar _tmp44 = _tmp43 + _tmp8;
  const Scalar _tmp45 = -_tmp23;
  const Scalar _tmp46 = -_tmp29;
  const Scalar _tmp47 = _tmp45 + _tmp46;
  const Scalar _tmp48 = _tmp28 + _tmp47;
  const Scalar _tmp49 = Scalar(1.0) * _tmp48;
  const Scalar _tmp50 = -_tmp49;
  const Scalar _tmp51 = _tmp30 + _tmp45;
  const Scalar _tmp52 = _tmp50 + _tmp51;
  const Scalar _tmp53 = _tmp23 + _tmp28 + _tmp46;
  const Scalar _tmp54 = Scalar(1.0) / (_tmp50 + _tmp53);
  const Scalar _tmp55 = -_tmp8;
  const Scalar _tmp56 = _tmp19 + _tmp55;
  const Scalar _tmp57 = _tmp43 + _tmp55;
  const Scalar _tmp58 = Scalar(1.0) * _tmp57;
  const Scalar _tmp59 = -_tmp56 + _tmp58;
  const Scalar _tmp60 = _tmp54 * _tmp59;
  const Scalar _tmp61 = _tmp52 * _tmp60;
  const Scalar _tmp62 = Scalar(1.0) / (-_tmp44 + _tmp58 - _tmp61);
  const Scalar _tmp63 = Scalar(1.0) * _tmp62;
  const Scalar _tmp64 = _tmp52 * _tmp54;
  const Scalar _tmp65 = Scalar(1.0) * _tmp63 * _tmp64 - Scalar(1.0) * _tmp63;
  const Scalar _tmp66 = _tmp34 * _tmp35;
  const Scalar _tmp67 = -Scalar(5.1796800000000003) * _tmp26 - _tmp31 * fv1 - _tmp41 * _tmp66;
  const Scalar _tmp68 = _tmp60 * _tmp63;
  const Scalar _tmp69 = _tmp54 * (_tmp61 * _tmp63 + Scalar(1.0));
  const Scalar _tmp70 = Scalar(1.0) * _tmp68 - Scalar(1.0) * _tmp69;
  const Scalar _tmp71 = _tmp56 + position_vector(0, 0);
  const Scalar _tmp72 = _tmp71 + Scalar(1.79662371);
  const Scalar _tmp73 = _tmp53 + position_vector(1, 0);
  const Scalar _tmp74 = _tmp73 + Scalar(-4.8333311099999996);
  const Scalar _tmp75 = std::pow(Scalar(std::pow(_tmp72, Scalar(2)) + std::pow(_tmp74, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp76 = _tmp72 * _tmp75;
  const Scalar _tmp77 = _tmp48 + position_vector(1, 0);
  const Scalar _tmp78 = _tmp77 + Scalar(8.3196563700000006);
  const Scalar _tmp79 = _tmp57 + position_vector(0, 0);
  const Scalar _tmp80 = _tmp79 + Scalar(1.9874742000000001);
  const Scalar _tmp81 =
      std::sqrt(Scalar(std::pow(_tmp78, Scalar(2)) + std::pow(_tmp80, Scalar(2))));
  const Scalar _tmp82 = Scalar(1.0) / (_tmp81);
  const Scalar _tmp83 = Scalar(1.0) / (_tmp80);
  const Scalar _tmp84 = _tmp81 * _tmp83;
  const Scalar _tmp85 = _tmp84 * (-_tmp48 * _tmp80 * _tmp82 + _tmp57 * _tmp78 * _tmp82);
  const Scalar _tmp86 = _tmp74 * _tmp75;
  const Scalar _tmp87 = _tmp53 * _tmp76 - _tmp56 * _tmp86 + _tmp76 * _tmp85;
  const Scalar _tmp88 = _tmp78 * _tmp83;
  const Scalar _tmp89 = Scalar(1.0) / (_tmp76 * _tmp88 - _tmp86);
  const Scalar _tmp90 = _tmp88 * _tmp89;
  const Scalar _tmp91 = _tmp38 - _tmp39;
  const Scalar _tmp92 = _tmp37 + _tmp91;
  const Scalar _tmp93 = -_tmp37;
  const Scalar _tmp94 = _tmp91 + _tmp93;
  const Scalar _tmp95 = _tmp88 * _tmp94;
  const Scalar _tmp96 = -_tmp76 * _tmp95 + _tmp86 * _tmp92;
  const Scalar _tmp97 = _tmp90 * _tmp96 + _tmp95;
  const Scalar _tmp98 = -_tmp76 * _tmp92 + _tmp76 * _tmp94;
  const Scalar _tmp99 = -_tmp60 * _tmp97 + _tmp90 * _tmp98 - _tmp94;
  const Scalar _tmp100 = _tmp51 + position_vector(1, 0);
  const Scalar _tmp101 = _tmp100 + Scalar(8.3888750099999996);
  const Scalar _tmp102 = _tmp44 + position_vector(0, 0);
  const Scalar _tmp103 = _tmp102 + Scalar(-2.5202214700000001);
  const Scalar _tmp104 =
      std::pow(Scalar(std::pow(_tmp101, Scalar(2)) + std::pow(_tmp103, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp105 = _tmp103 * _tmp104;
  const Scalar _tmp106 = _tmp101 * _tmp104;
  const Scalar _tmp107 = _tmp105 * _tmp88 - _tmp106;
  const Scalar _tmp108 = _tmp107 * _tmp89;
  const Scalar _tmp109 = _tmp105 * _tmp51 + _tmp105 * _tmp85 - _tmp106 * _tmp44 - _tmp108 * _tmp87;
  const Scalar _tmp110 = _tmp40 + _tmp93;
  const Scalar _tmp111 = -_tmp105 * _tmp95 + _tmp106 * _tmp110 - _tmp108 * _tmp96;
  const Scalar _tmp112 =
      -_tmp105 * _tmp110 + _tmp105 * _tmp94 - _tmp108 * _tmp98 - _tmp111 * _tmp60;
  const Scalar _tmp113 = Scalar(1.0) / (_tmp112);
  const Scalar _tmp114 = _tmp109 * _tmp113;
  const Scalar _tmp115 = Scalar(1.0) / (_tmp109);
  const Scalar _tmp116 = _tmp112 * _tmp115;
  const Scalar _tmp117 = _tmp116 * (-_tmp114 * _tmp99 - _tmp85 + _tmp87 * _tmp90);
  const Scalar _tmp118 = _tmp117 + _tmp99;
  const Scalar _tmp119 = _tmp111 * _tmp113;
  const Scalar _tmp120 = _tmp52 * _tmp62;
  const Scalar _tmp121 = _tmp117 * _tmp120 - _tmp118 * _tmp119 + _tmp97;
  const Scalar _tmp122 = Scalar(1.0) * _tmp54;
  const Scalar _tmp123 = _tmp36 * fh1;
  const Scalar _tmp124 = _tmp116 * _tmp63;
  const Scalar _tmp125 = Scalar(1.0) * _tmp115;
  const Scalar _tmp126 = -_tmp111 * _tmp125 + _tmp124 * _tmp52;
  const Scalar _tmp127 = fh1 * (_tmp20 * _tmp66 - _tmp31 * _tmp36);
  const Scalar _tmp128 = Scalar(1.0) * _tmp89;
  const Scalar _tmp129 = _tmp122 * _tmp59 * _tmp89 * _tmp96 - _tmp128 * _tmp98;
  const Scalar _tmp130 = _tmp116 * (-_tmp114 * _tmp129 - _tmp128 * _tmp87);
  const Scalar _tmp131 = _tmp129 + _tmp130;
  const Scalar _tmp132 = -_tmp119 * _tmp131 + _tmp120 * _tmp130 - _tmp128 * _tmp96;
  const Scalar _tmp133 = _tmp66 * fh1;
  const Scalar _tmp134 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp135 = _tmp49 * _tmp60 + _tmp58;
  const Scalar _tmp136 = 0;
  const Scalar _tmp137 = _tmp135 * _tmp62;
  const Scalar _tmp138 = _tmp54 * (-_tmp111 * _tmp136 - _tmp137 * _tmp52 + _tmp50);
  const Scalar _tmp139 = -Scalar(1.0) * _tmp137 - Scalar(1.0) * _tmp138 + Scalar(1.0);
  const Scalar _tmp140 = Scalar(1.0) * _tmp123 * (_tmp117 * _tmp63 - _tmp121 * _tmp122) +
                         Scalar(1.0) * _tmp127 * (-_tmp122 * _tmp126 + _tmp124) +
                         Scalar(1.0) * _tmp133 * (-_tmp122 * _tmp132 + _tmp130 * _tmp63) +
                         _tmp134 * _tmp139 + _tmp42 * _tmp65 + _tmp67 * _tmp70;
  const Scalar _tmp141 = _tmp108 * _tmp136;
  const Scalar _tmp142 = _tmp84 * (_tmp105 * _tmp136 - _tmp141 * _tmp76);
  const Scalar _tmp143 = _tmp105 * _tmp113;
  const Scalar _tmp144 = _tmp107 * _tmp113;
  const Scalar _tmp145 = -_tmp131 * _tmp144 + Scalar(1.0);
  const Scalar _tmp146 = _tmp76 * _tmp89;
  const Scalar _tmp147 = -_tmp118 * _tmp144 - _tmp88;
  const Scalar _tmp148 = -_tmp123 * _tmp84 * (_tmp118 * _tmp143 + _tmp146 * _tmp147 + Scalar(1.0)) -
                         _tmp127 * _tmp84 * (_tmp105 * _tmp125 - _tmp108 * _tmp125 * _tmp76) -
                         _tmp133 * _tmp84 * (_tmp131 * _tmp143 + _tmp145 * _tmp146) -
                         _tmp134 * _tmp142;
  const Scalar _tmp149 = Scalar(1.0) / (_tmp148);
  const Scalar _tmp150 = std::asinh(_tmp140 * _tmp149);
  const Scalar _tmp151 = Scalar(9.6622558468725703) * _tmp148;
  const Scalar _tmp152 =
      -_tmp150 * _tmp151 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp77 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp79 - 1), Scalar(2))));
  const Scalar _tmp153 = std::pow(_tmp148, Scalar(-2));
  const Scalar _tmp154 = _tmp142 * _tmp153;
  const Scalar _tmp155 = _tmp27 + _tmp47;
  const Scalar _tmp156 =
      (-_tmp140 * _tmp154 + _tmp149 * (-_tmp139 + _tmp155 * _tmp70 + _tmp20 * _tmp65)) /
      std::sqrt(Scalar(std::pow(_tmp140, Scalar(2)) * _tmp153 + 1));
  const Scalar _tmp157 = Scalar(9.6622558468725703) * _tmp142;
  const Scalar _tmp158 = Scalar(0.1034955) * _tmp149;
  const Scalar _tmp159 = _tmp152 * _tmp158;
  const Scalar _tmp160 = Scalar(1.0) * _tmp150;
  const Scalar _tmp161 = _tmp42 * _tmp63;
  const Scalar _tmp162 = _tmp121 * _tmp123 * _tmp54 + _tmp126 * _tmp127 * _tmp54 +
                         _tmp132 * _tmp133 * _tmp54 + _tmp134 * _tmp138 - _tmp161 * _tmp64 +
                         _tmp67 * _tmp69;
  const Scalar _tmp163 = _tmp125 * _tmp127;
  const Scalar _tmp164 = _tmp134 * _tmp136;
  const Scalar _tmp165 = -_tmp108 * _tmp163 - _tmp108 * _tmp164 + _tmp123 * _tmp147 * _tmp89 +
                         _tmp133 * _tmp145 * _tmp89;
  const Scalar _tmp166 = Scalar(1.0) / (_tmp165);
  const Scalar _tmp167 = std::asinh(_tmp162 * _tmp166);
  const Scalar _tmp168 = Scalar(1.0) * _tmp167;
  const Scalar _tmp169 = std::pow(_tmp165, Scalar(-2));
  const Scalar _tmp170 = _tmp20 * _tmp63;
  const Scalar _tmp171 = _tmp141 * _tmp169;
  const Scalar _tmp172 =
      (-_tmp162 * _tmp171 + _tmp166 * (-_tmp138 + _tmp155 * _tmp69 - _tmp170 * _tmp64)) /
      std::sqrt(Scalar(std::pow(_tmp162, Scalar(2)) * _tmp169 + 1));
  const Scalar _tmp173 = Scalar(9.6622558468725703) * _tmp165;
  const Scalar _tmp174 =
      -_tmp167 * _tmp173 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp73), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp71 - 1), Scalar(2))));
  const Scalar _tmp175 = Scalar(0.1034955) * _tmp166;
  const Scalar _tmp176 = _tmp174 * _tmp175;
  const Scalar _tmp177 = Scalar(9.6622558468725703) * _tmp136;
  const Scalar _tmp178 = _tmp108 * _tmp177;
  const Scalar _tmp179 =
      _tmp113 * _tmp118 * _tmp123 + _tmp113 * _tmp131 * _tmp133 + _tmp163 + _tmp164;
  const Scalar _tmp180 = Scalar(1.0) / (_tmp179);
  const Scalar _tmp181 = -_tmp117 * _tmp123 * _tmp62 - _tmp124 * _tmp127 -
                         _tmp130 * _tmp133 * _tmp62 + _tmp134 * _tmp137 + _tmp161 - _tmp67 * _tmp68;
  const Scalar _tmp182 = std::asinh(_tmp180 * _tmp181);
  const Scalar _tmp183 = Scalar(1.0) * _tmp182;
  const Scalar _tmp184 = Scalar(9.6622558468725703) * _tmp179;
  const Scalar _tmp185 =
      -_tmp182 * _tmp184 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp102), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp100 - 1), Scalar(2))));
  const Scalar _tmp186 = Scalar(0.1034955) * _tmp180;
  const Scalar _tmp187 = _tmp185 * _tmp186;
  const Scalar _tmp188 = std::pow(_tmp179, Scalar(-2));
  const Scalar _tmp189 = _tmp136 * _tmp188;
  const Scalar _tmp190 = (_tmp180 * (-_tmp137 - _tmp155 * _tmp68 + _tmp170) + _tmp181 * _tmp189) /
                         std::sqrt(Scalar(std::pow(_tmp181, Scalar(2)) * _tmp188 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp3 *
      (-_tmp2 * std::cosh(Scalar(1.0) * _tmp1) +
       _tmp2 *
           std::cosh(
               Scalar(0.1034955) * _tmp0 *
               (-_tmp1 * _tmp3 -
                Scalar(4.7752063900000001) *
                    std::sqrt(Scalar(
                        Scalar(0.32397683292140877) *
                            std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp21), Scalar(2)) +
                        std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp32), Scalar(2)))))));
  _res(1, 0) = _tmp151 * (-Scalar(1.0) * _tmp156 * std::cosh(_tmp160) -
                          (-Scalar(0.1034955) * _tmp152 * _tmp154 +
                           _tmp158 * (-_tmp150 * _tmp157 - _tmp151 * _tmp156)) *
                              std::cosh(_tmp159)) +
               _tmp157 * (-std::sinh(_tmp159) - std::sinh(_tmp160));
  _res(2, 0) = _tmp173 * (-Scalar(1.0) * _tmp172 * std::cosh(_tmp168) -
                          (-Scalar(0.1034955) * _tmp171 * _tmp174 +
                           _tmp175 * (-_tmp167 * _tmp178 - _tmp172 * _tmp173)) *
                              std::cosh(_tmp176)) +
               _tmp178 * (-std::sinh(_tmp168) - std::sinh(_tmp176));
  _res(3, 0) = -_tmp177 * (-std::sinh(_tmp183) - std::sinh(_tmp187)) +
               _tmp184 * (-Scalar(1.0) * _tmp190 * std::cosh(_tmp183) -
                          (Scalar(0.1034955) * _tmp185 * _tmp189 +
                           _tmp186 * (_tmp177 * _tmp182 - _tmp184 * _tmp190)) *
                              std::cosh(_tmp187));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym