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
 * Symbolic function: IK_residual_func_cost2_Nl4
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2Nl4(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 530

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (159)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp2 = -2 * std::pow(_tmp1, Scalar(2));
  const Scalar _tmp3 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp4 = 1 - 2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp5 = Scalar(0.20999999999999999) * _tmp2 + Scalar(0.20999999999999999) * _tmp4;
  const Scalar _tmp6 = -_tmp5;
  const Scalar _tmp7 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp8 = 2 * _tmp3 * _tmp7;
  const Scalar _tmp9 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                       2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp10 = _tmp1 * _tmp9;
  const Scalar _tmp11 = -_tmp10 + _tmp8;
  const Scalar _tmp12 = -Scalar(0.010999999999999999) * _tmp11;
  const Scalar _tmp13 = 2 * _tmp1;
  const Scalar _tmp14 = _tmp13 * _tmp7;
  const Scalar _tmp15 = _tmp3 * _tmp9;
  const Scalar _tmp16 = Scalar(0.20999999999999999) * _tmp14 + Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp17 = _tmp12 - _tmp16;
  const Scalar _tmp18 = _tmp17 + _tmp6;
  const Scalar _tmp19 = _tmp18 + position_vector(1, 0);
  const Scalar _tmp20 = -2 * std::pow(_tmp7, Scalar(2));
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp20 + Scalar(0.20999999999999999) * _tmp4;
  const Scalar _tmp22 = -_tmp21;
  const Scalar _tmp23 = _tmp13 * _tmp3;
  const Scalar _tmp24 = _tmp7 * _tmp9;
  const Scalar _tmp25 = _tmp23 + _tmp24;
  const Scalar _tmp26 = -Scalar(0.010999999999999999) * _tmp25;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp14 - Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp28 = _tmp26 - _tmp27;
  const Scalar _tmp29 = _tmp22 + _tmp28;
  const Scalar _tmp30 = _tmp29 + position_vector(0, 0);
  const Scalar _tmp31 = Scalar(69.216682114881593) *
                            std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp19 - 1), Scalar(2)) +
                        Scalar(3.9500536956656402) *
                            std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp30 - 1), Scalar(2));
  const Scalar _tmp32 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp33 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp34 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp35 = -_tmp34;
  const Scalar _tmp36 = -Scalar(0.010999999999999999) * _tmp2 -
                        Scalar(0.010999999999999999) * _tmp20 + Scalar(-0.010999999999999999);
  const Scalar _tmp37 = Scalar(0.20999999999999999) * _tmp23 - Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp38 = _tmp36 - _tmp37;
  const Scalar _tmp39 = _tmp35 + _tmp38;
  const Scalar _tmp40 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp41 = _tmp34 + _tmp38;
  const Scalar _tmp42 = _tmp12 + _tmp16;
  const Scalar _tmp43 = _tmp42 + _tmp5;
  const Scalar _tmp44 = _tmp43 + position_vector(1, 0);
  const Scalar _tmp45 = _tmp44 + Scalar(-4.7752063900000001);
  const Scalar _tmp46 = _tmp26 + _tmp27;
  const Scalar _tmp47 = _tmp21 + _tmp46;
  const Scalar _tmp48 = _tmp47 + position_vector(0, 0);
  const Scalar _tmp49 = _tmp48 + Scalar(-2.71799795);
  const Scalar _tmp50 = std::pow(Scalar(std::pow(_tmp45, Scalar(2)) + std::pow(_tmp49, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp51 = _tmp49 * _tmp50;
  const Scalar _tmp52 = _tmp45 * _tmp50;
  const Scalar _tmp53 = _tmp17 + _tmp5;
  const Scalar _tmp54 = _tmp53 + position_vector(1, 0);
  const Scalar _tmp55 = _tmp54 + Scalar(-4.8333311099999996);
  const Scalar _tmp56 = _tmp22 + _tmp46;
  const Scalar _tmp57 = _tmp56 + position_vector(0, 0);
  const Scalar _tmp58 = _tmp57 + Scalar(1.79662371);
  const Scalar _tmp59 = Scalar(1.0) / (_tmp58);
  const Scalar _tmp60 = _tmp55 * _tmp59;
  const Scalar _tmp61 = _tmp51 * _tmp60 - _tmp52;
  const Scalar _tmp62 = _tmp42 + _tmp6;
  const Scalar _tmp63 = _tmp62 + position_vector(1, 0);
  const Scalar _tmp64 = _tmp63 + Scalar(8.3888750099999996);
  const Scalar _tmp65 = _tmp21 + _tmp28;
  const Scalar _tmp66 = _tmp65 + position_vector(0, 0);
  const Scalar _tmp67 = _tmp66 + Scalar(-2.5202214700000001);
  const Scalar _tmp68 = std::pow(Scalar(std::pow(_tmp64, Scalar(2)) + std::pow(_tmp67, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp69 = _tmp64 * _tmp68;
  const Scalar _tmp70 = _tmp67 * _tmp68;
  const Scalar _tmp71 = Scalar(1.0) / (_tmp60 * _tmp70 - _tmp69);
  const Scalar _tmp72 = _tmp41 * _tmp70;
  const Scalar _tmp73 = _tmp36 + _tmp37;
  const Scalar _tmp74 = _tmp35 + _tmp73;
  const Scalar _tmp75 = _tmp71 * (-_tmp60 * _tmp72 + _tmp69 * _tmp74);
  const Scalar _tmp76 = _tmp41 * _tmp60;
  const Scalar _tmp77 = _tmp34 + _tmp73;
  const Scalar _tmp78 = -_tmp51 * _tmp76 + _tmp52 * _tmp77 - _tmp61 * _tmp75;
  const Scalar _tmp79 = Scalar(1.0) * _tmp53;
  const Scalar _tmp80 = -_tmp79;
  const Scalar _tmp81 = Scalar(1.0) / (_tmp62 + _tmp80);
  const Scalar _tmp82 = Scalar(1.0) * _tmp56;
  const Scalar _tmp83 = _tmp81 * (-_tmp65 + _tmp82);
  const Scalar _tmp84 = _tmp71 * (-_tmp70 * _tmp74 + _tmp72);
  const Scalar _tmp85 = _tmp41 * _tmp51 - _tmp51 * _tmp77 - _tmp61 * _tmp84 - _tmp78 * _tmp83;
  const Scalar _tmp86 = Scalar(1.0) / (_tmp85);
  const Scalar _tmp87 = _tmp79 * _tmp83 + _tmp82;
  const Scalar _tmp88 = 0;
  const Scalar _tmp89 = _tmp70 * _tmp71;
  const Scalar _tmp90 = _tmp61 * _tmp89;
  const Scalar _tmp91 =
      std::sqrt(Scalar(std::pow(_tmp55, Scalar(2)) + std::pow(_tmp58, Scalar(2))));
  const Scalar _tmp92 = _tmp59 * _tmp91;
  const Scalar _tmp93 = Scalar(1.0) * _tmp75;
  const Scalar _tmp94 = _tmp83 * _tmp93 - Scalar(1.0) * _tmp84;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp91);
  const Scalar _tmp96 = _tmp92 * (-_tmp53 * _tmp58 * _tmp95 + _tmp55 * _tmp56 * _tmp95);
  const Scalar _tmp97 = _tmp71 * (_tmp62 * _tmp70 - _tmp65 * _tmp69 + _tmp70 * _tmp96);
  const Scalar _tmp98 = _tmp43 * _tmp51 - _tmp47 * _tmp52 + _tmp51 * _tmp96 - _tmp61 * _tmp97;
  const Scalar _tmp99 = _tmp86 * _tmp98;
  const Scalar _tmp100 = Scalar(1.0) / (_tmp98);
  const Scalar _tmp101 = _tmp100 * _tmp85;
  const Scalar _tmp102 = _tmp101 * (-_tmp94 * _tmp99 - Scalar(1.0) * _tmp97);
  const Scalar _tmp103 = _tmp102 + _tmp94;
  const Scalar _tmp104 = _tmp61 * _tmp86;
  const Scalar _tmp105 = -_tmp103 * _tmp104 + Scalar(1.0);
  const Scalar _tmp106 = _tmp51 * _tmp86;
  const Scalar _tmp107 = _tmp30 + Scalar(1.9874742000000001);
  const Scalar _tmp108 = _tmp19 + Scalar(8.3196563700000006);
  const Scalar _tmp109 =
      std::pow(Scalar(std::pow(_tmp107, Scalar(2)) + std::pow(_tmp108, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp110 = _tmp108 * _tmp109;
  const Scalar _tmp111 = _tmp110 * fh1;
  const Scalar _tmp112 = Scalar(1.0) * _tmp100;
  const Scalar _tmp113 = _tmp107 * _tmp109;
  const Scalar _tmp114 = fh1 * (_tmp110 * _tmp29 - _tmp113 * _tmp18);
  const Scalar _tmp115 = _tmp60 * _tmp75 + _tmp76;
  const Scalar _tmp116 = -_tmp115 * _tmp83 - _tmp41 + _tmp60 * _tmp84;
  const Scalar _tmp117 = _tmp101 * (-_tmp116 * _tmp99 + _tmp60 * _tmp97 - _tmp96);
  const Scalar _tmp118 = _tmp116 + _tmp117;
  const Scalar _tmp119 = -_tmp104 * _tmp118 - _tmp60;
  const Scalar _tmp120 = _tmp113 * fh1;
  const Scalar _tmp121 = -_tmp111 * _tmp92 * (_tmp103 * _tmp106 + _tmp105 * _tmp89) -
                         _tmp114 * _tmp92 * (_tmp112 * _tmp51 - _tmp112 * _tmp90) -
                         _tmp120 * _tmp92 * (_tmp106 * _tmp118 + _tmp119 * _tmp89 + Scalar(1.0)) -
                         _tmp40 * _tmp92 * (_tmp51 * _tmp88 - _tmp88 * _tmp90);
  const Scalar _tmp122 = Scalar(1.0) / (_tmp121);
  const Scalar _tmp123 = _tmp43 + _tmp80;
  const Scalar _tmp124 = _tmp123 * _tmp83;
  const Scalar _tmp125 = Scalar(1.0) / (-_tmp124 - _tmp47 + _tmp82);
  const Scalar _tmp126 = Scalar(1.0) * _tmp125;
  const Scalar _tmp127 = _tmp123 * _tmp125;
  const Scalar _tmp128 = _tmp78 * _tmp86;
  const Scalar _tmp129 = _tmp102 * _tmp127 - _tmp103 * _tmp128 - _tmp93;
  const Scalar _tmp130 = Scalar(1.0) * _tmp81;
  const Scalar _tmp131 = _tmp81 * (_tmp124 * _tmp126 + Scalar(1.0));
  const Scalar _tmp132 = _tmp126 * _tmp83;
  const Scalar _tmp133 = _tmp39 * fh1;
  const Scalar _tmp134 = -Scalar(5.1796800000000003) * _tmp11 - _tmp110 * _tmp133 - _tmp18 * fv1;
  const Scalar _tmp135 = _tmp113 * _tmp133 + Scalar(5.1796800000000003) * _tmp25 + _tmp29 * fv1;
  const Scalar _tmp136 = _tmp123 * _tmp81;
  const Scalar _tmp137 = _tmp125 * _tmp87;
  const Scalar _tmp138 = -_tmp123 * _tmp137 - _tmp78 * _tmp88 + _tmp80;
  const Scalar _tmp139 = _tmp115 + _tmp117 * _tmp127 - _tmp118 * _tmp128;
  const Scalar _tmp140 = _tmp101 * _tmp126;
  const Scalar _tmp141 = -_tmp112 * _tmp78 + _tmp123 * _tmp140;
  const Scalar _tmp142 = std::asinh(
      _tmp122 * (Scalar(1.0) * _tmp111 * (_tmp102 * _tmp126 - _tmp129 * _tmp130) +
                 Scalar(1.0) * _tmp114 * (-_tmp130 * _tmp141 + _tmp140) +
                 Scalar(1.0) * _tmp120 * (_tmp117 * _tmp126 - _tmp130 * _tmp139) +
                 Scalar(1.0) * _tmp134 * (-Scalar(1.0) * _tmp131 + _tmp132) +
                 Scalar(1.0) * _tmp135 * (_tmp126 * _tmp136 - _tmp126) +
                 Scalar(1.0) * _tmp40 * (-_tmp126 * _tmp87 - _tmp130 * _tmp138 + Scalar(1.0))));
  const Scalar _tmp143 = Scalar(9.6622558468725703) * _tmp121;
  const Scalar _tmp144 = Scalar(23.361089618893828) *
                             std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp54), Scalar(2)) +
                         Scalar(3.2278567553341642) *
                             std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp57 - 1), Scalar(2));
  const Scalar _tmp145 = Scalar(6.351516257848961) *
                             std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp66), Scalar(2)) +
                         Scalar(70.3732239334025) *
                             std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp63 - 1), Scalar(2));
  const Scalar _tmp146 = _tmp126 * _tmp135;
  const Scalar _tmp147 = _tmp112 * _tmp114;
  const Scalar _tmp148 = _tmp61 * _tmp71;
  const Scalar _tmp149 = _tmp40 * _tmp88;
  const Scalar _tmp150 = _tmp105 * _tmp111 * _tmp71 + _tmp119 * _tmp120 * _tmp71 -
                         _tmp147 * _tmp148 - _tmp148 * _tmp149;
  const Scalar _tmp151 = Scalar(1.0) / (_tmp150);
  const Scalar _tmp152 =
      std::asinh(_tmp151 * (_tmp111 * _tmp129 * _tmp81 + _tmp114 * _tmp141 * _tmp81 +
                            _tmp120 * _tmp139 * _tmp81 + _tmp131 * _tmp134 - _tmp136 * _tmp146 +
                            _tmp138 * _tmp40 * _tmp81));
  const Scalar _tmp153 = Scalar(9.6622558468725703) * _tmp150;
  const Scalar _tmp154 =
      _tmp103 * _tmp111 * _tmp86 + _tmp118 * _tmp120 * _tmp86 + _tmp147 + _tmp149;
  const Scalar _tmp155 = Scalar(1.0) / (_tmp154);
  const Scalar _tmp156 = std::asinh(_tmp155 * (-_tmp102 * _tmp111 * _tmp125 - _tmp114 * _tmp140 -
                                               _tmp117 * _tmp120 * _tmp125 - _tmp132 * _tmp134 +
                                               _tmp137 * _tmp40 + _tmp146));
  const Scalar _tmp157 = Scalar(9.6622558468725703) * _tmp154;
  const Scalar _tmp158 = Scalar(22.802596067096832) *
                             std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp44), Scalar(2)) +
                         Scalar(7.3875128562042027) *
                             std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp48), Scalar(2));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp33 * (-std::sinh(Scalar(1.0) * _tmp32) -
                std::sinh(Scalar(0.1034955) * _tmp0 * (-std::sqrt(_tmp31) - _tmp32 * _tmp33))) -
      Scalar(8.4718465799999993) *
          std::sqrt(Scalar(Scalar(0.013932974275675287) * _tmp31 +
                           std::pow(Scalar(-Scalar(0.11803802046660766) * _tmp39 -
                                           Scalar(0.11803802046660766) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(1, 0) =
      _tmp143 *
          (-std::sinh(Scalar(1.0) * _tmp142) -
           std::sinh(Scalar(0.1034955) * _tmp122 * (-_tmp142 * _tmp143 - std::sqrt(_tmp144)))) -
      Scalar(8.3700199099999999) *
          std::sqrt(Scalar(Scalar(0.01427404356387209) * _tmp144 +
                           std::pow(Scalar(-Scalar(0.11947402882581673) * _tmp41 -
                                           Scalar(0.11947402882581673) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(2, 0) =
      _tmp153 *
          (-std::sinh(Scalar(1.0) * _tmp152) -
           std::sinh(Scalar(0.1034955) * _tmp151 * (-std::sqrt(_tmp145) - _tmp152 * _tmp153))) -
      Scalar(8.4693136199999994) *
          std::sqrt(Scalar(Scalar(0.013941309530580858) * _tmp145 +
                           std::pow(Scalar(-Scalar(0.11807332268798426) * _tmp74 -
                                           Scalar(0.11807332268798426) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(3, 0) =
      _tmp157 *
          (-std::sinh(Scalar(1.0) * _tmp156) -
           std::sinh(Scalar(0.1034955) * _tmp155 * (-_tmp156 * _tmp157 - std::sqrt(_tmp158)))) -
      Scalar(8.36416322) *
          std::sqrt(Scalar(Scalar(0.014294040284261563) * _tmp158 +
                           std::pow(Scalar(-Scalar(0.1195576860108189) * _tmp77 -
                                           Scalar(0.1195576860108189) * position_vector(2, 0) + 1),
                                    Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
