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
 * Symbolic function: IK_residual_func_cost2_Nl17
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2Nl17(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 531

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (163)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp3 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp4 = -2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp5 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp6 = 1 - 2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp7 = Scalar(0.20999999999999999) * _tmp4 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp8 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp9 = 2 * _tmp8;
  const Scalar _tmp10 = _tmp5 * _tmp9;
  const Scalar _tmp11 = -_DeltaRot[0] * _Rot_init[0] - _DeltaRot[1] * _Rot_init[1] -
                        _DeltaRot[2] * _Rot_init[2] + _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp12 = 2 * _tmp11;
  const Scalar _tmp13 = _tmp12 * _tmp3;
  const Scalar _tmp14 = _tmp10 + _tmp13;
  const Scalar _tmp15 = -Scalar(0.010999999999999999) * _tmp14;
  const Scalar _tmp16 = _tmp3 * _tmp9;
  const Scalar _tmp17 = _tmp12 * _tmp5;
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp16 - Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp19 = _tmp15 + _tmp18;
  const Scalar _tmp20 = _tmp19 + _tmp7;
  const Scalar _tmp21 = _tmp20 + position_vector(0, 0);
  const Scalar _tmp22 = -2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp23 = Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp24 = 2 * _tmp3 * _tmp5;
  const Scalar _tmp25 = _tmp11 * _tmp9;
  const Scalar _tmp26 = _tmp24 - _tmp25;
  const Scalar _tmp27 = -Scalar(0.010999999999999999) * _tmp26;
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp16 + Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp29 = _tmp27 + _tmp28;
  const Scalar _tmp30 = _tmp23 + _tmp29;
  const Scalar _tmp31 = _tmp30 + position_vector(1, 0);
  const Scalar _tmp32 = Scalar(7.3875128562042027) *
                            std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp21), Scalar(2)) +
                        Scalar(22.802596067096832) *
                            std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp31), Scalar(2));
  const Scalar _tmp33 = Scalar(0.20999999999999999) * _tmp10 - Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp34 = -Scalar(0.010999999999999999) * _tmp22 -
                        Scalar(0.010999999999999999) * _tmp4 + Scalar(-0.010999999999999999);
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp36 = _tmp34 + _tmp35;
  const Scalar _tmp37 = _tmp33 + _tmp36;
  const Scalar _tmp38 = -_tmp33;
  const Scalar _tmp39 = _tmp36 + _tmp38;
  const Scalar _tmp40 = _tmp27 - _tmp28;
  const Scalar _tmp41 = _tmp23 + _tmp40;
  const Scalar _tmp42 = _tmp41 + position_vector(1, 0);
  const Scalar _tmp43 = _tmp42 + Scalar(-4.8333311099999996);
  const Scalar _tmp44 = -_tmp7;
  const Scalar _tmp45 = _tmp19 + _tmp44;
  const Scalar _tmp46 = _tmp45 + position_vector(0, 0);
  const Scalar _tmp47 = _tmp46 + Scalar(1.79662371);
  const Scalar _tmp48 = Scalar(1.0) / (_tmp47);
  const Scalar _tmp49 = _tmp43 * _tmp48;
  const Scalar _tmp50 = -_tmp23;
  const Scalar _tmp51 = _tmp29 + _tmp50;
  const Scalar _tmp52 = _tmp51 + position_vector(1, 0);
  const Scalar _tmp53 = _tmp52 + Scalar(8.3888750099999996);
  const Scalar _tmp54 = _tmp15 - _tmp18;
  const Scalar _tmp55 = _tmp54 + _tmp7;
  const Scalar _tmp56 = _tmp55 + position_vector(0, 0);
  const Scalar _tmp57 = _tmp56 + Scalar(-2.5202214700000001);
  const Scalar _tmp58 = std::pow(Scalar(std::pow(_tmp53, Scalar(2)) + std::pow(_tmp57, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp59 = _tmp57 * _tmp58;
  const Scalar _tmp60 = _tmp39 * _tmp59;
  const Scalar _tmp61 = _tmp34 - _tmp35;
  const Scalar _tmp62 = _tmp33 + _tmp61;
  const Scalar _tmp63 = _tmp53 * _tmp58;
  const Scalar _tmp64 = -_tmp49 * _tmp60 + _tmp62 * _tmp63;
  const Scalar _tmp65 = Scalar(1.0) / (_tmp49 * _tmp59 - _tmp63);
  const Scalar _tmp66 = _tmp49 * _tmp65;
  const Scalar _tmp67 = _tmp39 * _tmp49 + _tmp64 * _tmp66;
  const Scalar _tmp68 = Scalar(1.0) * _tmp41;
  const Scalar _tmp69 = -_tmp68;
  const Scalar _tmp70 = Scalar(1.0) / (_tmp51 + _tmp69);
  const Scalar _tmp71 = Scalar(1.0) * _tmp45;
  const Scalar _tmp72 = -_tmp55 + _tmp71;
  const Scalar _tmp73 = _tmp70 * _tmp72;
  const Scalar _tmp74 = -_tmp59 * _tmp62 + _tmp60;
  const Scalar _tmp75 = -_tmp39 + _tmp66 * _tmp74 - _tmp67 * _tmp73;
  const Scalar _tmp76 = _tmp38 + _tmp61;
  const Scalar _tmp77 = _tmp40 + _tmp50;
  const Scalar _tmp78 = _tmp77 + position_vector(1, 0);
  const Scalar _tmp79 = _tmp78 + Scalar(8.3196563700000006);
  const Scalar _tmp80 = _tmp44 + _tmp54;
  const Scalar _tmp81 = _tmp80 + position_vector(0, 0);
  const Scalar _tmp82 = _tmp81 + Scalar(1.9874742000000001);
  const Scalar _tmp83 = std::pow(Scalar(std::pow(_tmp79, Scalar(2)) + std::pow(_tmp82, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp84 = _tmp79 * _tmp83;
  const Scalar _tmp85 = _tmp82 * _tmp83;
  const Scalar _tmp86 = _tmp39 * _tmp85;
  const Scalar _tmp87 = _tmp49 * _tmp85 - _tmp84;
  const Scalar _tmp88 = _tmp65 * _tmp87;
  const Scalar _tmp89 = -_tmp49 * _tmp86 - _tmp64 * _tmp88 + _tmp76 * _tmp84;
  const Scalar _tmp90 = -_tmp73 * _tmp89 - _tmp74 * _tmp88 - _tmp76 * _tmp85 + _tmp86;
  const Scalar _tmp91 = Scalar(1.0) / (_tmp90);
  const Scalar _tmp92 =
      std::sqrt(Scalar(std::pow(_tmp43, Scalar(2)) + std::pow(_tmp47, Scalar(2))));
  const Scalar _tmp93 = Scalar(1.0) / (_tmp92);
  const Scalar _tmp94 = _tmp48 * _tmp92;
  const Scalar _tmp95 = _tmp94 * (-_tmp41 * _tmp47 * _tmp93 + _tmp43 * _tmp45 * _tmp93);
  const Scalar _tmp96 = _tmp51 * _tmp59 - _tmp55 * _tmp63 + _tmp59 * _tmp95;
  const Scalar _tmp97 = _tmp77 * _tmp85 - _tmp80 * _tmp84 + _tmp85 * _tmp95 - _tmp88 * _tmp96;
  const Scalar _tmp98 = _tmp91 * _tmp97;
  const Scalar _tmp99 = Scalar(1.0) / (_tmp97);
  const Scalar _tmp100 = _tmp90 * _tmp99;
  const Scalar _tmp101 = _tmp100 * (_tmp66 * _tmp96 - _tmp75 * _tmp98 - _tmp95);
  const Scalar _tmp102 = _tmp101 + _tmp75;
  const Scalar _tmp103 = _tmp87 * _tmp91;
  const Scalar _tmp104 = -_tmp102 * _tmp103 - _tmp49;
  const Scalar _tmp105 = _tmp59 * _tmp65;
  const Scalar _tmp106 = _tmp85 * _tmp91;
  const Scalar _tmp107 = _tmp21 + Scalar(-2.71799795);
  const Scalar _tmp108 = _tmp31 + Scalar(-4.7752063900000001);
  const Scalar _tmp109 =
      std::pow(Scalar(std::pow(_tmp107, Scalar(2)) + std::pow(_tmp108, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp110 = _tmp107 * _tmp109;
  const Scalar _tmp111 = _tmp110 * fh1;
  const Scalar _tmp112 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp113 = _tmp68 * _tmp73 + _tmp71;
  const Scalar _tmp114 = 0;
  const Scalar _tmp115 = _tmp114 * _tmp91;
  const Scalar _tmp116 = Scalar(1.0) * _tmp65;
  const Scalar _tmp117 = _tmp116 * _tmp64;
  const Scalar _tmp118 = -_tmp116 * _tmp74 + _tmp117 * _tmp73;
  const Scalar _tmp119 = _tmp100 * (-_tmp116 * _tmp96 - _tmp118 * _tmp98);
  const Scalar _tmp120 = _tmp118 + _tmp119;
  const Scalar _tmp121 = -_tmp103 * _tmp120 + Scalar(1.0);
  const Scalar _tmp122 = _tmp108 * _tmp109;
  const Scalar _tmp123 = _tmp122 * fh1;
  const Scalar _tmp124 = Scalar(1.0) * _tmp99;
  const Scalar _tmp125 = _tmp116 * _tmp87 * _tmp99;
  const Scalar _tmp126 = fh1 * (-_tmp110 * _tmp30 + _tmp122 * _tmp20);
  const Scalar _tmp127 = -_tmp111 * _tmp94 * (_tmp102 * _tmp106 + _tmp104 * _tmp105 + Scalar(1.0)) -
                         _tmp112 * _tmp94 * (-_tmp115 * _tmp59 * _tmp88 + _tmp115 * _tmp85) -
                         _tmp123 * _tmp94 * (_tmp105 * _tmp121 + _tmp106 * _tmp120) -
                         _tmp126 * _tmp94 * (_tmp124 * _tmp85 - _tmp125 * _tmp59);
  const Scalar _tmp128 = Scalar(1.0) / (_tmp127);
  const Scalar _tmp129 = _tmp89 * _tmp91;
  const Scalar _tmp130 = _tmp69 + _tmp77;
  const Scalar _tmp131 = _tmp130 * _tmp73;
  const Scalar _tmp132 = Scalar(1.0) / (-_tmp131 + _tmp71 - _tmp80);
  const Scalar _tmp133 = _tmp130 * _tmp132;
  const Scalar _tmp134 = -_tmp117 + _tmp119 * _tmp133 - _tmp120 * _tmp129;
  const Scalar _tmp135 = Scalar(1.0) * _tmp70;
  const Scalar _tmp136 = Scalar(1.0) * _tmp132;
  const Scalar _tmp137 = _tmp37 * fh1;
  const Scalar _tmp138 = -_tmp122 * _tmp137 - Scalar(5.1796800000000003) * _tmp26 - _tmp30 * fv1;
  const Scalar _tmp139 = _tmp131 * _tmp136 + Scalar(1.0);
  const Scalar _tmp140 = _tmp113 * _tmp132;
  const Scalar _tmp141 = -_tmp114 * _tmp129 - _tmp130 * _tmp140 + _tmp69;
  const Scalar _tmp142 = _tmp101 * _tmp133 - _tmp102 * _tmp129 + _tmp67;
  const Scalar _tmp143 = _tmp100 * _tmp136;
  const Scalar _tmp144 = _tmp130 * _tmp136;
  const Scalar _tmp145 = _tmp100 * _tmp144 - _tmp124 * _tmp89;
  const Scalar _tmp146 = _tmp110 * _tmp137 + Scalar(5.1796800000000003) * _tmp14 + _tmp20 * fv1;
  const Scalar _tmp147 = std::asinh(
      _tmp128 * (Scalar(1.0) * _tmp111 * (_tmp101 * _tmp136 - _tmp135 * _tmp142) +
                 Scalar(1.0) * _tmp112 * (-_tmp113 * _tmp136 - _tmp135 * _tmp141 + Scalar(1.0)) +
                 Scalar(1.0) * _tmp123 * (_tmp119 * _tmp136 - _tmp134 * _tmp135) +
                 Scalar(1.0) * _tmp126 * (-_tmp135 * _tmp145 + _tmp143) +
                 Scalar(1.0) * _tmp138 * (-_tmp135 * _tmp139 + _tmp136 * _tmp73) +
                 Scalar(1.0) * _tmp146 * (-_tmp136 + _tmp144 * _tmp70)));
  const Scalar _tmp148 = Scalar(23.361089618893828) *
                             std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp42), Scalar(2)) +
                         Scalar(3.2278567553341642) *
                             std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp46 - 1), Scalar(2));
  const Scalar _tmp149 = Scalar(9.6622558468725703) * _tmp127;
  const Scalar _tmp150 = _tmp138 * _tmp70;
  const Scalar _tmp151 = _tmp136 * _tmp146;
  const Scalar _tmp152 = _tmp112 * _tmp115;
  const Scalar _tmp153 = _tmp104 * _tmp111 * _tmp65 + _tmp121 * _tmp123 * _tmp65 -
                         _tmp125 * _tmp126 - _tmp152 * _tmp88;
  const Scalar _tmp154 = Scalar(1.0) / (_tmp153);
  const Scalar _tmp155 =
      std::asinh(_tmp154 * (_tmp111 * _tmp142 * _tmp70 + _tmp112 * _tmp141 * _tmp70 +
                            _tmp123 * _tmp134 * _tmp70 + _tmp126 * _tmp145 * _tmp70 -
                            _tmp130 * _tmp151 * _tmp70 + _tmp139 * _tmp150));
  const Scalar _tmp156 = Scalar(9.6622558468725703) * _tmp153;
  const Scalar _tmp157 = Scalar(6.351516257848961) *
                             std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp56), Scalar(2)) +
                         Scalar(70.3732239334025) *
                             std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp52 - 1), Scalar(2));
  const Scalar _tmp158 =
      _tmp102 * _tmp111 * _tmp91 + _tmp120 * _tmp123 * _tmp91 + _tmp124 * _tmp126 + _tmp152;
  const Scalar _tmp159 = Scalar(1.0) / (_tmp158);
  const Scalar _tmp160 = std::asinh(_tmp159 * (-_tmp101 * _tmp111 * _tmp132 + _tmp112 * _tmp140 -
                                               _tmp119 * _tmp123 * _tmp132 - _tmp126 * _tmp143 -
                                               _tmp136 * _tmp150 * _tmp72 + _tmp151));
  const Scalar _tmp161 = Scalar(9.6622558468725703) * _tmp158;
  const Scalar _tmp162 =
      Scalar(69.216682114881593) *
          std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp78 - 1), Scalar(2)) +
      Scalar(3.9500536956656402) *
          std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp81 - 1), Scalar(2));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp2 * (-std::sinh(Scalar(1.0) * _tmp1) -
               std::sinh(Scalar(0.1034955) * _tmp0 * (-_tmp1 * _tmp2 - std::sqrt(_tmp32)))) -
      Scalar(8.36416322) *
          std::sqrt(Scalar(Scalar(0.014294040284261563) * _tmp32 +
                           std::pow(Scalar(-Scalar(0.1195576860108189) * _tmp37 -
                                           Scalar(0.1195576860108189) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(1, 0) =
      _tmp149 *
          (-std::sinh(Scalar(1.0) * _tmp147) -
           std::sinh(Scalar(0.1034955) * _tmp128 * (-_tmp147 * _tmp149 - std::sqrt(_tmp148)))) -
      Scalar(8.3700199099999999) *
          std::sqrt(Scalar(Scalar(0.01427404356387209) * _tmp148 +
                           std::pow(Scalar(-Scalar(0.11947402882581673) * _tmp39 -
                                           Scalar(0.11947402882581673) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(2, 0) =
      _tmp156 *
          (-std::sinh(Scalar(1.0) * _tmp155) -
           std::sinh(Scalar(0.1034955) * _tmp154 * (-_tmp155 * _tmp156 - std::sqrt(_tmp157)))) -
      Scalar(8.4693136199999994) *
          std::sqrt(Scalar(Scalar(0.013941309530580858) * _tmp157 +
                           std::pow(Scalar(-Scalar(0.11807332268798426) * _tmp62 -
                                           Scalar(0.11807332268798426) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(3, 0) =
      _tmp161 *
          (-std::sinh(Scalar(1.0) * _tmp160) -
           std::sinh(Scalar(0.1034955) * _tmp159 * (-_tmp160 * _tmp161 - std::sqrt(_tmp162)))) -
      Scalar(8.4718465799999993) *
          std::sqrt(Scalar(Scalar(0.013932974275675287) * _tmp162 +
                           std::pow(Scalar(-Scalar(0.11803802046660766) * _tmp76 -
                                           Scalar(0.11803802046660766) * position_vector(2, 0) + 1),
                                    Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
