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
 * Symbolic function: IK_residual_func_cost2_Nl8
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2Nl8(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 529

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (158)
  const Scalar _tmp0 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp1 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp2 = 2 * _tmp1;
  const Scalar _tmp3 = _tmp0 * _tmp2;
  const Scalar _tmp4 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp5 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                       2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp6 = _tmp4 * _tmp5;
  const Scalar _tmp7 = Scalar(0.20999999999999999) * _tmp3 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp8 = -_tmp7;
  const Scalar _tmp9 = -2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp10 = 1 - 2 * std::pow(_tmp0, Scalar(2));
  const Scalar _tmp11 =
      -Scalar(0.010999999999999999) * _tmp10 - Scalar(0.010999999999999999) * _tmp9;
  const Scalar _tmp12 = _tmp2 * _tmp4;
  const Scalar _tmp13 = _tmp0 * _tmp5;
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp12 - Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 = _tmp11 + _tmp14;
  const Scalar _tmp16 = _tmp15 + _tmp8;
  const Scalar _tmp17 = -2 * std::pow(_tmp1, Scalar(2));
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp9 +
                        Scalar(0.20999999999999999);
  const Scalar _tmp19 = -_tmp18;
  const Scalar _tmp20 = _tmp3 - _tmp6;
  const Scalar _tmp21 = -Scalar(0.010999999999999999) * _tmp20;
  const Scalar _tmp22 = 2 * _tmp0 * _tmp4;
  const Scalar _tmp23 = _tmp1 * _tmp5;
  const Scalar _tmp24 = Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp25 = _tmp21 + _tmp24;
  const Scalar _tmp26 = _tmp19 + _tmp25;
  const Scalar _tmp27 = _tmp26 + position_vector(1, 0);
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp29 = _tmp12 + _tmp13;
  const Scalar _tmp30 = -Scalar(0.010999999999999999) * _tmp29;
  const Scalar _tmp31 = Scalar(0.20999999999999999) * _tmp22 - Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp32 = _tmp30 - _tmp31;
  const Scalar _tmp33 = _tmp28 + _tmp32;
  const Scalar _tmp34 = _tmp33 + position_vector(0, 0);
  const Scalar _tmp35 = Scalar(6.351516257848961) *
                            std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp34), Scalar(2)) +
                        Scalar(70.3732239334025) *
                            std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp27 - 1), Scalar(2));
  const Scalar _tmp36 = Scalar(1.0) / (fh1);
  const Scalar _tmp37 = std::asinh(_tmp36 * fv1);
  const Scalar _tmp38 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp39 = -_tmp28;
  const Scalar _tmp40 = _tmp30 + _tmp31;
  const Scalar _tmp41 = _tmp39 + _tmp40;
  const Scalar _tmp42 = _tmp41 + position_vector(0, 0);
  const Scalar _tmp43 = _tmp42 + Scalar(1.79662371);
  const Scalar _tmp44 = _tmp21 - _tmp24;
  const Scalar _tmp45 = _tmp18 + _tmp44;
  const Scalar _tmp46 = _tmp45 + position_vector(1, 0);
  const Scalar _tmp47 = _tmp46 + Scalar(-4.8333311099999996);
  const Scalar _tmp48 = std::pow(Scalar(std::pow(_tmp43, Scalar(2)) + std::pow(_tmp47, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp49 = _tmp43 * _tmp48;
  const Scalar _tmp50 = _tmp11 - _tmp14;
  const Scalar _tmp51 = _tmp50 + _tmp8;
  const Scalar _tmp52 = _tmp32 + _tmp39;
  const Scalar _tmp53 = _tmp52 + position_vector(0, 0);
  const Scalar _tmp54 = _tmp53 + Scalar(1.9874742000000001);
  const Scalar _tmp55 = _tmp19 + _tmp44;
  const Scalar _tmp56 = _tmp55 + position_vector(1, 0);
  const Scalar _tmp57 = _tmp56 + Scalar(8.3196563700000006);
  const Scalar _tmp58 = std::pow(Scalar(std::pow(_tmp54, Scalar(2)) + std::pow(_tmp57, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp59 = _tmp54 * _tmp58;
  const Scalar _tmp60 = _tmp15 + _tmp7;
  const Scalar _tmp61 = _tmp59 * _tmp60;
  const Scalar _tmp62 = -_tmp51 * _tmp59 + _tmp61;
  const Scalar _tmp63 = _tmp47 * _tmp48;
  const Scalar _tmp64 = _tmp18 + _tmp25;
  const Scalar _tmp65 = _tmp64 + position_vector(1, 0);
  const Scalar _tmp66 = _tmp65 + Scalar(-4.7752063900000001);
  const Scalar _tmp67 = _tmp28 + _tmp40;
  const Scalar _tmp68 = _tmp67 + position_vector(0, 0);
  const Scalar _tmp69 = _tmp68 + Scalar(-2.71799795);
  const Scalar _tmp70 = Scalar(1.0) / (_tmp69);
  const Scalar _tmp71 = _tmp66 * _tmp70;
  const Scalar _tmp72 = _tmp49 * _tmp71 - _tmp63;
  const Scalar _tmp73 = _tmp57 * _tmp58;
  const Scalar _tmp74 = Scalar(1.0) / (_tmp59 * _tmp71 - _tmp73);
  const Scalar _tmp75 = _tmp72 * _tmp74;
  const Scalar _tmp76 = _tmp50 + _tmp7;
  const Scalar _tmp77 = _tmp49 * _tmp60;
  const Scalar _tmp78 = _tmp51 * _tmp73 - _tmp61 * _tmp71;
  const Scalar _tmp79 = _tmp63 * _tmp76 - _tmp71 * _tmp77 - _tmp75 * _tmp78;
  const Scalar _tmp80 = Scalar(1.0) * _tmp64;
  const Scalar _tmp81 = -_tmp80;
  const Scalar _tmp82 = Scalar(1.0) / (_tmp55 + _tmp81);
  const Scalar _tmp83 = Scalar(1.0) * _tmp67;
  const Scalar _tmp84 = -_tmp52 + _tmp83;
  const Scalar _tmp85 = _tmp82 * _tmp84;
  const Scalar _tmp86 = -_tmp49 * _tmp76 - _tmp62 * _tmp75 + _tmp77 - _tmp79 * _tmp85;
  const Scalar _tmp87 = Scalar(1.0) / (_tmp86);
  const Scalar _tmp88 =
      std::sqrt(Scalar(std::pow(_tmp66, Scalar(2)) + std::pow(_tmp69, Scalar(2))));
  const Scalar _tmp89 = Scalar(1.0) / (_tmp88);
  const Scalar _tmp90 = _tmp70 * _tmp88;
  const Scalar _tmp91 = _tmp90 * (-_tmp64 * _tmp69 * _tmp89 + _tmp66 * _tmp67 * _tmp89);
  const Scalar _tmp92 = _tmp74 * (-_tmp52 * _tmp73 + _tmp55 * _tmp59 + _tmp59 * _tmp91);
  const Scalar _tmp93 = Scalar(1.0) * _tmp82;
  const Scalar _tmp94 = Scalar(1.0) * _tmp74;
  const Scalar _tmp95 = -_tmp62 * _tmp94 + _tmp74 * _tmp78 * _tmp84 * _tmp93;
  const Scalar _tmp96 = -_tmp41 * _tmp63 + _tmp45 * _tmp49 + _tmp49 * _tmp91 - _tmp72 * _tmp92;
  const Scalar _tmp97 = _tmp87 * _tmp96;
  const Scalar _tmp98 = Scalar(1.0) / (_tmp96);
  const Scalar _tmp99 = _tmp86 * _tmp98;
  const Scalar _tmp100 = _tmp99 * (-Scalar(1.0) * _tmp92 - _tmp95 * _tmp97);
  const Scalar _tmp101 = _tmp87 * (_tmp100 + _tmp95);
  const Scalar _tmp102 = -_tmp101 * _tmp72 + Scalar(1.0);
  const Scalar _tmp103 = _tmp59 * _tmp74;
  const Scalar _tmp104 = _tmp27 + Scalar(8.3888750099999996);
  const Scalar _tmp105 = _tmp34 + Scalar(-2.5202214700000001);
  const Scalar _tmp106 =
      std::pow(Scalar(std::pow(_tmp104, Scalar(2)) + std::pow(_tmp105, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp107 = _tmp104 * _tmp106;
  const Scalar _tmp108 = _tmp107 * fh1;
  const Scalar _tmp109 = _tmp71 * _tmp74;
  const Scalar _tmp110 = _tmp109 * _tmp78 + _tmp60 * _tmp71;
  const Scalar _tmp111 = _tmp109 * _tmp62 - _tmp110 * _tmp85 - _tmp60;
  const Scalar _tmp112 = _tmp99 * (-_tmp111 * _tmp97 + _tmp71 * _tmp92 - _tmp91);
  const Scalar _tmp113 = _tmp87 * (_tmp111 + _tmp112);
  const Scalar _tmp114 = -_tmp113 * _tmp72 - _tmp71;
  const Scalar _tmp115 = _tmp105 * _tmp106;
  const Scalar _tmp116 = _tmp115 * fh1;
  const Scalar _tmp117 = Scalar(1.0) * _tmp98;
  const Scalar _tmp118 = _tmp59 * _tmp75;
  const Scalar _tmp119 = fh1 * (_tmp107 * _tmp33 - _tmp115 * _tmp26);
  const Scalar _tmp120 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp121 = _tmp80 * _tmp85 + _tmp83;
  const Scalar _tmp122 = 0;
  const Scalar _tmp123 = -_tmp108 * _tmp90 * (_tmp101 * _tmp49 + _tmp102 * _tmp103) -
                         _tmp116 * _tmp90 * (_tmp103 * _tmp114 + _tmp113 * _tmp49 + Scalar(1.0)) -
                         _tmp119 * _tmp90 * (-_tmp117 * _tmp118 + _tmp117 * _tmp49) -
                         _tmp120 * _tmp90 * (-_tmp118 * _tmp122 + _tmp122 * _tmp49);
  const Scalar _tmp124 = Scalar(1.0) / (_tmp123);
  const Scalar _tmp125 = _tmp45 + _tmp81;
  const Scalar _tmp126 = _tmp125 * _tmp85;
  const Scalar _tmp127 = Scalar(1.0) / (-_tmp126 - _tmp41 + _tmp83);
  const Scalar _tmp128 = Scalar(1.0) * _tmp127;
  const Scalar _tmp129 = _tmp128 * _tmp99;
  const Scalar _tmp130 = -_tmp117 * _tmp79 + _tmp125 * _tmp129;
  const Scalar _tmp131 = _tmp125 * _tmp127;
  const Scalar _tmp132 = _tmp110 + _tmp112 * _tmp131 - _tmp113 * _tmp79;
  const Scalar _tmp133 = _tmp125 * _tmp82;
  const Scalar _tmp134 = _tmp16 * fh1;
  const Scalar _tmp135 = _tmp115 * _tmp134 + Scalar(5.1796800000000003) * _tmp29 + _tmp33 * fv1;
  const Scalar _tmp136 = _tmp126 * _tmp128 + Scalar(1.0);
  const Scalar _tmp137 = _tmp128 * _tmp85;
  const Scalar _tmp138 = -_tmp107 * _tmp134 - Scalar(5.1796800000000003) * _tmp20 - _tmp26 * fv1;
  const Scalar _tmp139 = _tmp100 * _tmp131 - _tmp101 * _tmp79 - _tmp78 * _tmp94;
  const Scalar _tmp140 = _tmp121 * _tmp127;
  const Scalar _tmp141 = -_tmp122 * _tmp79 - _tmp125 * _tmp140 + _tmp81;
  const Scalar _tmp142 = std::asinh(
      _tmp124 * (Scalar(1.0) * _tmp108 * (_tmp100 * _tmp128 - _tmp139 * _tmp93) +
                 Scalar(1.0) * _tmp116 * (_tmp112 * _tmp128 - _tmp132 * _tmp93) +
                 Scalar(1.0) * _tmp119 * (_tmp129 - _tmp130 * _tmp93) +
                 Scalar(1.0) * _tmp120 * (-_tmp121 * _tmp128 - _tmp141 * _tmp93 + Scalar(1.0)) +
                 Scalar(1.0) * _tmp135 * (_tmp128 * _tmp133 - _tmp128) +
                 Scalar(1.0) * _tmp138 * (-_tmp136 * _tmp93 + _tmp137)));
  const Scalar _tmp143 = Scalar(9.6622558468725703) * _tmp123;
  const Scalar _tmp144 = Scalar(22.802596067096832) *
                             std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp65), Scalar(2)) +
                         Scalar(7.3875128562042027) *
                             std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp68), Scalar(2));
  const Scalar _tmp145 =
      Scalar(3.9500536956656402) *
          std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp53 - 1), Scalar(2)) +
      Scalar(69.216682114881593) *
          std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp56 - 1), Scalar(2));
  const Scalar _tmp146 = _tmp128 * _tmp135;
  const Scalar _tmp147 = _tmp117 * _tmp119;
  const Scalar _tmp148 = _tmp120 * _tmp122;
  const Scalar _tmp149 =
      _tmp102 * _tmp108 * _tmp74 + _tmp114 * _tmp116 * _tmp74 - _tmp147 * _tmp75 - _tmp148 * _tmp75;
  const Scalar _tmp150 = Scalar(1.0) / (_tmp149);
  const Scalar _tmp151 =
      std::asinh(_tmp150 * (_tmp108 * _tmp139 * _tmp82 + _tmp116 * _tmp132 * _tmp82 +
                            _tmp119 * _tmp130 * _tmp82 + _tmp120 * _tmp141 * _tmp82 -
                            _tmp133 * _tmp146 + _tmp136 * _tmp138 * _tmp82));
  const Scalar _tmp152 = Scalar(9.6622558468725703) * _tmp149;
  const Scalar _tmp153 = _tmp101 * _tmp108 + _tmp113 * _tmp116 + _tmp147 + _tmp148;
  const Scalar _tmp154 = Scalar(1.0) / (_tmp153);
  const Scalar _tmp155 =
      std::asinh(_tmp154 * (-_tmp100 * _tmp108 * _tmp127 - _tmp112 * _tmp116 * _tmp127 -
                            _tmp119 * _tmp129 + _tmp120 * _tmp140 - _tmp137 * _tmp138 + _tmp146));
  const Scalar _tmp156 = Scalar(9.6622558468725703) * _tmp153;
  const Scalar _tmp157 = Scalar(23.361089618893828) *
                             std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp46), Scalar(2)) +
                         Scalar(3.2278567553341642) *
                             std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp42 - 1), Scalar(2));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp38 * (-std::sinh(Scalar(1.0) * _tmp37) -
                std::sinh(Scalar(0.1034955) * _tmp36 * (-std::sqrt(_tmp35) - _tmp37 * _tmp38))) -
      Scalar(8.4693136199999994) *
          std::sqrt(Scalar(Scalar(0.013941309530580858) * _tmp35 +
                           std::pow(Scalar(-Scalar(0.11807332268798426) * _tmp16 -
                                           Scalar(0.11807332268798426) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(1, 0) =
      _tmp143 *
          (-std::sinh(Scalar(1.0) * _tmp142) -
           std::sinh(Scalar(0.1034955) * _tmp124 * (-_tmp142 * _tmp143 - std::sqrt(_tmp144)))) -
      Scalar(8.36416322) *
          std::sqrt(Scalar(Scalar(0.014294040284261563) * _tmp144 +
                           std::pow(Scalar(-Scalar(0.1195576860108189) * _tmp60 -
                                           Scalar(0.1195576860108189) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(2, 0) =
      _tmp152 *
          (-std::sinh(Scalar(1.0) * _tmp151) -
           std::sinh(Scalar(0.1034955) * _tmp150 * (-std::sqrt(_tmp145) - _tmp151 * _tmp152))) -
      Scalar(8.4718465799999993) *
          std::sqrt(Scalar(Scalar(0.013932974275675287) * _tmp145 +
                           std::pow(Scalar(-Scalar(0.11803802046660766) * _tmp51 -
                                           Scalar(0.11803802046660766) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(3, 0) =
      _tmp156 *
          (-std::sinh(Scalar(1.0) * _tmp155) -
           std::sinh(Scalar(0.1034955) * _tmp154 * (-_tmp155 * _tmp156 - std::sqrt(_tmp157)))) -
      Scalar(8.3700199099999999) *
          std::sqrt(Scalar(Scalar(0.01427404356387209) * _tmp157 +
                           std::pow(Scalar(-Scalar(0.11947402882581673) * _tmp76 -
                                           Scalar(0.11947402882581673) * position_vector(2, 0) + 1),
                                    Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym