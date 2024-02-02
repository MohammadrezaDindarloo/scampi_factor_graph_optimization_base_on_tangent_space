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
 * Symbolic function: IK_residual_func_cost2_Nl15
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2Nl15(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 531

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (161)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp3 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp4 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp5 = 2 * _tmp3 * _tmp4;
  const Scalar _tmp6 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp7 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                       2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp8 = _tmp6 * _tmp7;
  const Scalar _tmp9 = Scalar(0.20999999999999999) * _tmp5 - Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp10 = -2 * std::pow(_tmp6, Scalar(2));
  const Scalar _tmp11 = 1 - 2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp12 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp11;
  const Scalar _tmp13 = 2 * _tmp6;
  const Scalar _tmp14 = _tmp13 * _tmp4;
  const Scalar _tmp15 = _tmp3 * _tmp7;
  const Scalar _tmp16 = _tmp14 + _tmp15;
  const Scalar _tmp17 = -Scalar(0.010999999999999999) * _tmp16;
  const Scalar _tmp18 = _tmp12 + _tmp17;
  const Scalar _tmp19 = _tmp18 + _tmp9;
  const Scalar _tmp20 = _tmp19 + position_vector(0, 0);
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp5 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp22 = -2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp23 = Scalar(0.20999999999999999) * _tmp10 +
                        Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999);
  const Scalar _tmp24 = _tmp13 * _tmp3;
  const Scalar _tmp25 = _tmp4 * _tmp7;
  const Scalar _tmp26 = _tmp24 - _tmp25;
  const Scalar _tmp27 = -Scalar(0.010999999999999999) * _tmp26;
  const Scalar _tmp28 = _tmp23 + _tmp27;
  const Scalar _tmp29 = _tmp21 + _tmp28;
  const Scalar _tmp30 = _tmp29 + position_vector(1, 0);
  const Scalar _tmp31 = Scalar(7.3875128562042027) *
                            std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp20), Scalar(2)) +
                        Scalar(22.802596067096832) *
                            std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp30), Scalar(2));
  const Scalar _tmp32 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp33 =
      -Scalar(0.010999999999999999) * _tmp11 - Scalar(0.010999999999999999) * _tmp22;
  const Scalar _tmp34 = Scalar(0.20999999999999999) * _tmp14 - Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp35 = _tmp33 + _tmp34;
  const Scalar _tmp36 = _tmp32 + _tmp35;
  const Scalar _tmp37 = -_tmp32;
  const Scalar _tmp38 = _tmp35 + _tmp37;
  const Scalar _tmp39 = -_tmp23 + _tmp27;
  const Scalar _tmp40 = _tmp21 + _tmp39;
  const Scalar _tmp41 = _tmp40 + position_vector(1, 0);
  const Scalar _tmp42 = -_tmp9;
  const Scalar _tmp43 = _tmp18 + _tmp42;
  const Scalar _tmp44 = _tmp43 + position_vector(0, 0);
  const Scalar _tmp45 = Scalar(6.351516257848961) *
                            std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp44), Scalar(2)) +
                        Scalar(70.3732239334025) *
                            std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp41 - 1), Scalar(2));
  const Scalar _tmp46 = _tmp33 - _tmp34;
  const Scalar _tmp47 = _tmp32 + _tmp46;
  const Scalar _tmp48 = -_tmp12 + _tmp17;
  const Scalar _tmp49 = _tmp48 + _tmp9;
  const Scalar _tmp50 = _tmp49 + position_vector(0, 0);
  const Scalar _tmp51 = _tmp50 + Scalar(1.79662371);
  const Scalar _tmp52 = -_tmp21;
  const Scalar _tmp53 = _tmp28 + _tmp52;
  const Scalar _tmp54 = _tmp53 + position_vector(1, 0);
  const Scalar _tmp55 = _tmp54 + Scalar(-4.8333311099999996);
  const Scalar _tmp56 = std::pow(Scalar(std::pow(_tmp51, Scalar(2)) + std::pow(_tmp55, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp57 = _tmp51 * _tmp56;
  const Scalar _tmp58 = _tmp38 * _tmp57 - _tmp47 * _tmp57;
  const Scalar _tmp59 = _tmp55 * _tmp56;
  const Scalar _tmp60 = _tmp44 + Scalar(-2.5202214700000001);
  const Scalar _tmp61 = Scalar(1.0) / (_tmp60);
  const Scalar _tmp62 = _tmp41 + Scalar(8.3888750099999996);
  const Scalar _tmp63 = _tmp61 * _tmp62;
  const Scalar _tmp64 = Scalar(1.0) / (_tmp57 * _tmp63 - _tmp59);
  const Scalar _tmp65 = _tmp63 * _tmp64;
  const Scalar _tmp66 = _tmp38 * _tmp63;
  const Scalar _tmp67 = _tmp47 * _tmp59 - _tmp57 * _tmp66;
  const Scalar _tmp68 = _tmp65 * _tmp67 + _tmp66;
  const Scalar _tmp69 = Scalar(1.0) * _tmp40;
  const Scalar _tmp70 = -_tmp69;
  const Scalar _tmp71 = Scalar(1.0) / (_tmp53 + _tmp70);
  const Scalar _tmp72 = Scalar(1.0) * _tmp43;
  const Scalar _tmp73 = _tmp71 * (-_tmp49 + _tmp72);
  const Scalar _tmp74 = -_tmp38 + _tmp58 * _tmp65 - _tmp68 * _tmp73;
  const Scalar _tmp75 = _tmp42 + _tmp48;
  const Scalar _tmp76 = _tmp75 + position_vector(0, 0);
  const Scalar _tmp77 = _tmp76 + Scalar(1.9874742000000001);
  const Scalar _tmp78 = _tmp39 + _tmp52;
  const Scalar _tmp79 = _tmp78 + position_vector(1, 0);
  const Scalar _tmp80 = _tmp79 + Scalar(8.3196563700000006);
  const Scalar _tmp81 = std::pow(Scalar(std::pow(_tmp77, Scalar(2)) + std::pow(_tmp80, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp82 = _tmp77 * _tmp81;
  const Scalar _tmp83 = _tmp37 + _tmp46;
  const Scalar _tmp84 = _tmp80 * _tmp81;
  const Scalar _tmp85 = _tmp63 * _tmp82 - _tmp84;
  const Scalar _tmp86 = _tmp64 * _tmp85;
  const Scalar _tmp87 = -_tmp66 * _tmp82 - _tmp67 * _tmp86 + _tmp83 * _tmp84;
  const Scalar _tmp88 = _tmp38 * _tmp82 - _tmp58 * _tmp86 - _tmp73 * _tmp87 - _tmp82 * _tmp83;
  const Scalar _tmp89 = Scalar(1.0) / (_tmp88);
  const Scalar _tmp90 =
      std::sqrt(Scalar(std::pow(_tmp60, Scalar(2)) + std::pow(_tmp62, Scalar(2))));
  const Scalar _tmp91 = Scalar(1.0) / (_tmp90);
  const Scalar _tmp92 = _tmp61 * _tmp90;
  const Scalar _tmp93 = _tmp92 * (-_tmp40 * _tmp60 * _tmp91 + _tmp43 * _tmp62 * _tmp91);
  const Scalar _tmp94 = -_tmp49 * _tmp59 + _tmp53 * _tmp57 + _tmp57 * _tmp93;
  const Scalar _tmp95 = -_tmp75 * _tmp84 + _tmp78 * _tmp82 + _tmp82 * _tmp93 - _tmp86 * _tmp94;
  const Scalar _tmp96 = _tmp89 * _tmp95;
  const Scalar _tmp97 = Scalar(1.0) / (_tmp95);
  const Scalar _tmp98 = _tmp88 * _tmp97;
  const Scalar _tmp99 = _tmp98 * (_tmp65 * _tmp94 - _tmp74 * _tmp96 - _tmp93);
  const Scalar _tmp100 = _tmp74 + _tmp99;
  const Scalar _tmp101 = _tmp82 * _tmp89;
  const Scalar _tmp102 = _tmp85 * _tmp89;
  const Scalar _tmp103 = -_tmp100 * _tmp102 - _tmp63;
  const Scalar _tmp104 = _tmp57 * _tmp64;
  const Scalar _tmp105 = _tmp20 + Scalar(-2.71799795);
  const Scalar _tmp106 = _tmp30 + Scalar(-4.7752063900000001);
  const Scalar _tmp107 =
      std::pow(Scalar(std::pow(_tmp105, Scalar(2)) + std::pow(_tmp106, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp108 = _tmp105 * _tmp107;
  const Scalar _tmp109 = _tmp108 * fh1;
  const Scalar _tmp110 = Scalar(1.0) * _tmp64;
  const Scalar _tmp111 = _tmp110 * _tmp67;
  const Scalar _tmp112 = -_tmp110 * _tmp58 + _tmp111 * _tmp73;
  const Scalar _tmp113 = _tmp98 * (-_tmp110 * _tmp94 - _tmp112 * _tmp96);
  const Scalar _tmp114 = _tmp112 + _tmp113;
  const Scalar _tmp115 = -_tmp102 * _tmp114 + Scalar(1.0);
  const Scalar _tmp116 = _tmp106 * _tmp107;
  const Scalar _tmp117 = _tmp116 * fh1;
  const Scalar _tmp118 = Scalar(1.0) * _tmp97;
  const Scalar _tmp119 = _tmp57 * _tmp86;
  const Scalar _tmp120 = fh1 * (-_tmp108 * _tmp29 + _tmp116 * _tmp19);
  const Scalar _tmp121 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp122 = _tmp69 * _tmp73 + _tmp72;
  const Scalar _tmp123 = 0;
  const Scalar _tmp124 = _tmp123 * _tmp89;
  const Scalar _tmp125 = -_tmp109 * _tmp92 * (_tmp100 * _tmp101 + _tmp103 * _tmp104 + Scalar(1.0)) -
                         _tmp117 * _tmp92 * (_tmp101 * _tmp114 + _tmp104 * _tmp115) -
                         _tmp120 * _tmp92 * (-_tmp118 * _tmp119 + _tmp118 * _tmp82) -
                         _tmp121 * _tmp92 * (-_tmp119 * _tmp124 + _tmp124 * _tmp82);
  const Scalar _tmp126 = Scalar(1.0) / (_tmp125);
  const Scalar _tmp127 = _tmp70 + _tmp78;
  const Scalar _tmp128 = _tmp127 * _tmp73;
  const Scalar _tmp129 = Scalar(1.0) / (-_tmp128 + _tmp72 - _tmp75);
  const Scalar _tmp130 = Scalar(1.0) * _tmp129;
  const Scalar _tmp131 = _tmp87 * _tmp89;
  const Scalar _tmp132 = _tmp122 * _tmp129;
  const Scalar _tmp133 = _tmp71 * (-_tmp123 * _tmp131 - _tmp127 * _tmp132 + _tmp70);
  const Scalar _tmp134 = _tmp127 * _tmp130;
  const Scalar _tmp135 = -_tmp118 * _tmp87 + _tmp134 * _tmp98;
  const Scalar _tmp136 = Scalar(1.0) * _tmp71;
  const Scalar _tmp137 = _tmp130 * _tmp98;
  const Scalar _tmp138 = _tmp127 * _tmp129;
  const Scalar _tmp139 = -_tmp100 * _tmp131 + _tmp138 * _tmp99 + _tmp68;
  const Scalar _tmp140 = -_tmp111 + _tmp113 * _tmp138 - _tmp114 * _tmp131;
  const Scalar _tmp141 = _tmp36 * fh1;
  const Scalar _tmp142 = -_tmp116 * _tmp141 - Scalar(5.1796800000000003) * _tmp26 - _tmp29 * fv1;
  const Scalar _tmp143 = _tmp71 * (_tmp128 * _tmp130 + Scalar(1.0));
  const Scalar _tmp144 = _tmp130 * _tmp73;
  const Scalar _tmp145 = _tmp108 * _tmp141 + Scalar(5.1796800000000003) * _tmp16 + _tmp19 * fv1;
  const Scalar _tmp146 =
      std::asinh(_tmp126 * (Scalar(1.0) * _tmp109 * (_tmp130 * _tmp99 - _tmp136 * _tmp139) +
                            Scalar(1.0) * _tmp117 * (_tmp113 * _tmp130 - _tmp136 * _tmp140) +
                            Scalar(1.0) * _tmp120 * (-_tmp135 * _tmp136 + _tmp137) +
                            Scalar(1.0) * _tmp121 *
                                (-_tmp122 * _tmp130 - Scalar(1.0) * _tmp133 + Scalar(1.0)) +
                            Scalar(1.0) * _tmp142 * (-Scalar(1.0) * _tmp143 + _tmp144) +
                            Scalar(1.0) * _tmp145 * (-_tmp130 + _tmp134 * _tmp71)));
  const Scalar _tmp147 = Scalar(9.6622558468725703) * _tmp125;
  const Scalar _tmp148 = _tmp130 * _tmp145;
  const Scalar _tmp149 = _tmp121 * _tmp124;
  const Scalar _tmp150 = _tmp118 * _tmp120;
  const Scalar _tmp151 =
      _tmp103 * _tmp109 * _tmp64 + _tmp115 * _tmp117 * _tmp64 - _tmp149 * _tmp86 - _tmp150 * _tmp86;
  const Scalar _tmp152 = Scalar(1.0) / (_tmp151);
  const Scalar _tmp153 =
      std::asinh(_tmp152 * (_tmp109 * _tmp139 * _tmp71 + _tmp117 * _tmp140 * _tmp71 +
                            _tmp120 * _tmp135 * _tmp71 + _tmp121 * _tmp133 -
                            _tmp127 * _tmp148 * _tmp71 + _tmp142 * _tmp143));
  const Scalar _tmp154 = Scalar(9.6622558468725703) * _tmp151;
  const Scalar _tmp155 = Scalar(23.361089618893828) *
                             std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp54), Scalar(2)) +
                         Scalar(3.2278567553341642) *
                             std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp50 - 1), Scalar(2));
  const Scalar _tmp156 =
      _tmp100 * _tmp109 * _tmp89 + _tmp114 * _tmp117 * _tmp89 + _tmp149 + _tmp150;
  const Scalar _tmp157 = Scalar(1.0) / (_tmp156);
  const Scalar _tmp158 =
      std::asinh(_tmp157 * (-_tmp109 * _tmp129 * _tmp99 - _tmp113 * _tmp117 * _tmp129 -
                            _tmp120 * _tmp137 + _tmp121 * _tmp132 - _tmp142 * _tmp144 + _tmp148));
  const Scalar _tmp159 = Scalar(9.6622558468725703) * _tmp156;
  const Scalar _tmp160 =
      Scalar(3.9500536956656402) *
          std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp76 - 1), Scalar(2)) +
      Scalar(69.216682114881593) *
          std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp79 - 1), Scalar(2));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp2 * (-std::sinh(Scalar(1.0) * _tmp1) -
               std::sinh(Scalar(0.1034955) * _tmp0 * (-_tmp1 * _tmp2 - std::sqrt(_tmp31)))) -
      Scalar(8.36416322) *
          std::sqrt(Scalar(Scalar(0.014294040284261563) * _tmp31 +
                           std::pow(Scalar(-Scalar(0.1195576860108189) * _tmp36 -
                                           Scalar(0.1195576860108189) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(1, 0) =
      _tmp147 *
          (-std::sinh(Scalar(1.0) * _tmp146) -
           std::sinh(Scalar(0.1034955) * _tmp126 * (-_tmp146 * _tmp147 - std::sqrt(_tmp45)))) -
      Scalar(8.4693136199999994) *
          std::sqrt(Scalar(Scalar(0.013941309530580858) * _tmp45 +
                           std::pow(Scalar(-Scalar(0.11807332268798426) * _tmp38 -
                                           Scalar(0.11807332268798426) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(2, 0) =
      _tmp154 *
          (-std::sinh(Scalar(1.0) * _tmp153) -
           std::sinh(Scalar(0.1034955) * _tmp152 * (-_tmp153 * _tmp154 - std::sqrt(_tmp155)))) -
      Scalar(8.3700199099999999) *
          std::sqrt(Scalar(Scalar(0.01427404356387209) * _tmp155 +
                           std::pow(Scalar(-Scalar(0.11947402882581673) * _tmp47 -
                                           Scalar(0.11947402882581673) * position_vector(2, 0) + 1),
                                    Scalar(2))));
  _res(3, 0) =
      _tmp159 *
          (-std::sinh(Scalar(1.0) * _tmp158) -
           std::sinh(Scalar(0.1034955) * _tmp157 * (-_tmp158 * _tmp159 - std::sqrt(_tmp160)))) -
      Scalar(8.4718465799999993) *
          std::sqrt(Scalar(Scalar(0.013932974275675287) * _tmp160 +
                           std::pow(Scalar(-Scalar(0.11803802046660766) * _tmp83 -
                                           Scalar(0.11803802046660766) * position_vector(2, 0) + 1),
                                    Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
