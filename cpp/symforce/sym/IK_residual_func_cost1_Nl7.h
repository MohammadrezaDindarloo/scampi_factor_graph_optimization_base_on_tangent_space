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
 * Symbolic function: IK_residual_func_cost1_Nl7
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1Nl7(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 500

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (155)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp3 = -2 * std::pow(_tmp2, Scalar(2));
  const Scalar _tmp4 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp5 = 1 - 2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp6 = Scalar(0.20999999999999999) * _tmp3 + Scalar(0.20999999999999999) * _tmp5;
  const Scalar _tmp7 = -_tmp6;
  const Scalar _tmp8 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp9 = 2 * _tmp4;
  const Scalar _tmp10 = _tmp8 * _tmp9;
  const Scalar _tmp11 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                        2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp12 = _tmp11 * _tmp2;
  const Scalar _tmp13 = _tmp10 - _tmp12;
  const Scalar _tmp14 = -Scalar(0.010999999999999999) * _tmp13;
  const Scalar _tmp15 = 2 * _tmp2 * _tmp8;
  const Scalar _tmp16 = _tmp11 * _tmp4;
  const Scalar _tmp17 = Scalar(0.20999999999999999) * _tmp15 + Scalar(0.20999999999999999) * _tmp16;
  const Scalar _tmp18 = _tmp14 + _tmp17;
  const Scalar _tmp19 = _tmp18 + _tmp7;
  const Scalar _tmp20 = _tmp19 + position_vector(1, 0);
  const Scalar _tmp21 = -2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999) * _tmp5;
  const Scalar _tmp23 = _tmp2 * _tmp9;
  const Scalar _tmp24 = _tmp11 * _tmp8;
  const Scalar _tmp25 = _tmp23 + _tmp24;
  const Scalar _tmp26 = -Scalar(0.010999999999999999) * _tmp25;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp15 - Scalar(0.20999999999999999) * _tmp16;
  const Scalar _tmp28 = _tmp26 - _tmp27;
  const Scalar _tmp29 = _tmp22 + _tmp28;
  const Scalar _tmp30 = _tmp29 + position_vector(0, 0);
  const Scalar _tmp31 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp32 = Scalar(0.20999999999999999) * _tmp23 - Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp33 = -Scalar(0.010999999999999999) * _tmp21 -
                        Scalar(0.010999999999999999) * _tmp3 + Scalar(-0.010999999999999999);
  const Scalar _tmp34 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp12;
  const Scalar _tmp35 = _tmp33 - _tmp34;
  const Scalar _tmp36 = _tmp32 + _tmp35;
  const Scalar _tmp37 = _tmp20 + Scalar(8.3888750099999996);
  const Scalar _tmp38 = _tmp30 + Scalar(-2.5202214700000001);
  const Scalar _tmp39 = std::pow(Scalar(std::pow(_tmp37, Scalar(2)) + std::pow(_tmp38, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp40 = _tmp37 * _tmp39;
  const Scalar _tmp41 = _tmp36 * fh1;
  const Scalar _tmp42 = -Scalar(5.1796800000000003) * _tmp13 - _tmp19 * fv1 - _tmp40 * _tmp41;
  const Scalar _tmp43 = _tmp14 - _tmp17;
  const Scalar _tmp44 = _tmp43 + _tmp7;
  const Scalar _tmp45 = Scalar(1.0) * _tmp44;
  const Scalar _tmp46 = -_tmp45;
  const Scalar _tmp47 = _tmp18 + _tmp6;
  const Scalar _tmp48 = _tmp46 + _tmp47;
  const Scalar _tmp49 = _tmp43 + _tmp6;
  const Scalar _tmp50 = Scalar(1.0) / (_tmp46 + _tmp49);
  const Scalar _tmp51 = -_tmp22;
  const Scalar _tmp52 = _tmp26 + _tmp27;
  const Scalar _tmp53 = _tmp51 + _tmp52;
  const Scalar _tmp54 = _tmp28 + _tmp51;
  const Scalar _tmp55 = Scalar(1.0) * _tmp54;
  const Scalar _tmp56 = _tmp50 * (-_tmp53 + _tmp55);
  const Scalar _tmp57 = _tmp48 * _tmp56;
  const Scalar _tmp58 = _tmp22 + _tmp52;
  const Scalar _tmp59 = Scalar(1.0) / (_tmp55 - _tmp57 - _tmp58);
  const Scalar _tmp60 = Scalar(1.0) * _tmp59;
  const Scalar _tmp61 = _tmp57 * _tmp60 + Scalar(1.0);
  const Scalar _tmp62 = Scalar(1.0) * _tmp50;
  const Scalar _tmp63 = _tmp56 * _tmp60;
  const Scalar _tmp64 = -_tmp32;
  const Scalar _tmp65 = _tmp33 + _tmp34;
  const Scalar _tmp66 = _tmp64 + _tmp65;
  const Scalar _tmp67 = _tmp53 + position_vector(0, 0);
  const Scalar _tmp68 = _tmp67 + Scalar(1.79662371);
  const Scalar _tmp69 = _tmp49 + position_vector(1, 0);
  const Scalar _tmp70 = _tmp69 + Scalar(-4.8333311099999996);
  const Scalar _tmp71 = std::pow(Scalar(std::pow(_tmp68, Scalar(2)) + std::pow(_tmp70, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp72 = _tmp68 * _tmp71;
  const Scalar _tmp73 = _tmp35 + _tmp64;
  const Scalar _tmp74 = -_tmp66 * _tmp72 + _tmp72 * _tmp73;
  const Scalar _tmp75 = _tmp70 * _tmp71;
  const Scalar _tmp76 = _tmp44 + position_vector(1, 0);
  const Scalar _tmp77 = _tmp76 + Scalar(8.3196563700000006);
  const Scalar _tmp78 = _tmp54 + position_vector(0, 0);
  const Scalar _tmp79 = _tmp78 + Scalar(1.9874742000000001);
  const Scalar _tmp80 = Scalar(1.0) / (_tmp79);
  const Scalar _tmp81 = _tmp77 * _tmp80;
  const Scalar _tmp82 = Scalar(1.0) / (_tmp72 * _tmp81 - _tmp75);
  const Scalar _tmp83 = _tmp47 + position_vector(1, 0);
  const Scalar _tmp84 = _tmp83 + Scalar(-4.7752063900000001);
  const Scalar _tmp85 = _tmp58 + position_vector(0, 0);
  const Scalar _tmp86 = _tmp85 + Scalar(-2.71799795);
  const Scalar _tmp87 = std::pow(Scalar(std::pow(_tmp84, Scalar(2)) + std::pow(_tmp86, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp88 = _tmp84 * _tmp87;
  const Scalar _tmp89 = _tmp86 * _tmp87;
  const Scalar _tmp90 = _tmp81 * _tmp89 - _tmp88;
  const Scalar _tmp91 = _tmp82 * _tmp90;
  const Scalar _tmp92 = _tmp32 + _tmp65;
  const Scalar _tmp93 = _tmp73 * _tmp89;
  const Scalar _tmp94 = _tmp73 * _tmp81;
  const Scalar _tmp95 = _tmp66 * _tmp75 - _tmp72 * _tmp94;
  const Scalar _tmp96 = -_tmp81 * _tmp93 + _tmp88 * _tmp92 - _tmp91 * _tmp95;
  const Scalar _tmp97 = -_tmp56 * _tmp96 - _tmp74 * _tmp91 - _tmp89 * _tmp92 + _tmp93;
  const Scalar _tmp98 =
      std::sqrt(Scalar(std::pow(_tmp77, Scalar(2)) + std::pow(_tmp79, Scalar(2))));
  const Scalar _tmp99 = Scalar(1.0) / (_tmp98);
  const Scalar _tmp100 = _tmp80 * _tmp98;
  const Scalar _tmp101 = _tmp100 * (-_tmp44 * _tmp79 * _tmp99 + _tmp54 * _tmp77 * _tmp99);
  const Scalar _tmp102 = _tmp101 * _tmp72 + _tmp49 * _tmp72 - _tmp53 * _tmp75;
  const Scalar _tmp103 = _tmp101 * _tmp89 - _tmp102 * _tmp91 + _tmp47 * _tmp89 - _tmp58 * _tmp88;
  const Scalar _tmp104 = Scalar(1.0) / (_tmp103);
  const Scalar _tmp105 = _tmp104 * _tmp97;
  const Scalar _tmp106 = _tmp105 * _tmp60;
  const Scalar _tmp107 = _tmp48 * _tmp60;
  const Scalar _tmp108 = Scalar(1.0) * _tmp104;
  const Scalar _tmp109 = _tmp105 * _tmp107 - _tmp108 * _tmp96;
  const Scalar _tmp110 = _tmp38 * _tmp39;
  const Scalar _tmp111 = fh1 * (-_tmp110 * _tmp19 + _tmp29 * _tmp40);
  const Scalar _tmp112 = _tmp110 * _tmp41 + Scalar(5.1796800000000003) * _tmp25 + _tmp29 * fv1;
  const Scalar _tmp113 = _tmp81 * _tmp82;
  const Scalar _tmp114 = _tmp113 * _tmp95 + _tmp94;
  const Scalar _tmp115 = _tmp113 * _tmp74 - _tmp114 * _tmp56 - _tmp73;
  const Scalar _tmp116 = Scalar(1.0) / (_tmp97);
  const Scalar _tmp117 = _tmp103 * _tmp116;
  const Scalar _tmp118 = _tmp105 * (-_tmp101 + _tmp102 * _tmp113 - _tmp115 * _tmp117);
  const Scalar _tmp119 = _tmp116 * (_tmp115 + _tmp118);
  const Scalar _tmp120 = _tmp48 * _tmp59;
  const Scalar _tmp121 = _tmp114 + _tmp118 * _tmp120 - _tmp119 * _tmp96;
  const Scalar _tmp122 = _tmp110 * fh1;
  const Scalar _tmp123 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp124 = _tmp45 * _tmp56 + _tmp55;
  const Scalar _tmp125 = _tmp124 * _tmp59;
  const Scalar _tmp126 = 0;
  const Scalar _tmp127 = _tmp50 * (-_tmp125 * _tmp48 - _tmp126 * _tmp96 + _tmp46);
  const Scalar _tmp128 = Scalar(1.0) * _tmp82;
  const Scalar _tmp129 = _tmp128 * _tmp95;
  const Scalar _tmp130 = -_tmp128 * _tmp74 + _tmp129 * _tmp56;
  const Scalar _tmp131 = _tmp105 * (-_tmp102 * _tmp128 - _tmp117 * _tmp130);
  const Scalar _tmp132 = _tmp116 * (_tmp130 + _tmp131);
  const Scalar _tmp133 = _tmp120 * _tmp131 - _tmp129 - _tmp132 * _tmp96;
  const Scalar _tmp134 = _tmp40 * fh1;
  const Scalar _tmp135 = _tmp72 * _tmp91;
  const Scalar _tmp136 = -_tmp119 * _tmp90 - _tmp81;
  const Scalar _tmp137 = _tmp72 * _tmp82;
  const Scalar _tmp138 = -_tmp132 * _tmp90 + Scalar(1.0);
  const Scalar _tmp139 = -_tmp100 * _tmp111 * (-_tmp108 * _tmp135 + _tmp108 * _tmp89) -
                         _tmp100 * _tmp122 * (_tmp119 * _tmp89 + _tmp136 * _tmp137 + Scalar(1.0)) -
                         _tmp100 * _tmp123 * (-_tmp126 * _tmp135 + _tmp126 * _tmp89) -
                         _tmp100 * _tmp134 * (_tmp132 * _tmp89 + _tmp137 * _tmp138);
  const Scalar _tmp140 = Scalar(1.0) / (_tmp139);
  const Scalar _tmp141 = std::asinh(
      _tmp140 * (Scalar(1.0) * _tmp111 * (_tmp106 - _tmp109 * _tmp62) +
                 Scalar(1.0) * _tmp112 * (_tmp107 * _tmp50 - _tmp60) +
                 Scalar(1.0) * _tmp122 * (_tmp118 * _tmp60 - _tmp121 * _tmp62) +
                 Scalar(1.0) * _tmp123 * (-_tmp124 * _tmp60 - Scalar(1.0) * _tmp127 + Scalar(1.0)) +
                 Scalar(1.0) * _tmp134 * (_tmp131 * _tmp60 - _tmp133 * _tmp62) +
                 Scalar(1.0) * _tmp42 * (-_tmp61 * _tmp62 + _tmp63)));
  const Scalar _tmp142 = Scalar(9.6622558468725703) * _tmp139;
  const Scalar _tmp143 = _tmp108 * _tmp111;
  const Scalar _tmp144 = _tmp123 * _tmp126;
  const Scalar _tmp145 =
      _tmp122 * _tmp136 * _tmp82 + _tmp134 * _tmp138 * _tmp82 - _tmp143 * _tmp91 - _tmp144 * _tmp91;
  const Scalar _tmp146 = Scalar(1.0) / (_tmp145);
  const Scalar _tmp147 = _tmp50 * fh1;
  const Scalar _tmp148 = _tmp112 * _tmp60;
  const Scalar _tmp149 =
      std::asinh(_tmp146 * (_tmp109 * _tmp111 * _tmp50 + _tmp110 * _tmp121 * _tmp147 +
                            _tmp123 * _tmp127 + _tmp133 * _tmp147 * _tmp40 -
                            _tmp148 * _tmp48 * _tmp50 + _tmp42 * _tmp50 * _tmp61));
  const Scalar _tmp150 = Scalar(9.6622558468725703) * _tmp145;
  const Scalar _tmp151 = _tmp119 * _tmp122 + _tmp132 * _tmp134 + _tmp143 + _tmp144;
  const Scalar _tmp152 = Scalar(1.0) / (_tmp151);
  const Scalar _tmp153 =
      std::asinh(_tmp152 * (-_tmp106 * _tmp111 - _tmp118 * _tmp122 * _tmp59 + _tmp123 * _tmp125 -
                            _tmp131 * _tmp134 * _tmp59 + _tmp148 - _tmp42 * _tmp63));
  const Scalar _tmp154 = Scalar(9.6622558468725703) * _tmp151;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -_tmp31 *
          (Scalar(0.87653584775870996) * _tmp0 + std::cosh(Scalar(1.0) * _tmp1) -
           std::cosh(
               Scalar(0.1034955) * _tmp0 *
               (-_tmp1 * _tmp31 -
                Scalar(8.3888750099999996) *
                    std::sqrt(Scalar(
                        Scalar(0.090254729040973036) *
                            std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp30), Scalar(2)) +
                        std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp20 - 1),
                                 Scalar(2))))))) +
      _tmp36 + position_vector(2, 0);
  _res(1, 0) =
      -_tmp142 *
          (Scalar(0.87679799772039002) * _tmp140 + std::cosh(Scalar(1.0) * _tmp141) -
           std::cosh(
               Scalar(0.1034955) * _tmp140 *
               (-_tmp141 * _tmp142 -
                Scalar(8.3196563700000006) *
                    std::sqrt(Scalar(
                        std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp76 - 1), Scalar(2)) +
                        Scalar(0.057067943376852184) *
                            std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp78 - 1),
                                     Scalar(2))))))) +
      _tmp73 + position_vector(2, 0);
  _res(2, 0) =
      -_tmp150 *
          (Scalar(0.86625939559540499) * _tmp146 + std::cosh(Scalar(1.0) * _tmp149) -
           std::cosh(
               Scalar(0.1034955) * _tmp146 *
               (-_tmp149 * _tmp150 -
                Scalar(4.8333311099999996) *
                    std::sqrt(Scalar(
                        std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp69), Scalar(2)) +
                        Scalar(0.13817235445745474) *
                            std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp67 - 1),
                                     Scalar(2))))))) +
      _tmp66 + position_vector(2, 0);
  _res(3, 0) =
      -_tmp154 *
          (Scalar(0.86565325453551001) * _tmp152 + std::cosh(Scalar(1.0) * _tmp153) -
           std::cosh(
               Scalar(0.1034955) * _tmp152 *
               (-_tmp153 * _tmp154 -
                Scalar(4.7752063900000001) *
                    std::sqrt(Scalar(
                        std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp83), Scalar(2)) +
                        Scalar(0.32397683292140877) *
                            std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp85),
                                     Scalar(2))))))) +
      _tmp92 + position_vector(2, 0);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
