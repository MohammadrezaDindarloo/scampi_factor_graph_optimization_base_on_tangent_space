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
 * Symbolic function: IK_residual_func_cost2_Nl21
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2Nl21(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 529

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (162)
  const Scalar _tmp0 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp1 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp2 = 2 * _tmp1;
  const Scalar _tmp3 = _tmp0 * _tmp2;
  const Scalar _tmp4 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp5 = -_DeltaRot[0] * _Rot_init[0] - _DeltaRot[1] * _Rot_init[1] -
                       _DeltaRot[2] * _Rot_init[2] + _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp6 = 2 * _tmp4 * _tmp5;
  const Scalar _tmp7 = Scalar(0.20999999999999999) * _tmp3 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp8 = -2 * std::pow(_tmp0, Scalar(2));
  const Scalar _tmp9 = -2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp10 = -Scalar(0.010999999999999999) * _tmp8 -
                        Scalar(0.010999999999999999) * _tmp9 + Scalar(-0.010999999999999999);
  const Scalar _tmp11 = _tmp2 * _tmp4;
  const Scalar _tmp12 = 2 * _tmp0;
  const Scalar _tmp13 = _tmp12 * _tmp5;
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp11 - Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 = _tmp10 - _tmp14;
  const Scalar _tmp16 = _tmp15 + _tmp7;
  const Scalar _tmp17 = 1 - 2 * std::pow(_tmp1, Scalar(2));
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp19 = -_tmp18;
  const Scalar _tmp20 = _tmp11 + _tmp13;
  const Scalar _tmp21 = -Scalar(0.010999999999999999) * _tmp20;
  const Scalar _tmp22 = _tmp12 * _tmp4;
  const Scalar _tmp23 = _tmp2 * _tmp5;
  const Scalar _tmp24 = Scalar(0.20999999999999999) * _tmp22 - Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp25 = _tmp21 + _tmp24;
  const Scalar _tmp26 = _tmp19 + _tmp25;
  const Scalar _tmp27 = _tmp26 + position_vector(0, 0);
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp29 = Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp30 = _tmp3 - _tmp6;
  const Scalar _tmp31 = -Scalar(0.010999999999999999) * _tmp30;
  const Scalar _tmp32 = -_tmp29 + _tmp31;
  const Scalar _tmp33 = _tmp28 + _tmp32;
  const Scalar _tmp34 = _tmp33 + position_vector(1, 0);
  const Scalar _tmp35 =
      Scalar(12100.0) * std::pow(Scalar(1 - Scalar(0.0090909090909090905) * _tmp34), Scalar(2)) +
      Scalar(15625.0) * std::pow(Scalar(-Scalar(0.0080000000000000002) * _tmp27 - 1), Scalar(2));
  const Scalar _tmp36 = Scalar(1.0) / (fh1);
  const Scalar _tmp37 = std::asinh(_tmp36 * fv1);
  const Scalar _tmp38 = Scalar(1.4083112389913199) * fh1;
  const Scalar _tmp39 = -_tmp28;
  const Scalar _tmp40 = _tmp29 + _tmp31;
  const Scalar _tmp41 = _tmp39 + _tmp40;
  const Scalar _tmp42 = _tmp41 + position_vector(1, 0);
  const Scalar _tmp43 = _tmp21 - _tmp24;
  const Scalar _tmp44 = _tmp18 + _tmp43;
  const Scalar _tmp45 = _tmp44 + position_vector(0, 0);
  const Scalar _tmp46 =
      Scalar(15625.0) * std::pow(Scalar(1 - Scalar(0.0080000000000000002) * _tmp45), Scalar(2)) +
      Scalar(12100.0) * std::pow(Scalar(-Scalar(0.0090909090909090905) * _tmp42 - 1), Scalar(2));
  const Scalar _tmp47 = Scalar(1.0) * _tmp44;
  const Scalar _tmp48 = Scalar(1.0) * _tmp41;
  const Scalar _tmp49 = -_tmp48;
  const Scalar _tmp50 = _tmp32 + _tmp39;
  const Scalar _tmp51 = _tmp49 + _tmp50;
  const Scalar _tmp52 = _tmp28 + _tmp40;
  const Scalar _tmp53 = Scalar(1.0) / (_tmp49 + _tmp52);
  const Scalar _tmp54 = _tmp18 + _tmp25;
  const Scalar _tmp55 = _tmp53 * (_tmp47 - _tmp54);
  const Scalar _tmp56 = _tmp51 * _tmp55;
  const Scalar _tmp57 = _tmp19 + _tmp43;
  const Scalar _tmp58 = Scalar(1.0) / (_tmp47 - _tmp56 - _tmp57);
  const Scalar _tmp59 = Scalar(1.0) * _tmp58;
  const Scalar _tmp60 = -_tmp7;
  const Scalar _tmp61 = _tmp10 + _tmp14;
  const Scalar _tmp62 = _tmp60 + _tmp61;
  const Scalar _tmp63 = _tmp45 + Scalar(-125.0);
  const Scalar _tmp64 = Scalar(1.0) / (_tmp63);
  const Scalar _tmp65 = _tmp42 + Scalar(110.0);
  const Scalar _tmp66 = _tmp64 * _tmp65;
  const Scalar _tmp67 = _tmp52 + position_vector(1, 0);
  const Scalar _tmp68 = _tmp67 + Scalar(-110.0);
  const Scalar _tmp69 = _tmp54 + position_vector(0, 0);
  const Scalar _tmp70 = _tmp69 + Scalar(-125.0);
  const Scalar _tmp71 = std::pow(Scalar(std::pow(_tmp68, Scalar(2)) + std::pow(_tmp70, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp72 = _tmp70 * _tmp71;
  const Scalar _tmp73 = _tmp62 * _tmp72;
  const Scalar _tmp74 = _tmp61 + _tmp7;
  const Scalar _tmp75 = _tmp68 * _tmp71;
  const Scalar _tmp76 = -_tmp66 * _tmp73 + _tmp74 * _tmp75;
  const Scalar _tmp77 = Scalar(1.0) / (_tmp66 * _tmp72 - _tmp75);
  const Scalar _tmp78 = _tmp66 * _tmp77;
  const Scalar _tmp79 = _tmp62 * _tmp66 + _tmp76 * _tmp78;
  const Scalar _tmp80 = -_tmp72 * _tmp74 + _tmp73;
  const Scalar _tmp81 = -_tmp55 * _tmp79 - _tmp62 + _tmp78 * _tmp80;
  const Scalar _tmp82 = _tmp50 + position_vector(1, 0);
  const Scalar _tmp83 = _tmp82 + Scalar(110.0);
  const Scalar _tmp84 = _tmp57 + position_vector(0, 0);
  const Scalar _tmp85 = _tmp84 + Scalar(125.0);
  const Scalar _tmp86 = std::pow(Scalar(std::pow(_tmp83, Scalar(2)) + std::pow(_tmp85, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp87 = _tmp85 * _tmp86;
  const Scalar _tmp88 = _tmp62 * _tmp87;
  const Scalar _tmp89 = _tmp15 + _tmp60;
  const Scalar _tmp90 = _tmp83 * _tmp86;
  const Scalar _tmp91 = _tmp66 * _tmp87 - _tmp90;
  const Scalar _tmp92 = _tmp77 * _tmp91;
  const Scalar _tmp93 = -_tmp66 * _tmp88 - _tmp76 * _tmp92 + _tmp89 * _tmp90;
  const Scalar _tmp94 = -_tmp55 * _tmp93 - _tmp80 * _tmp92 - _tmp87 * _tmp89 + _tmp88;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp94);
  const Scalar _tmp96 =
      std::sqrt(Scalar(std::pow(_tmp63, Scalar(2)) + std::pow(_tmp65, Scalar(2))));
  const Scalar _tmp97 = Scalar(1.0) / (_tmp96);
  const Scalar _tmp98 = _tmp64 * _tmp96;
  const Scalar _tmp99 = _tmp98 * (-_tmp41 * _tmp63 * _tmp97 + _tmp44 * _tmp65 * _tmp97);
  const Scalar _tmp100 = _tmp52 * _tmp72 - _tmp54 * _tmp75 + _tmp72 * _tmp99;
  const Scalar _tmp101 = -_tmp100 * _tmp92 + _tmp50 * _tmp87 - _tmp57 * _tmp90 + _tmp87 * _tmp99;
  const Scalar _tmp102 = _tmp101 * _tmp95;
  const Scalar _tmp103 = Scalar(1.0) / (_tmp101);
  const Scalar _tmp104 = _tmp103 * _tmp94;
  const Scalar _tmp105 = _tmp104 * (_tmp100 * _tmp78 - _tmp102 * _tmp81 - _tmp99);
  const Scalar _tmp106 = _tmp105 + _tmp81;
  const Scalar _tmp107 = _tmp93 * _tmp95;
  const Scalar _tmp108 = _tmp51 * _tmp58;
  const Scalar _tmp109 = _tmp105 * _tmp108 - _tmp106 * _tmp107 + _tmp79;
  const Scalar _tmp110 = Scalar(1.0) * _tmp53;
  const Scalar _tmp111 = _tmp27 + Scalar(125.0);
  const Scalar _tmp112 = _tmp34 + Scalar(-110.0);
  const Scalar _tmp113 =
      std::pow(Scalar(std::pow(_tmp111, Scalar(2)) + std::pow(_tmp112, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp114 = _tmp111 * _tmp113;
  const Scalar _tmp115 = _tmp114 * fh1;
  const Scalar _tmp116 = Scalar(333.54000000000002) - fv1;
  const Scalar _tmp117 = _tmp47 + _tmp48 * _tmp55;
  const Scalar _tmp118 = _tmp117 * _tmp58;
  const Scalar _tmp119 = 0;
  const Scalar _tmp120 = _tmp53 * (-_tmp107 * _tmp119 - _tmp118 * _tmp51 + _tmp49);
  const Scalar _tmp121 = Scalar(1.0) * _tmp77;
  const Scalar _tmp122 = _tmp121 * _tmp76;
  const Scalar _tmp123 = -_tmp121 * _tmp80 + _tmp122 * _tmp55;
  const Scalar _tmp124 = _tmp104 * (-_tmp100 * _tmp121 - _tmp102 * _tmp123);
  const Scalar _tmp125 = _tmp123 + _tmp124;
  const Scalar _tmp126 = -_tmp107 * _tmp125 + _tmp108 * _tmp124 - _tmp122;
  const Scalar _tmp127 = _tmp112 * _tmp113;
  const Scalar _tmp128 = _tmp127 * fh1;
  const Scalar _tmp129 = Scalar(1.0) * _tmp103;
  const Scalar _tmp130 = _tmp104 * _tmp59;
  const Scalar _tmp131 = -_tmp129 * _tmp93 + _tmp130 * _tmp51;
  const Scalar _tmp132 = fh1 * (-_tmp114 * _tmp33 + _tmp127 * _tmp26);
  const Scalar _tmp133 = _tmp55 * _tmp59;
  const Scalar _tmp134 = _tmp56 * _tmp59 + Scalar(1.0);
  const Scalar _tmp135 = _tmp16 * fh1;
  const Scalar _tmp136 = -_tmp127 * _tmp135 - Scalar(40.024799999999999) * _tmp30 - _tmp33 * fv1;
  const Scalar _tmp137 = _tmp114 * _tmp135 + Scalar(40.024799999999999) * _tmp20 + _tmp26 * fv1;
  const Scalar _tmp138 = _tmp51 * _tmp53;
  const Scalar _tmp139 = _tmp87 * _tmp95;
  const Scalar _tmp140 = _tmp91 * _tmp95;
  const Scalar _tmp141 = -_tmp125 * _tmp140 + Scalar(1.0);
  const Scalar _tmp142 = _tmp72 * _tmp77;
  const Scalar _tmp143 = -_tmp106 * _tmp140 - _tmp66;
  const Scalar _tmp144 = _tmp119 * _tmp140;
  const Scalar _tmp145 = _tmp119 * _tmp95;
  const Scalar _tmp146 = -_tmp115 * _tmp98 * (_tmp106 * _tmp139 + _tmp142 * _tmp143 + Scalar(1.0)) -
                         _tmp116 * _tmp98 * (-_tmp142 * _tmp144 + _tmp145 * _tmp87) -
                         _tmp128 * _tmp98 * (_tmp125 * _tmp139 + _tmp141 * _tmp142) -
                         _tmp132 * _tmp98 * (-_tmp129 * _tmp72 * _tmp92 + _tmp129 * _tmp87);
  const Scalar _tmp147 = Scalar(1.0) / (_tmp146);
  const Scalar _tmp148 = std::asinh(
      _tmp147 * (Scalar(1.0) * _tmp115 * (_tmp105 * _tmp59 - _tmp109 * _tmp110) +
                 Scalar(1.0) * _tmp116 * (-_tmp117 * _tmp59 - Scalar(1.0) * _tmp120 + Scalar(1.0)) +
                 Scalar(1.0) * _tmp128 * (-_tmp110 * _tmp126 + _tmp124 * _tmp59) +
                 Scalar(1.0) * _tmp132 * (-_tmp110 * _tmp131 + _tmp130) +
                 Scalar(1.0) * _tmp136 * (-_tmp110 * _tmp134 + _tmp133) +
                 Scalar(1.0) * _tmp137 * (_tmp138 * _tmp59 - _tmp59)));
  const Scalar _tmp149 = Scalar(1.4083112389913199) * _tmp146;
  const Scalar _tmp150 =
      Scalar(12100.0) * std::pow(Scalar(1 - Scalar(0.0090909090909090905) * _tmp67), Scalar(2)) +
      Scalar(15625.0) * std::pow(Scalar(1 - Scalar(0.0080000000000000002) * _tmp69), Scalar(2));
  const Scalar _tmp151 = _tmp137 * _tmp59;
  const Scalar _tmp152 = _tmp129 * _tmp132;
  const Scalar _tmp153 = _tmp115 * _tmp143 * _tmp77 - _tmp116 * _tmp144 * _tmp77 +
                         _tmp128 * _tmp141 * _tmp77 - _tmp152 * _tmp92;
  const Scalar _tmp154 = Scalar(1.0) / (_tmp153);
  const Scalar _tmp155 = std::asinh(
      _tmp154 * (_tmp109 * _tmp115 * _tmp53 + _tmp116 * _tmp120 + _tmp126 * _tmp128 * _tmp53 +
                 _tmp131 * _tmp132 * _tmp53 + _tmp134 * _tmp136 * _tmp53 - _tmp138 * _tmp151));
  const Scalar _tmp156 = Scalar(1.4083112389913199) * _tmp153;
  const Scalar _tmp157 =
      Scalar(12100.0) * std::pow(Scalar(-Scalar(0.0090909090909090905) * _tmp82 - 1), Scalar(2)) +
      Scalar(15625.0) * std::pow(Scalar(-Scalar(0.0080000000000000002) * _tmp84 - 1), Scalar(2));
  const Scalar _tmp158 =
      _tmp106 * _tmp115 * _tmp95 + _tmp116 * _tmp145 + _tmp125 * _tmp128 * _tmp95 + _tmp152;
  const Scalar _tmp159 = Scalar(1.0) / (_tmp158);
  const Scalar _tmp160 = std::asinh(_tmp159 * (-_tmp105 * _tmp115 * _tmp58 + _tmp116 * _tmp118 -
                                               _tmp124 * _tmp128 * _tmp58 - _tmp130 * _tmp132 -
                                               _tmp133 * _tmp136 + _tmp151));
  const Scalar _tmp161 = Scalar(1.4083112389913199) * _tmp158;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp38 *
          (-std::sinh(Scalar(1.0) * _tmp37) - std::sinh(Scalar(0.71007031138673404) * _tmp36 *
                                                        (-std::sqrt(_tmp35) - _tmp37 * _tmp38))) -
      Scalar(48.0) * std::sqrt(Scalar(
                         Scalar(0.00043402777777777775) * _tmp35 +
                         std::pow(Scalar(-Scalar(0.020833333333333332) * _tmp16 -
                                         Scalar(0.020833333333333332) * position_vector(2, 0) + 1),
                                  Scalar(2))));
  _res(1, 0) =
      _tmp149 * (-std::sinh(Scalar(1.0) * _tmp148) -
                 std::sinh(Scalar(0.71007031138673404) * _tmp147 *
                           (-_tmp148 * _tmp149 - std::sqrt(_tmp46)))) -
      Scalar(48.0) * std::sqrt(Scalar(
                         Scalar(0.00043402777777777775) * _tmp46 +
                         std::pow(Scalar(-Scalar(0.020833333333333332) * _tmp62 -
                                         Scalar(0.020833333333333332) * position_vector(2, 0) + 1),
                                  Scalar(2))));
  _res(2, 0) =
      _tmp156 * (-std::sinh(Scalar(1.0) * _tmp155) -
                 std::sinh(Scalar(0.71007031138673404) * _tmp154 *
                           (-std::sqrt(_tmp150) - _tmp155 * _tmp156))) -
      Scalar(48.0) * std::sqrt(Scalar(
                         Scalar(0.00043402777777777775) * _tmp150 +
                         std::pow(Scalar(-Scalar(0.020833333333333332) * _tmp74 -
                                         Scalar(0.020833333333333332) * position_vector(2, 0) + 1),
                                  Scalar(2))));
  _res(3, 0) =
      _tmp161 * (-std::sinh(Scalar(1.0) * _tmp160) -
                 std::sinh(Scalar(0.71007031138673404) * _tmp159 *
                           (-std::sqrt(_tmp157) - _tmp160 * _tmp161))) -
      Scalar(48.0) * std::sqrt(Scalar(
                         Scalar(0.00043402777777777775) * _tmp157 +
                         std::pow(Scalar(-Scalar(0.020833333333333332) * _tmp89 -
                                         Scalar(0.020833333333333332) * position_vector(2, 0) + 1),
                                  Scalar(2))));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
