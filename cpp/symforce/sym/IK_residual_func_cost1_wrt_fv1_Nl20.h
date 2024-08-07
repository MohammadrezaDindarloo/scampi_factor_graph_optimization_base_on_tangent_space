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
 * Symbolic function: IK_residual_func_cost1_wrt_fv1_Nl20
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFv1Nl20(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 608

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (193)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(1.0) * _tmp0 /
                       std::sqrt(Scalar(1 + std::pow(fv1, Scalar(2)) / std::pow(fh1, Scalar(2))));
  const Scalar _tmp3 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp4 = -2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp5 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp6 = 1 - 2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp7 = Scalar(0.20999999999999999) * _tmp4 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp8 = -_tmp7;
  const Scalar _tmp9 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp10 = 2 * _tmp9;
  const Scalar _tmp11 = _tmp10 * _tmp5;
  const Scalar _tmp12 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                        2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp13 = _tmp12 * _tmp3;
  const Scalar _tmp14 = _tmp11 + _tmp13;
  const Scalar _tmp15 = -Scalar(0.010999999999999999) * _tmp14;
  const Scalar _tmp16 = _tmp10 * _tmp3;
  const Scalar _tmp17 = _tmp12 * _tmp5;
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp16 - Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp19 = _tmp15 + _tmp18;
  const Scalar _tmp20 = _tmp19 + _tmp8;
  const Scalar _tmp21 = _tmp20 + position_vector(0, 0);
  const Scalar _tmp22 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp23 = Scalar(0.20999999999999999) * _tmp22 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp24 = Scalar(0.20999999999999999) * _tmp16 + Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp25 = 2 * _tmp3 * _tmp5;
  const Scalar _tmp26 = _tmp12 * _tmp9;
  const Scalar _tmp27 = _tmp25 - _tmp26;
  const Scalar _tmp28 = Scalar(0.010999999999999999) * _tmp27;
  const Scalar _tmp29 = -_tmp28;
  const Scalar _tmp30 = -_tmp24 + _tmp29;
  const Scalar _tmp31 = _tmp23 + _tmp30;
  const Scalar _tmp32 = _tmp31 + position_vector(1, 0);
  const Scalar _tmp33 = Scalar(1.4083112389913199) * fh1;
  const Scalar _tmp34 = Scalar(333.54000000000002) - fv1;
  const Scalar _tmp35 = Scalar(0.20999999999999999) * _tmp25 + Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp36 = -Scalar(0.010999999999999999) * _tmp22 -
                        Scalar(0.010999999999999999) * _tmp4 + Scalar(-0.010999999999999999);
  const Scalar _tmp37 = Scalar(0.20999999999999999) * _tmp11 - Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp38 = _tmp36 + _tmp37;
  const Scalar _tmp39 = _tmp35 + _tmp38;
  const Scalar _tmp40 = _tmp24 + _tmp29;
  const Scalar _tmp41 = _tmp23 + _tmp40;
  const Scalar _tmp42 = _tmp41 + position_vector(1, 0);
  const Scalar _tmp43 = _tmp42 + Scalar(-110.0);
  const Scalar _tmp44 = _tmp19 + _tmp7;
  const Scalar _tmp45 = _tmp44 + position_vector(0, 0);
  const Scalar _tmp46 = _tmp45 + Scalar(-125.0);
  const Scalar _tmp47 = std::pow(Scalar(std::pow(_tmp43, Scalar(2)) + std::pow(_tmp46, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp48 = _tmp46 * _tmp47;
  const Scalar _tmp49 = -_tmp35;
  const Scalar _tmp50 = _tmp38 + _tmp49;
  const Scalar _tmp51 = _tmp15 - _tmp18;
  const Scalar _tmp52 = _tmp51 + _tmp7;
  const Scalar _tmp53 = _tmp52 + position_vector(0, 0);
  const Scalar _tmp54 = _tmp53 + Scalar(-125.0);
  const Scalar _tmp55 = Scalar(1.0) / (_tmp54);
  const Scalar _tmp56 = -_tmp23;
  const Scalar _tmp57 = _tmp40 + _tmp56;
  const Scalar _tmp58 = _tmp57 + position_vector(1, 0);
  const Scalar _tmp59 = _tmp58 + Scalar(110.0);
  const Scalar _tmp60 = _tmp55 * _tmp59;
  const Scalar _tmp61 = _tmp50 * _tmp60;
  const Scalar _tmp62 = _tmp36 - _tmp37;
  const Scalar _tmp63 = _tmp49 + _tmp62;
  const Scalar _tmp64 = _tmp30 + _tmp56;
  const Scalar _tmp65 = _tmp64 + position_vector(1, 0);
  const Scalar _tmp66 = _tmp65 + Scalar(110.0);
  const Scalar _tmp67 = _tmp51 + _tmp8;
  const Scalar _tmp68 = _tmp67 + position_vector(0, 0);
  const Scalar _tmp69 = _tmp68 + Scalar(125.0);
  const Scalar _tmp70 = std::pow(Scalar(std::pow(_tmp66, Scalar(2)) + std::pow(_tmp69, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp71 = _tmp66 * _tmp70;
  const Scalar _tmp72 = _tmp69 * _tmp70;
  const Scalar _tmp73 = -_tmp61 * _tmp72 + _tmp63 * _tmp71;
  const Scalar _tmp74 = Scalar(1.0) / (_tmp60 * _tmp72 - _tmp71);
  const Scalar _tmp75 = _tmp43 * _tmp47;
  const Scalar _tmp76 = _tmp48 * _tmp60 - _tmp75;
  const Scalar _tmp77 = _tmp74 * _tmp76;
  const Scalar _tmp78 = _tmp39 * _tmp75 - _tmp48 * _tmp61 - _tmp73 * _tmp77;
  const Scalar _tmp79 = Scalar(1.0) * _tmp57;
  const Scalar _tmp80 = -_tmp79;
  const Scalar _tmp81 = Scalar(1.0) / (_tmp64 + _tmp80);
  const Scalar _tmp82 = Scalar(1.0) * _tmp52;
  const Scalar _tmp83 = _tmp81 * (-_tmp67 + _tmp82);
  const Scalar _tmp84 = _tmp50 * _tmp72 - _tmp63 * _tmp72;
  const Scalar _tmp85 = -_tmp39 * _tmp48 + _tmp48 * _tmp50 - _tmp77 * _tmp84 - _tmp78 * _tmp83;
  const Scalar _tmp86 = Scalar(1.0) / (_tmp85);
  const Scalar _tmp87 = _tmp79 * _tmp83 + _tmp82;
  const Scalar _tmp88 = 0;
  const Scalar _tmp89 = _tmp86 * _tmp88;
  const Scalar _tmp90 = _tmp72 * _tmp77;
  const Scalar _tmp91 =
      std::sqrt(Scalar(std::pow(_tmp54, Scalar(2)) + std::pow(_tmp59, Scalar(2))));
  const Scalar _tmp92 = _tmp55 * _tmp91;
  const Scalar _tmp93 = _tmp92 * (_tmp48 * _tmp89 - _tmp89 * _tmp90);
  const Scalar _tmp94 = _tmp60 * _tmp74;
  const Scalar _tmp95 = _tmp61 + _tmp73 * _tmp94;
  const Scalar _tmp96 = -_tmp50 - _tmp83 * _tmp95 + _tmp84 * _tmp94;
  const Scalar _tmp97 = Scalar(1.0) / (_tmp91);
  const Scalar _tmp98 = _tmp92 * (_tmp52 * _tmp59 * _tmp97 - _tmp54 * _tmp57 * _tmp97);
  const Scalar _tmp99 = _tmp64 * _tmp72 - _tmp67 * _tmp71 + _tmp72 * _tmp98;
  const Scalar _tmp100 = _tmp41 * _tmp48 - _tmp44 * _tmp75 + _tmp48 * _tmp98 - _tmp77 * _tmp99;
  const Scalar _tmp101 = _tmp100 * _tmp86;
  const Scalar _tmp102 = Scalar(1.0) / (_tmp100);
  const Scalar _tmp103 = _tmp102 * _tmp85;
  const Scalar _tmp104 = _tmp103 * (-_tmp101 * _tmp96 + _tmp94 * _tmp99 - _tmp98);
  const Scalar _tmp105 = _tmp104 + _tmp96;
  const Scalar _tmp106 = _tmp76 * _tmp86;
  const Scalar _tmp107 = -_tmp105 * _tmp106 - _tmp60;
  const Scalar _tmp108 = _tmp72 * _tmp74;
  const Scalar _tmp109 = _tmp48 * _tmp86;
  const Scalar _tmp110 = _tmp21 + Scalar(125.0);
  const Scalar _tmp111 = _tmp32 + Scalar(-110.0);
  const Scalar _tmp112 =
      std::pow(Scalar(std::pow(_tmp110, Scalar(2)) + std::pow(_tmp111, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp113 = _tmp110 * _tmp112;
  const Scalar _tmp114 = _tmp113 * fh1;
  const Scalar _tmp115 = Scalar(1.0) * _tmp74;
  const Scalar _tmp116 = _tmp115 * _tmp73;
  const Scalar _tmp117 = -_tmp115 * _tmp84 + _tmp116 * _tmp83;
  const Scalar _tmp118 = _tmp103 * (-_tmp101 * _tmp117 - _tmp115 * _tmp99);
  const Scalar _tmp119 = _tmp117 + _tmp118;
  const Scalar _tmp120 = -_tmp106 * _tmp119 + Scalar(1.0);
  const Scalar _tmp121 = _tmp111 * _tmp112;
  const Scalar _tmp122 = _tmp121 * fh1;
  const Scalar _tmp123 = Scalar(1.0) * _tmp102;
  const Scalar _tmp124 = fh1 * (-_tmp113 * _tmp31 + _tmp121 * _tmp20);
  const Scalar _tmp125 = -_tmp114 * _tmp92 * (_tmp105 * _tmp109 + _tmp107 * _tmp108 + Scalar(1.0)) -
                         _tmp122 * _tmp92 * (_tmp108 * _tmp120 + _tmp109 * _tmp119) -
                         _tmp124 * _tmp92 * (_tmp123 * _tmp48 - _tmp123 * _tmp90) - _tmp34 * _tmp93;
  const Scalar _tmp126 = Scalar(1.0) / (_tmp125);
  const Scalar _tmp127 = _tmp41 + _tmp80;
  const Scalar _tmp128 = _tmp127 * _tmp83;
  const Scalar _tmp129 = Scalar(1.0) / (-_tmp128 - _tmp44 + _tmp82);
  const Scalar _tmp130 = Scalar(1.0) * _tmp129;
  const Scalar _tmp131 = _tmp129 * _tmp87;
  const Scalar _tmp132 = _tmp78 * _tmp86;
  const Scalar _tmp133 = _tmp81 * (-_tmp127 * _tmp131 - _tmp132 * _tmp88 + _tmp80);
  const Scalar _tmp134 = -Scalar(1.0) * _tmp130 * _tmp87 - Scalar(1.0) * _tmp133 + Scalar(1.0);
  const Scalar _tmp135 = _tmp127 * _tmp129;
  const Scalar _tmp136 = -_tmp116 + _tmp118 * _tmp135 - _tmp119 * _tmp132;
  const Scalar _tmp137 = Scalar(1.0) * _tmp81;
  const Scalar _tmp138 = _tmp104 * _tmp135 - _tmp105 * _tmp132 + _tmp95;
  const Scalar _tmp139 = fh1 * (_tmp35 + _tmp62);
  const Scalar _tmp140 = -_tmp121 * _tmp139 - Scalar(40.024799999999999) * _tmp27 - _tmp31 * fv1;
  const Scalar _tmp141 = _tmp130 * _tmp83;
  const Scalar _tmp142 = _tmp128 * _tmp130 + Scalar(1.0);
  const Scalar _tmp143 = -Scalar(1.0) * _tmp137 * _tmp142 + Scalar(1.0) * _tmp141;
  const Scalar _tmp144 = _tmp113 * _tmp139 + Scalar(40.024799999999999) * _tmp14 + _tmp20 * fv1;
  const Scalar _tmp145 = _tmp127 * _tmp81;
  const Scalar _tmp146 = Scalar(1.0) * _tmp130 * _tmp145 - Scalar(1.0) * _tmp130;
  const Scalar _tmp147 = _tmp103 * _tmp130;
  const Scalar _tmp148 = -_tmp123 * _tmp78 + _tmp127 * _tmp147;
  const Scalar _tmp149 = Scalar(1.0) * _tmp114 * (_tmp104 * _tmp130 - _tmp137 * _tmp138) +
                         Scalar(1.0) * _tmp122 * (_tmp118 * _tmp130 - _tmp136 * _tmp137) +
                         Scalar(1.0) * _tmp124 * (-_tmp137 * _tmp148 + _tmp147) + _tmp134 * _tmp34 +
                         _tmp140 * _tmp143 + _tmp144 * _tmp146;
  const Scalar _tmp150 = std::asinh(_tmp126 * _tmp149);
  const Scalar _tmp151 = Scalar(1.0) * _tmp150;
  const Scalar _tmp152 = _tmp24 + _tmp28 + _tmp56;
  const Scalar _tmp153 = std::pow(_tmp125, Scalar(-2));
  const Scalar _tmp154 = _tmp153 * _tmp93;
  const Scalar _tmp155 =
      (_tmp126 * (-_tmp134 + _tmp143 * _tmp152 + _tmp146 * _tmp20) - _tmp149 * _tmp154) /
      std::sqrt(Scalar(std::pow(_tmp149, Scalar(2)) * _tmp153 + 1));
  const Scalar _tmp156 = Scalar(1.4083112389913199) * _tmp125;
  const Scalar _tmp157 =
      -_tmp150 * _tmp156 -
      Scalar(125.0) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.0080000000000000002) * _tmp53), Scalar(2)) +
                     Scalar(0.77439999999999998) *
                         std::pow(Scalar(-Scalar(0.0090909090909090905) * _tmp58 - 1), Scalar(2))));
  const Scalar _tmp158 = Scalar(1.4083112389913199) * _tmp93;
  const Scalar _tmp159 = Scalar(0.71007031138673404) * _tmp126;
  const Scalar _tmp160 = _tmp157 * _tmp159;
  const Scalar _tmp161 = _tmp123 * _tmp124;
  const Scalar _tmp162 = _tmp34 * _tmp89;
  const Scalar _tmp163 =
      _tmp107 * _tmp114 * _tmp74 + _tmp120 * _tmp122 * _tmp74 - _tmp161 * _tmp77 - _tmp162 * _tmp77;
  const Scalar _tmp164 = Scalar(1.0) / (_tmp163);
  const Scalar _tmp165 = _tmp81 * fh1;
  const Scalar _tmp166 = _tmp142 * _tmp81;
  const Scalar _tmp167 = _tmp130 * _tmp144;
  const Scalar _tmp168 = _tmp113 * _tmp138 * _tmp165 + _tmp121 * _tmp136 * _tmp165 +
                         _tmp124 * _tmp148 * _tmp81 + _tmp133 * _tmp34 + _tmp140 * _tmp166 -
                         _tmp145 * _tmp167;
  const Scalar _tmp169 = std::asinh(_tmp164 * _tmp168);
  const Scalar _tmp170 = Scalar(1.4083112389913199) * _tmp163;
  const Scalar _tmp171 =
      -_tmp169 * _tmp170 -
      Scalar(125.0) *
          std::sqrt(
              Scalar(Scalar(0.77439999999999998) *
                         std::pow(Scalar(-Scalar(0.0090909090909090905) * _tmp65 - 1), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.0080000000000000002) * _tmp68 - 1), Scalar(2))));
  const Scalar _tmp172 = Scalar(0.71007031138673404) * _tmp164;
  const Scalar _tmp173 = _tmp171 * _tmp172;
  const Scalar _tmp174 = Scalar(1.0) * _tmp169;
  const Scalar _tmp175 = Scalar(1.4083112389913199) * _tmp89;
  const Scalar _tmp176 = _tmp175 * _tmp77;
  const Scalar _tmp177 = std::pow(_tmp163, Scalar(-2));
  const Scalar _tmp178 = _tmp177 * _tmp77 * _tmp89;
  const Scalar _tmp179 = _tmp130 * _tmp20;
  const Scalar _tmp180 =
      (_tmp164 * (-_tmp133 - _tmp145 * _tmp179 + _tmp152 * _tmp166) - _tmp168 * _tmp178) /
      std::sqrt(Scalar(std::pow(_tmp168, Scalar(2)) * _tmp177 + 1));
  const Scalar _tmp181 = -_tmp104 * _tmp114 * _tmp129 - _tmp118 * _tmp122 * _tmp129 -
                         _tmp124 * _tmp147 + _tmp131 * _tmp34 - _tmp140 * _tmp141 + _tmp167;
  const Scalar _tmp182 =
      _tmp105 * _tmp114 * _tmp86 + _tmp119 * _tmp122 * _tmp86 + _tmp161 + _tmp162;
  const Scalar _tmp183 = Scalar(1.0) / (_tmp182);
  const Scalar _tmp184 = std::asinh(_tmp181 * _tmp183);
  const Scalar _tmp185 = Scalar(1.4083112389913199) * _tmp182;
  const Scalar _tmp186 =
      -_tmp184 * _tmp185 -
      Scalar(125.0) *
          std::sqrt(
              Scalar(Scalar(0.77439999999999998) *
                         std::pow(Scalar(1 - Scalar(0.0090909090909090905) * _tmp42), Scalar(2)) +
                     std::pow(Scalar(1 - Scalar(0.0080000000000000002) * _tmp45), Scalar(2))));
  const Scalar _tmp187 = Scalar(0.71007031138673404) * _tmp183;
  const Scalar _tmp188 = _tmp186 * _tmp187;
  const Scalar _tmp189 = Scalar(1.0) * _tmp184;
  const Scalar _tmp190 = std::pow(_tmp182, Scalar(-2));
  const Scalar _tmp191 = _tmp190 * _tmp89;
  const Scalar _tmp192 = (_tmp181 * _tmp191 + _tmp183 * (-_tmp131 - _tmp141 * _tmp152 + _tmp179)) /
                         std::sqrt(Scalar(std::pow(_tmp181, Scalar(2)) * _tmp190 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -_tmp33 *
      (_tmp2 * std::sinh(Scalar(1.0) * _tmp1) +
       _tmp2 * std::sinh(Scalar(0.71007031138673404) * _tmp0 *
                         (-_tmp1 * _tmp33 -
                          Scalar(125.0) *
                              std::sqrt(Scalar(
                                  Scalar(0.77439999999999998) *
                                      std::pow(Scalar(1 - Scalar(0.0090909090909090905) * _tmp32),
                                               Scalar(2)) +
                                  std::pow(Scalar(-Scalar(0.0080000000000000002) * _tmp21 - 1),
                                           Scalar(2)))))));
  _res(1, 0) =
      -_tmp156 *
          (-Scalar(34.083374946563197) * _tmp154 + Scalar(1.0) * _tmp155 * std::sinh(_tmp151) -
           (-Scalar(0.71007031138673404) * _tmp154 * _tmp157 +
            _tmp159 * (-_tmp150 * _tmp158 - _tmp155 * _tmp156)) *
               std::sinh(_tmp160)) -
      _tmp158 * (Scalar(34.083374946563197) * _tmp126 + std::cosh(_tmp151) - std::cosh(_tmp160));
  _res(2, 0) =
      -_tmp170 *
          (-Scalar(34.083374946563197) * _tmp178 + Scalar(1.0) * _tmp180 * std::sinh(_tmp174) -
           (-Scalar(0.71007031138673404) * _tmp171 * _tmp178 +
            _tmp172 * (-_tmp169 * _tmp176 - _tmp170 * _tmp180)) *
               std::sinh(_tmp173)) -
      _tmp176 * (Scalar(34.083374946563197) * _tmp164 - std::cosh(_tmp173) + std::cosh(_tmp174));
  _res(3, 0) =
      _tmp175 * (Scalar(34.083374946563197) * _tmp183 - std::cosh(_tmp188) + std::cosh(_tmp189)) -
      _tmp185 * (Scalar(34.083374946563197) * _tmp191 + Scalar(1.0) * _tmp192 * std::sinh(_tmp189) -
                 (Scalar(0.71007031138673404) * _tmp186 * _tmp191 +
                  _tmp187 * (_tmp175 * _tmp184 - _tmp185 * _tmp192)) *
                     std::sinh(_tmp188));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
