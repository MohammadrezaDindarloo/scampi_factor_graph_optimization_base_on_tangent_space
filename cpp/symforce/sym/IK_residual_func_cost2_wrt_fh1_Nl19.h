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
 * Symbolic function: IK_residual_func_cost2_wrt_fh1_Nl19
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFh1Nl19(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 640

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (214)
  const Scalar _tmp0 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp1 = -2 * std::pow(_tmp0, Scalar(2));
  const Scalar _tmp2 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp3 = 1 - 2 * std::pow(_tmp2, Scalar(2));
  const Scalar _tmp4 = Scalar(0.20999999999999999) * _tmp1 + Scalar(0.20999999999999999) * _tmp3;
  const Scalar _tmp5 = -_tmp4;
  const Scalar _tmp6 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp7 = 2 * _tmp2 * _tmp6;
  const Scalar _tmp8 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                       2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp9 = _tmp0 * _tmp8;
  const Scalar _tmp10 = _tmp7 + _tmp9;
  const Scalar _tmp11 = -Scalar(0.010999999999999999) * _tmp10;
  const Scalar _tmp12 = 2 * _tmp0;
  const Scalar _tmp13 = _tmp12 * _tmp6;
  const Scalar _tmp14 = _tmp2 * _tmp8;
  const Scalar _tmp15 = Scalar(0.20999999999999999) * _tmp13 - Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp16 = _tmp11 + _tmp15;
  const Scalar _tmp17 = _tmp16 + _tmp5;
  const Scalar _tmp18 = _tmp17 + position_vector(0, 0);
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp14;
  const Scalar _tmp20 = -_tmp19;
  const Scalar _tmp21 = _tmp12 * _tmp2;
  const Scalar _tmp22 = _tmp6 * _tmp8;
  const Scalar _tmp23 = _tmp21 - _tmp22;
  const Scalar _tmp24 = -Scalar(0.010999999999999999) * _tmp23;
  const Scalar _tmp25 = -2 * std::pow(_tmp6, Scalar(2));
  const Scalar _tmp26 = Scalar(0.20999999999999999) * _tmp25 + Scalar(0.20999999999999999) * _tmp3;
  const Scalar _tmp27 = _tmp24 + _tmp26;
  const Scalar _tmp28 = _tmp20 + _tmp27;
  const Scalar _tmp29 = _tmp28 + position_vector(1, 0);
  const Scalar _tmp30 = Scalar(1.0) / (fh1);
  const Scalar _tmp31 = _tmp30 * fv1;
  const Scalar _tmp32 = std::asinh(_tmp31);
  const Scalar _tmp33 = Scalar(1.4083112389913199) * _tmp32;
  const Scalar _tmp34 =
      -_tmp33 * fh1 -
      Scalar(125.0) *
          std::sqrt(
              Scalar(Scalar(0.77439999999999998) *
                         std::pow(Scalar(1 - Scalar(0.0090909090909090905) * _tmp29), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.0080000000000000002) * _tmp18 - 1), Scalar(2))));
  const Scalar _tmp35 = Scalar(0.71007031138673404) * _tmp30;
  const Scalar _tmp36 = _tmp34 * _tmp35;
  const Scalar _tmp37 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp38 =
      std::pow(Scalar(_tmp37 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp39 = Scalar(1.0) * _tmp32;
  const Scalar _tmp40 = _tmp18 + Scalar(125.0);
  const Scalar _tmp41 = _tmp29 + Scalar(-110.0);
  const Scalar _tmp42 = std::pow(Scalar(std::pow(_tmp40, Scalar(2)) + std::pow(_tmp41, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp43 = _tmp41 * _tmp42;
  const Scalar _tmp44 = _tmp40 * _tmp42;
  const Scalar _tmp45 = _tmp17 * _tmp43 - _tmp28 * _tmp44;
  const Scalar _tmp46 = _tmp24 - _tmp26;
  const Scalar _tmp47 = _tmp20 + _tmp46;
  const Scalar _tmp48 = Scalar(1.0) * _tmp47;
  const Scalar _tmp49 = -_tmp48;
  const Scalar _tmp50 = _tmp19 + _tmp46;
  const Scalar _tmp51 = _tmp49 + _tmp50;
  const Scalar _tmp52 = _tmp19 + _tmp27;
  const Scalar _tmp53 = Scalar(1.0) / (_tmp49 + _tmp52);
  const Scalar _tmp54 = _tmp11 - _tmp15;
  const Scalar _tmp55 = _tmp5 + _tmp54;
  const Scalar _tmp56 = Scalar(1.0) * _tmp55;
  const Scalar _tmp57 = _tmp16 + _tmp4;
  const Scalar _tmp58 = _tmp56 - _tmp57;
  const Scalar _tmp59 = _tmp53 * _tmp58;
  const Scalar _tmp60 = _tmp51 * _tmp59;
  const Scalar _tmp61 = _tmp4 + _tmp54;
  const Scalar _tmp62 = Scalar(1.0) / (_tmp56 - _tmp60 - _tmp61);
  const Scalar _tmp63 = Scalar(1.0) * _tmp62;
  const Scalar _tmp64 = _tmp52 + position_vector(1, 0);
  const Scalar _tmp65 = _tmp64 + Scalar(-110.0);
  const Scalar _tmp66 = _tmp57 + position_vector(0, 0);
  const Scalar _tmp67 = _tmp66 + Scalar(-125.0);
  const Scalar _tmp68 = std::pow(Scalar(std::pow(_tmp65, Scalar(2)) + std::pow(_tmp67, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp69 = _tmp67 * _tmp68;
  const Scalar _tmp70 = _tmp65 * _tmp68;
  const Scalar _tmp71 = _tmp47 + position_vector(1, 0);
  const Scalar _tmp72 = _tmp71 + Scalar(110.0);
  const Scalar _tmp73 = _tmp55 + position_vector(0, 0);
  const Scalar _tmp74 = _tmp73 + Scalar(125.0);
  const Scalar _tmp75 =
      std::sqrt(Scalar(std::pow(_tmp72, Scalar(2)) + std::pow(_tmp74, Scalar(2))));
  const Scalar _tmp76 = Scalar(1.0) / (_tmp75);
  const Scalar _tmp77 = Scalar(1.0) / (_tmp74);
  const Scalar _tmp78 = _tmp75 * _tmp77;
  const Scalar _tmp79 = _tmp78 * (-_tmp47 * _tmp74 * _tmp76 + _tmp55 * _tmp72 * _tmp76);
  const Scalar _tmp80 = _tmp52 * _tmp69 - _tmp57 * _tmp70 + _tmp69 * _tmp79;
  const Scalar _tmp81 = _tmp72 * _tmp77;
  const Scalar _tmp82 = Scalar(1.0) / (_tmp69 * _tmp81 - _tmp70);
  const Scalar _tmp83 = _tmp50 + position_vector(1, 0);
  const Scalar _tmp84 = _tmp83 + Scalar(110.0);
  const Scalar _tmp85 = _tmp61 + position_vector(0, 0);
  const Scalar _tmp86 = _tmp85 + Scalar(-125.0);
  const Scalar _tmp87 = std::pow(Scalar(std::pow(_tmp84, Scalar(2)) + std::pow(_tmp86, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp88 = _tmp84 * _tmp87;
  const Scalar _tmp89 = _tmp86 * _tmp87;
  const Scalar _tmp90 = _tmp81 * _tmp89 - _tmp88;
  const Scalar _tmp91 = _tmp82 * _tmp90;
  const Scalar _tmp92 = _tmp50 * _tmp89 - _tmp61 * _tmp88 + _tmp79 * _tmp89 - _tmp80 * _tmp91;
  const Scalar _tmp93 = Scalar(1.0) / (_tmp92);
  const Scalar _tmp94 = Scalar(0.20999999999999999) * _tmp7 - Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp95 = -_tmp94;
  const Scalar _tmp96 = -Scalar(0.010999999999999999) * _tmp1 -
                        Scalar(0.010999999999999999) * _tmp25 + Scalar(-0.010999999999999999);
  const Scalar _tmp97 = Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999) * _tmp22;
  const Scalar _tmp98 = _tmp96 - _tmp97;
  const Scalar _tmp99 = _tmp95 + _tmp98;
  const Scalar _tmp100 = _tmp96 + _tmp97;
  const Scalar _tmp101 = _tmp100 + _tmp94;
  const Scalar _tmp102 = -_tmp101 * _tmp69 + _tmp69 * _tmp99;
  const Scalar _tmp103 = _tmp94 + _tmp98;
  const Scalar _tmp104 = _tmp81 * _tmp99;
  const Scalar _tmp105 = _tmp101 * _tmp70 - _tmp104 * _tmp69;
  const Scalar _tmp106 = _tmp103 * _tmp88 - _tmp104 * _tmp89 - _tmp105 * _tmp91;
  const Scalar _tmp107 = -_tmp102 * _tmp91 - _tmp103 * _tmp89 - _tmp106 * _tmp59 + _tmp89 * _tmp99;
  const Scalar _tmp108 = _tmp107 * _tmp93;
  const Scalar _tmp109 = _tmp108 * _tmp63;
  const Scalar _tmp110 = Scalar(1.0) * _tmp93;
  const Scalar _tmp111 = -_tmp106 * _tmp110 + _tmp109 * _tmp51;
  const Scalar _tmp112 = Scalar(1.0) * _tmp53;
  const Scalar _tmp113 = Scalar(1.0) * _tmp45 * (_tmp109 - _tmp111 * _tmp112);
  const Scalar _tmp114 = _tmp100 + _tmp95;
  const Scalar _tmp115 = _tmp114 * fh1;
  const Scalar _tmp116 = -_tmp115 * _tmp43 - Scalar(40.024799999999999) * _tmp23 - _tmp28 * fv1;
  const Scalar _tmp117 = _tmp60 * _tmp63 + Scalar(1.0);
  const Scalar _tmp118 = _tmp59 * _tmp63;
  const Scalar _tmp119 = -Scalar(1.0) * _tmp112 * _tmp117 + Scalar(1.0) * _tmp118;
  const Scalar _tmp120 = Scalar(40.024799999999999) * _tmp10 + _tmp115 * _tmp44 + _tmp17 * fv1;
  const Scalar _tmp121 = _tmp51 * _tmp53;
  const Scalar _tmp122 = _tmp121 * _tmp63;
  const Scalar _tmp123 = Scalar(1.0) * _tmp122 - Scalar(1.0) * _tmp63;
  const Scalar _tmp124 = Scalar(1.0) * _tmp82;
  const Scalar _tmp125 = -_tmp102 * _tmp124 + _tmp105 * _tmp112 * _tmp58 * _tmp82;
  const Scalar _tmp126 = Scalar(1.0) / (_tmp107);
  const Scalar _tmp127 = _tmp126 * _tmp92;
  const Scalar _tmp128 = _tmp108 * (-_tmp124 * _tmp80 - _tmp125 * _tmp127);
  const Scalar _tmp129 = _tmp125 + _tmp128;
  const Scalar _tmp130 = _tmp106 * _tmp126;
  const Scalar _tmp131 = _tmp51 * _tmp62;
  const Scalar _tmp132 = -_tmp105 * _tmp124 + _tmp128 * _tmp131 - _tmp129 * _tmp130;
  const Scalar _tmp133 = Scalar(1.0) * _tmp43 * (-_tmp112 * _tmp132 + _tmp128 * _tmp63);
  const Scalar _tmp134 = Scalar(333.54000000000002) - fv1;
  const Scalar _tmp135 = _tmp48 * _tmp59 + _tmp56;
  const Scalar _tmp136 = 0;
  const Scalar _tmp137 = _tmp135 * _tmp62;
  const Scalar _tmp138 = -_tmp106 * _tmp136 - _tmp137 * _tmp51 + _tmp49;
  const Scalar _tmp139 = _tmp81 * _tmp82;
  const Scalar _tmp140 = _tmp104 + _tmp105 * _tmp139;
  const Scalar _tmp141 = _tmp102 * _tmp139 - _tmp140 * _tmp59 - _tmp99;
  const Scalar _tmp142 = _tmp108 * (-_tmp127 * _tmp141 + _tmp139 * _tmp80 - _tmp79);
  const Scalar _tmp143 = _tmp141 + _tmp142;
  const Scalar _tmp144 = -_tmp130 * _tmp143 + _tmp131 * _tmp142 + _tmp140;
  const Scalar _tmp145 = Scalar(1.0) * _tmp44 * (-_tmp112 * _tmp144 + _tmp142 * _tmp63);
  const Scalar _tmp146 =
      _tmp113 * fh1 + _tmp116 * _tmp119 + _tmp120 * _tmp123 + _tmp133 * fh1 +
      Scalar(1.0) * _tmp134 * (-_tmp112 * _tmp138 - _tmp135 * _tmp63 + Scalar(1.0)) + _tmp145 * fh1;
  const Scalar _tmp147 = _tmp126 * _tmp129;
  const Scalar _tmp148 = _tmp126 * _tmp90;
  const Scalar _tmp149 = _tmp82 * (-_tmp129 * _tmp148 + Scalar(1.0));
  const Scalar _tmp150 = _tmp43 * _tmp78 * (_tmp147 * _tmp89 + _tmp149 * _tmp69);
  const Scalar _tmp151 = _tmp69 * _tmp91;
  const Scalar _tmp152 = _tmp45 * _tmp78 * (-_tmp110 * _tmp151 + _tmp110 * _tmp89);
  const Scalar _tmp153 = _tmp126 * _tmp143;
  const Scalar _tmp154 = _tmp82 * (-_tmp143 * _tmp148 - _tmp81);
  const Scalar _tmp155 = _tmp44 * _tmp78 * (_tmp153 * _tmp89 + _tmp154 * _tmp69 + Scalar(1.0));
  const Scalar _tmp156 = -_tmp134 * _tmp78 * (-_tmp136 * _tmp151 + _tmp136 * _tmp89) -
                         _tmp150 * fh1 - _tmp152 * fh1 - _tmp155 * fh1;
  const Scalar _tmp157 = Scalar(1.0) / (_tmp156);
  const Scalar _tmp158 = std::asinh(_tmp146 * _tmp157);
  const Scalar _tmp159 = Scalar(1.0) * _tmp158;
  const Scalar _tmp160 = Scalar(1.4083112389913199) * _tmp156;
  const Scalar _tmp161 =
      -_tmp158 * _tmp160 -
      Scalar(125.0) *
          std::sqrt(
              Scalar(Scalar(0.77439999999999998) *
                         std::pow(Scalar(-Scalar(0.0090909090909090905) * _tmp71 - 1), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.0080000000000000002) * _tmp73 - 1), Scalar(2))));
  const Scalar _tmp162 = Scalar(0.71007031138673404) * _tmp157;
  const Scalar _tmp163 = _tmp161 * _tmp162;
  const Scalar _tmp164 = -_tmp150 - _tmp152 - _tmp155;
  const Scalar _tmp165 = Scalar(1.4083112389913199) * _tmp164;
  const Scalar _tmp166 = _tmp114 * _tmp44;
  const Scalar _tmp167 = _tmp114 * _tmp43;
  const Scalar _tmp168 = std::pow(_tmp156, Scalar(-2));
  const Scalar _tmp169 = _tmp164 * _tmp168;
  const Scalar _tmp170 = (-_tmp146 * _tmp169 + _tmp157 * (_tmp113 - _tmp119 * _tmp167 +
                                                          _tmp123 * _tmp166 + _tmp133 + _tmp145)) /
                         std::sqrt(Scalar(std::pow(_tmp146, Scalar(2)) * _tmp168 + 1));
  const Scalar _tmp171 = _tmp154 * _tmp44;
  const Scalar _tmp172 = _tmp110 * _tmp45;
  const Scalar _tmp173 = _tmp172 * fh1;
  const Scalar _tmp174 = _tmp149 * _tmp43;
  const Scalar _tmp175 = _tmp134 * _tmp136;
  const Scalar _tmp176 = _tmp171 * fh1 - _tmp173 * _tmp91 + _tmp174 * fh1 - _tmp175 * _tmp91;
  const Scalar _tmp177 = Scalar(1.0) / (_tmp176);
  const Scalar _tmp178 = _tmp132 * _tmp43 * _tmp53;
  const Scalar _tmp179 = _tmp117 * _tmp53;
  const Scalar _tmp180 = _tmp120 * _tmp63;
  const Scalar _tmp181 = _tmp111 * _tmp45 * _tmp53;
  const Scalar _tmp182 = _tmp144 * _tmp44 * _tmp53;
  const Scalar _tmp183 = _tmp116 * _tmp179 - _tmp121 * _tmp180 + _tmp134 * _tmp138 * _tmp53 +
                         _tmp178 * fh1 + _tmp181 * fh1 + _tmp182 * fh1;
  const Scalar _tmp184 = std::asinh(_tmp177 * _tmp183);
  const Scalar _tmp185 = Scalar(1.0) * _tmp184;
  const Scalar _tmp186 = std::pow(_tmp176, Scalar(-2));
  const Scalar _tmp187 = _tmp171 - _tmp172 * _tmp91 + _tmp174;
  const Scalar _tmp188 = _tmp186 * _tmp187;
  const Scalar _tmp189 =
      (_tmp177 * (-_tmp122 * _tmp166 - _tmp167 * _tmp179 + _tmp178 + _tmp181 + _tmp182) -
       _tmp183 * _tmp188) /
      std::sqrt(Scalar(std::pow(_tmp183, Scalar(2)) * _tmp186 + 1));
  const Scalar _tmp190 = Scalar(1.4083112389913199) * _tmp176;
  const Scalar _tmp191 =
      -_tmp184 * _tmp190 -
      Scalar(125.0) *
          std::sqrt(
              Scalar(Scalar(0.77439999999999998) *
                         std::pow(Scalar(1 - Scalar(0.0090909090909090905) * _tmp64), Scalar(2)) +
                     std::pow(Scalar(1 - Scalar(0.0080000000000000002) * _tmp66), Scalar(2))));
  const Scalar _tmp192 = Scalar(0.71007031138673404) * _tmp177;
  const Scalar _tmp193 = _tmp191 * _tmp192;
  const Scalar _tmp194 = Scalar(1.4083112389913199) * _tmp187;
  const Scalar _tmp195 = _tmp147 * _tmp43;
  const Scalar _tmp196 = _tmp153 * _tmp44;
  const Scalar _tmp197 = _tmp173 + _tmp175 + _tmp195 * fh1 + _tmp196 * fh1;
  const Scalar _tmp198 = Scalar(1.0) / (_tmp197);
  const Scalar _tmp199 = _tmp128 * _tmp43 * _tmp62;
  const Scalar _tmp200 = _tmp109 * _tmp45;
  const Scalar _tmp201 = _tmp142 * _tmp44 * _tmp62;
  const Scalar _tmp202 = -_tmp116 * _tmp118 + _tmp134 * _tmp137 + _tmp180 - _tmp199 * fh1 -
                         _tmp200 * fh1 - _tmp201 * fh1;
  const Scalar _tmp203 = std::asinh(_tmp198 * _tmp202);
  const Scalar _tmp204 = Scalar(1.0) * _tmp203;
  const Scalar _tmp205 = std::pow(_tmp197, Scalar(-2));
  const Scalar _tmp206 = _tmp172 + _tmp195 + _tmp196;
  const Scalar _tmp207 = _tmp205 * _tmp206;
  const Scalar _tmp208 =
      (_tmp198 * (_tmp118 * _tmp167 + _tmp166 * _tmp63 - _tmp199 - _tmp200 - _tmp201) -
       _tmp202 * _tmp207) /
      std::sqrt(Scalar(std::pow(_tmp202, Scalar(2)) * _tmp205 + 1));
  const Scalar _tmp209 = Scalar(1.4083112389913199) * _tmp197;
  const Scalar _tmp210 =
      -_tmp203 * _tmp209 -
      Scalar(125.0) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.0080000000000000002) * _tmp85), Scalar(2)) +
                     Scalar(0.77439999999999998) *
                         std::pow(Scalar(-Scalar(0.0090909090909090905) * _tmp83 - 1), Scalar(2))));
  const Scalar _tmp211 = Scalar(0.71007031138673404) * _tmp198;
  const Scalar _tmp212 = _tmp210 * _tmp211;
  const Scalar _tmp213 = Scalar(1.4083112389913199) * _tmp206;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = Scalar(1.4083112389913199) * fh1 *
                   (Scalar(1.0) * _tmp37 * _tmp38 * fv1 * std::cosh(_tmp39) -
                    (-Scalar(0.71007031138673404) * _tmp34 * _tmp37 +
                     _tmp35 * (Scalar(1.4083112389913199) * _tmp31 * _tmp38 - _tmp33)) *
                        std::cosh(_tmp36)) -
               Scalar(1.4083112389913199) * std::sinh(_tmp36) -
               Scalar(1.4083112389913199) * std::sinh(_tmp39);
  _res(1, 0) = _tmp160 * (-Scalar(1.0) * _tmp170 * std::cosh(_tmp159) -
                          (-Scalar(0.71007031138673404) * _tmp161 * _tmp169 +
                           _tmp162 * (-_tmp158 * _tmp165 - _tmp160 * _tmp170)) *
                              std::cosh(_tmp163)) +
               _tmp165 * (-std::sinh(_tmp159) - std::sinh(_tmp163));
  _res(2, 0) = _tmp190 * (-Scalar(1.0) * _tmp189 * std::cosh(_tmp185) -
                          (-Scalar(0.71007031138673404) * _tmp188 * _tmp191 +
                           _tmp192 * (-_tmp184 * _tmp194 - _tmp189 * _tmp190)) *
                              std::cosh(_tmp193)) +
               _tmp194 * (-std::sinh(_tmp185) - std::sinh(_tmp193));
  _res(3, 0) = _tmp209 * (-Scalar(1.0) * _tmp208 * std::cosh(_tmp204) -
                          (-Scalar(0.71007031138673404) * _tmp207 * _tmp210 +
                           _tmp211 * (-_tmp203 * _tmp213 - _tmp208 * _tmp209)) *
                              std::cosh(_tmp212)) +
               _tmp213 * (-std::sinh(_tmp204) - std::sinh(_tmp212));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
