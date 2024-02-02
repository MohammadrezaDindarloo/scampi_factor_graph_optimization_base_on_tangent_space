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
 * Symbolic function: IK_residual_func_cost1_wrt_fv1_Nl22
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFv1Nl22(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 608

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (192)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(1.0) * _tmp0 /
                       std::sqrt(Scalar(1 + std::pow(fv1, Scalar(2)) / std::pow(fh1, Scalar(2))));
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
  const Scalar _tmp18 = -_tmp12 + _tmp17;
  const Scalar _tmp19 = _tmp18 + _tmp9;
  const Scalar _tmp20 = _tmp19 + position_vector(0, 0);
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp5 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp22 = -_tmp21;
  const Scalar _tmp23 = _tmp13 * _tmp3;
  const Scalar _tmp24 = _tmp4 * _tmp7;
  const Scalar _tmp25 = _tmp23 - _tmp24;
  const Scalar _tmp26 = Scalar(0.010999999999999999) * _tmp25;
  const Scalar _tmp27 = -_tmp26;
  const Scalar _tmp28 = -2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp29 = Scalar(0.20999999999999999) * _tmp10 +
                        Scalar(0.20999999999999999) * _tmp28 + Scalar(0.20999999999999999);
  const Scalar _tmp30 = _tmp27 + _tmp29;
  const Scalar _tmp31 = _tmp22 + _tmp30;
  const Scalar _tmp32 = _tmp31 + position_vector(1, 0);
  const Scalar _tmp33 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp34 = -_tmp9;
  const Scalar _tmp35 = _tmp18 + _tmp34;
  const Scalar _tmp36 = -_tmp29;
  const Scalar _tmp37 = _tmp27 + _tmp36;
  const Scalar _tmp38 = _tmp22 + _tmp37;
  const Scalar _tmp39 = _tmp38 + position_vector(1, 0);
  const Scalar _tmp40 = _tmp39 + Scalar(8.3196563700000006);
  const Scalar _tmp41 = _tmp35 + position_vector(0, 0);
  const Scalar _tmp42 = _tmp41 + Scalar(1.9874742000000001);
  const Scalar _tmp43 = std::pow(Scalar(std::pow(_tmp40, Scalar(2)) + std::pow(_tmp42, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp44 = _tmp40 * _tmp43;
  const Scalar _tmp45 = _tmp42 * _tmp43;
  const Scalar _tmp46 = _tmp21 + _tmp30;
  const Scalar _tmp47 = _tmp46 + position_vector(1, 0);
  const Scalar _tmp48 = _tmp47 + Scalar(-4.7752063900000001);
  const Scalar _tmp49 = _tmp12 + _tmp17;
  const Scalar _tmp50 = _tmp49 + _tmp9;
  const Scalar _tmp51 = _tmp50 + position_vector(0, 0);
  const Scalar _tmp52 = _tmp51 + Scalar(-2.71799795);
  const Scalar _tmp53 =
      std::sqrt(Scalar(std::pow(_tmp48, Scalar(2)) + std::pow(_tmp52, Scalar(2))));
  const Scalar _tmp54 = Scalar(1.0) / (_tmp53);
  const Scalar _tmp55 = Scalar(1.0) / (_tmp52);
  const Scalar _tmp56 = _tmp53 * _tmp55;
  const Scalar _tmp57 = _tmp56 * (-_tmp46 * _tmp52 * _tmp54 + _tmp48 * _tmp50 * _tmp54);
  const Scalar _tmp58 = -_tmp35 * _tmp44 + _tmp38 * _tmp45 + _tmp45 * _tmp57;
  const Scalar _tmp59 = _tmp48 * _tmp55;
  const Scalar _tmp60 = Scalar(1.0) / (-_tmp44 + _tmp45 * _tmp59);
  const Scalar _tmp61 = Scalar(1.0) * _tmp60;
  const Scalar _tmp62 = Scalar(1.0) * _tmp50;
  const Scalar _tmp63 = -_tmp35 + _tmp62;
  const Scalar _tmp64 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp65 = -_tmp64;
  const Scalar _tmp66 =
      -Scalar(0.010999999999999999) * _tmp11 - Scalar(0.010999999999999999) * _tmp28;
  const Scalar _tmp67 = Scalar(0.20999999999999999) * _tmp14 - Scalar(0.20999999999999999) * _tmp15;
  const Scalar _tmp68 = _tmp66 - _tmp67;
  const Scalar _tmp69 = _tmp65 + _tmp68;
  const Scalar _tmp70 = _tmp66 + _tmp67;
  const Scalar _tmp71 = _tmp64 + _tmp70;
  const Scalar _tmp72 = _tmp59 * _tmp71;
  const Scalar _tmp73 = _tmp60 * (_tmp44 * _tmp69 - _tmp45 * _tmp72);
  const Scalar _tmp74 = Scalar(1.0) * _tmp46;
  const Scalar _tmp75 = -_tmp74;
  const Scalar _tmp76 = Scalar(1.0) / (_tmp38 + _tmp75);
  const Scalar _tmp77 = Scalar(1.0) * _tmp76;
  const Scalar _tmp78 = -_tmp45 * _tmp69 + _tmp45 * _tmp71;
  const Scalar _tmp79 = -_tmp61 * _tmp78 + _tmp63 * _tmp73 * _tmp77;
  const Scalar _tmp80 = _tmp21 + _tmp37;
  const Scalar _tmp81 = _tmp80 + position_vector(1, 0);
  const Scalar _tmp82 = _tmp81 + Scalar(8.3888750099999996);
  const Scalar _tmp83 = _tmp34 + _tmp49;
  const Scalar _tmp84 = _tmp83 + position_vector(0, 0);
  const Scalar _tmp85 = _tmp84 + Scalar(-2.5202214700000001);
  const Scalar _tmp86 = std::pow(Scalar(std::pow(_tmp82, Scalar(2)) + std::pow(_tmp85, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp87 = _tmp85 * _tmp86;
  const Scalar _tmp88 = _tmp65 + _tmp70;
  const Scalar _tmp89 = _tmp82 * _tmp86;
  const Scalar _tmp90 = _tmp59 * _tmp87 - _tmp89;
  const Scalar _tmp91 = -_tmp72 * _tmp87 - _tmp73 * _tmp90 + _tmp88 * _tmp89;
  const Scalar _tmp92 = _tmp63 * _tmp76;
  const Scalar _tmp93 = _tmp60 * _tmp90;
  const Scalar _tmp94 = _tmp71 * _tmp87 - _tmp78 * _tmp93 - _tmp87 * _tmp88 - _tmp91 * _tmp92;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp94);
  const Scalar _tmp96 = _tmp57 * _tmp87 - _tmp58 * _tmp93 + _tmp80 * _tmp87 - _tmp83 * _tmp89;
  const Scalar _tmp97 = _tmp95 * _tmp96;
  const Scalar _tmp98 = Scalar(1.0) / (_tmp96);
  const Scalar _tmp99 = _tmp94 * _tmp98;
  const Scalar _tmp100 = _tmp99 * (-_tmp58 * _tmp61 - _tmp79 * _tmp97);
  const Scalar _tmp101 = _tmp100 + _tmp79;
  const Scalar _tmp102 = _tmp90 * _tmp95;
  const Scalar _tmp103 = -_tmp101 * _tmp102 + Scalar(1.0);
  const Scalar _tmp104 = _tmp45 * _tmp60;
  const Scalar _tmp105 = _tmp87 * _tmp95;
  const Scalar _tmp106 = _tmp20 + Scalar(1.79662371);
  const Scalar _tmp107 = _tmp32 + Scalar(-4.8333311099999996);
  const Scalar _tmp108 =
      std::pow(Scalar(std::pow(_tmp106, Scalar(2)) + std::pow(_tmp107, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp109 = _tmp107 * _tmp108;
  const Scalar _tmp110 = _tmp109 * fh1;
  const Scalar _tmp111 = _tmp59 * _tmp60;
  const Scalar _tmp112 = _tmp59 * _tmp73 + _tmp72;
  const Scalar _tmp113 = _tmp111 * _tmp78 - _tmp112 * _tmp92 - _tmp71;
  const Scalar _tmp114 = _tmp99 * (_tmp111 * _tmp58 - _tmp113 * _tmp97 - _tmp57);
  const Scalar _tmp115 = _tmp113 + _tmp114;
  const Scalar _tmp116 = -_tmp102 * _tmp115 - _tmp59;
  const Scalar _tmp117 = _tmp106 * _tmp108;
  const Scalar _tmp118 = _tmp117 * fh1;
  const Scalar _tmp119 = Scalar(1.0) * _tmp98;
  const Scalar _tmp120 = _tmp45 * _tmp93;
  const Scalar _tmp121 = fh1 * (_tmp109 * _tmp19 - _tmp117 * _tmp31);
  const Scalar _tmp122 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp123 = _tmp62 + _tmp74 * _tmp92;
  const Scalar _tmp124 = 0;
  const Scalar _tmp125 = _tmp124 * _tmp95;
  const Scalar _tmp126 = _tmp56 * (-_tmp120 * _tmp125 + _tmp125 * _tmp87);
  const Scalar _tmp127 = -_tmp110 * _tmp56 * (_tmp101 * _tmp105 + _tmp103 * _tmp104) -
                         _tmp118 * _tmp56 * (_tmp104 * _tmp116 + _tmp105 * _tmp115 + Scalar(1.0)) -
                         _tmp121 * _tmp56 * (-_tmp119 * _tmp120 + _tmp119 * _tmp87) -
                         _tmp122 * _tmp126;
  const Scalar _tmp128 = std::pow(_tmp127, Scalar(-2));
  const Scalar _tmp129 = _tmp126 * _tmp128;
  const Scalar _tmp130 = _tmp75 + _tmp80;
  const Scalar _tmp131 = _tmp130 * _tmp92;
  const Scalar _tmp132 = Scalar(1.0) / (-_tmp131 + _tmp62 - _tmp83);
  const Scalar _tmp133 = Scalar(1.0) * _tmp132;
  const Scalar _tmp134 = _tmp130 * _tmp132;
  const Scalar _tmp135 = _tmp91 * _tmp95;
  const Scalar _tmp136 = _tmp112 + _tmp114 * _tmp134 - _tmp115 * _tmp135;
  const Scalar _tmp137 = _tmp100 * _tmp134 - _tmp101 * _tmp135 - Scalar(1.0) * _tmp73;
  const Scalar _tmp138 = _tmp133 * _tmp99;
  const Scalar _tmp139 = -_tmp119 * _tmp91 + _tmp130 * _tmp138;
  const Scalar _tmp140 = fh1 * (_tmp64 + _tmp68);
  const Scalar _tmp141 = _tmp117 * _tmp140 + Scalar(5.1796800000000003) * _tmp16 + _tmp19 * fv1;
  const Scalar _tmp142 = _tmp130 * _tmp76;
  const Scalar _tmp143 = Scalar(1.0) * _tmp133 * _tmp142 - Scalar(1.0) * _tmp133;
  const Scalar _tmp144 = _tmp123 * _tmp132;
  const Scalar _tmp145 = _tmp76 * (-_tmp124 * _tmp135 - _tmp130 * _tmp144 + _tmp75);
  const Scalar _tmp146 = -Scalar(1.0) * _tmp144 - Scalar(1.0) * _tmp145 + Scalar(1.0);
  const Scalar _tmp147 = -_tmp109 * _tmp140 - Scalar(5.1796800000000003) * _tmp25 - _tmp31 * fv1;
  const Scalar _tmp148 = _tmp76 * (_tmp131 * _tmp133 + Scalar(1.0));
  const Scalar _tmp149 = _tmp133 * _tmp92;
  const Scalar _tmp150 = -Scalar(1.0) * _tmp148 + Scalar(1.0) * _tmp149;
  const Scalar _tmp151 = Scalar(1.0) * _tmp110 * (_tmp100 * _tmp133 - _tmp137 * _tmp77) +
                         Scalar(1.0) * _tmp118 * (_tmp114 * _tmp133 - _tmp136 * _tmp77) +
                         Scalar(1.0) * _tmp121 * (_tmp138 - _tmp139 * _tmp77) + _tmp122 * _tmp146 +
                         _tmp141 * _tmp143 + _tmp147 * _tmp150;
  const Scalar _tmp152 = Scalar(1.0) / (_tmp127);
  const Scalar _tmp153 = std::asinh(_tmp151 * _tmp152);
  const Scalar _tmp154 = Scalar(1.0) * _tmp153;
  const Scalar _tmp155 = _tmp21 + _tmp26 + _tmp36;
  const Scalar _tmp156 =
      (-_tmp129 * _tmp151 + _tmp152 * (_tmp143 * _tmp19 - _tmp146 + _tmp150 * _tmp155)) /
      std::sqrt(Scalar(_tmp128 * std::pow(_tmp151, Scalar(2)) + 1));
  const Scalar _tmp157 = Scalar(9.6622558468725703) * _tmp127;
  const Scalar _tmp158 =
      -_tmp153 * _tmp157 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp47), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp51), Scalar(2))));
  const Scalar _tmp159 = Scalar(0.1034955) * _tmp152;
  const Scalar _tmp160 = _tmp158 * _tmp159;
  const Scalar _tmp161 = Scalar(9.6622558468725703) * _tmp126;
  const Scalar _tmp162 = _tmp133 * _tmp141;
  const Scalar _tmp163 = _tmp110 * _tmp137 * _tmp76 + _tmp118 * _tmp136 * _tmp76 +
                         _tmp121 * _tmp139 * _tmp76 + _tmp122 * _tmp145 - _tmp142 * _tmp162 +
                         _tmp147 * _tmp148;
  const Scalar _tmp164 = _tmp122 * _tmp125;
  const Scalar _tmp165 = _tmp119 * _tmp121;
  const Scalar _tmp166 =
      _tmp103 * _tmp110 * _tmp60 + _tmp116 * _tmp118 * _tmp60 - _tmp164 * _tmp93 - _tmp165 * _tmp93;
  const Scalar _tmp167 = Scalar(1.0) / (_tmp166);
  const Scalar _tmp168 = std::asinh(_tmp163 * _tmp167);
  const Scalar _tmp169 = Scalar(9.6622558468725703) * _tmp166;
  const Scalar _tmp170 =
      -_tmp168 * _tmp169 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp39 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp41 - 1), Scalar(2))));
  const Scalar _tmp171 = Scalar(0.1034955) * _tmp167;
  const Scalar _tmp172 = _tmp170 * _tmp171;
  const Scalar _tmp173 = Scalar(9.6622558468725703) * _tmp125;
  const Scalar _tmp174 = _tmp173 * _tmp93;
  const Scalar _tmp175 = _tmp133 * _tmp19;
  const Scalar _tmp176 = std::pow(_tmp166, Scalar(-2));
  const Scalar _tmp177 = _tmp125 * _tmp176 * _tmp93;
  const Scalar _tmp178 =
      (-_tmp163 * _tmp177 + _tmp167 * (-_tmp142 * _tmp175 - _tmp145 + _tmp148 * _tmp155)) /
      std::sqrt(Scalar(std::pow(_tmp163, Scalar(2)) * _tmp176 + 1));
  const Scalar _tmp179 = Scalar(1.0) * _tmp168;
  const Scalar _tmp180 = -_tmp100 * _tmp110 * _tmp132 - _tmp114 * _tmp118 * _tmp132 -
                         _tmp121 * _tmp138 + _tmp122 * _tmp144 - _tmp147 * _tmp149 + _tmp162;
  const Scalar _tmp181 =
      _tmp101 * _tmp110 * _tmp95 + _tmp115 * _tmp118 * _tmp95 + _tmp164 + _tmp165;
  const Scalar _tmp182 = Scalar(1.0) / (_tmp181);
  const Scalar _tmp183 = std::asinh(_tmp180 * _tmp182);
  const Scalar _tmp184 = Scalar(9.6622558468725703) * _tmp181;
  const Scalar _tmp185 =
      -_tmp183 * _tmp184 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp84), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp81 - 1), Scalar(2))));
  const Scalar _tmp186 = Scalar(0.1034955) * _tmp182;
  const Scalar _tmp187 = _tmp185 * _tmp186;
  const Scalar _tmp188 = Scalar(1.0) * _tmp183;
  const Scalar _tmp189 = std::pow(_tmp181, Scalar(-2));
  const Scalar _tmp190 = _tmp125 * _tmp189;
  const Scalar _tmp191 = (_tmp180 * _tmp190 + _tmp182 * (-_tmp144 - _tmp149 * _tmp155 + _tmp175)) /
                         std::sqrt(Scalar(std::pow(_tmp180, Scalar(2)) * _tmp189 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -_tmp33 *
      (_tmp2 * std::sinh(Scalar(1.0) * _tmp1) +
       _tmp2 * std::sinh(
                   Scalar(0.1034955) * _tmp0 *
                   (-_tmp1 * _tmp33 -
                    Scalar(4.8333311099999996) *
                        std::sqrt(Scalar(
                            std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp32), Scalar(2)) +
                            Scalar(0.13817235445745474) *
                                std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp20 - 1),
                                         Scalar(2)))))));
  _res(1, 0) =
      -_tmp157 *
          (-Scalar(0.86565325453551001) * _tmp129 + Scalar(1.0) * _tmp156 * std::sinh(_tmp154) -
           (-Scalar(0.1034955) * _tmp129 * _tmp158 +
            _tmp159 * (-_tmp153 * _tmp161 - _tmp156 * _tmp157)) *
               std::sinh(_tmp160)) -
      _tmp161 * (Scalar(0.86565325453551001) * _tmp152 + std::cosh(_tmp154) - std::cosh(_tmp160));
  _res(2, 0) =
      -_tmp169 *
          (-Scalar(0.87679799772039002) * _tmp177 + Scalar(1.0) * _tmp178 * std::sinh(_tmp179) -
           (-Scalar(0.1034955) * _tmp170 * _tmp177 +
            _tmp171 * (-_tmp168 * _tmp174 - _tmp169 * _tmp178)) *
               std::sinh(_tmp172)) -
      _tmp174 * (Scalar(0.87679799772039002) * _tmp167 - std::cosh(_tmp172) + std::cosh(_tmp179));
  _res(3, 0) =
      _tmp173 * (Scalar(0.87653584775870996) * _tmp182 - std::cosh(_tmp187) + std::cosh(_tmp188)) -
      _tmp184 *
          (Scalar(0.87653584775870996) * _tmp190 + Scalar(1.0) * _tmp191 * std::sinh(_tmp188) -
           (Scalar(0.1034955) * _tmp185 * _tmp190 +
            _tmp186 * (_tmp173 * _tmp183 - _tmp184 * _tmp191)) *
               std::sinh(_tmp187));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym