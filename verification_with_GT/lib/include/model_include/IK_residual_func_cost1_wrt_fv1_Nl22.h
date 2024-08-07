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
 *     p_init0: Scalar
 *     p_init1: Scalar
 *     p_init2: Scalar
 *     rot_init_x: Scalar
 *     rot_init_y: Scalar
 *     rot_init_z: Scalar
 *     rot_init_w: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix41
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFv1Nl22(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot, const Scalar p_init0,
    const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x, const Scalar rot_init_y,
    const Scalar rot_init_z, const Scalar rot_init_w, const Scalar epsilon) {
  // Total ops: 605

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();

  // Intermediate terms (193)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(1.0) * _tmp0 /
                       std::sqrt(Scalar(1 + std::pow(fv1, Scalar(2)) / std::pow(fh1, Scalar(2))));
  const Scalar _tmp3 = _DeltaRot[0] * rot_init_w - _DeltaRot[1] * rot_init_z +
                       _DeltaRot[2] * rot_init_y + _DeltaRot[3] * rot_init_x;
  const Scalar _tmp4 = _DeltaRot[0] * rot_init_z + _DeltaRot[1] * rot_init_w -
                       _DeltaRot[2] * rot_init_x + _DeltaRot[3] * rot_init_y;
  const Scalar _tmp5 = 2 * _tmp4;
  const Scalar _tmp6 = _tmp3 * _tmp5;
  const Scalar _tmp7 = -_DeltaRot[0] * rot_init_x - _DeltaRot[1] * rot_init_y -
                       _DeltaRot[2] * rot_init_z + _DeltaRot[3] * rot_init_w;
  const Scalar _tmp8 = -_DeltaRot[0] * rot_init_y + _DeltaRot[1] * rot_init_x +
                       _DeltaRot[2] * rot_init_w + _DeltaRot[3] * rot_init_z;
  const Scalar _tmp9 = 2 * _tmp8;
  const Scalar _tmp10 = _tmp7 * _tmp9;
  const Scalar _tmp11 = -Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp12 = -2 * std::pow(_tmp4, Scalar(2));
  const Scalar _tmp13 = 1 - 2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 = _tmp3 * _tmp9;
  const Scalar _tmp16 = _tmp5 * _tmp7;
  const Scalar _tmp17 = _tmp15 + _tmp16;
  const Scalar _tmp18 = -Scalar(0.010999999999999999) * _tmp17;
  const Scalar _tmp19 = -_tmp14 + _tmp18;
  const Scalar _tmp20 = _tmp11 + _tmp19;
  const Scalar _tmp21 = _tmp20 + p_init0;
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp23 = -_tmp22;
  const Scalar _tmp24 = _tmp5 * _tmp8;
  const Scalar _tmp25 = 2 * _tmp3 * _tmp7;
  const Scalar _tmp26 = _tmp24 - _tmp25;
  const Scalar _tmp27 = Scalar(0.010999999999999999) * _tmp26;
  const Scalar _tmp28 = -_tmp27;
  const Scalar _tmp29 = -2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp30 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp29;
  const Scalar _tmp31 = _tmp28 + _tmp30;
  const Scalar _tmp32 = _tmp23 + _tmp31;
  const Scalar _tmp33 = _tmp32 + p_init1;
  const Scalar _tmp34 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp35 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp36 = _tmp22 + _tmp31;
  const Scalar _tmp37 = Scalar(1.0) * _tmp36;
  const Scalar _tmp38 = -_tmp37;
  const Scalar _tmp39 = -_tmp30;
  const Scalar _tmp40 = _tmp28 + _tmp39;
  const Scalar _tmp41 = _tmp23 + _tmp40;
  const Scalar _tmp42 = Scalar(1.0) / (_tmp38 + _tmp41);
  const Scalar _tmp43 = -_tmp11;
  const Scalar _tmp44 = _tmp19 + _tmp43;
  const Scalar _tmp45 = _tmp14 + _tmp18;
  const Scalar _tmp46 = _tmp11 + _tmp45;
  const Scalar _tmp47 = Scalar(1.0) * _tmp46;
  const Scalar _tmp48 = _tmp42 * (-_tmp44 + _tmp47);
  const Scalar _tmp49 = _tmp37 * _tmp48 + _tmp47;
  const Scalar _tmp50 = 0;
  const Scalar _tmp51 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp25;
  const Scalar _tmp52 = -_tmp51;
  const Scalar _tmp53 = -Scalar(0.010999999999999999) * _tmp12 -
                        Scalar(0.010999999999999999) * _tmp29 + Scalar(-0.010999999999999999);
  const Scalar _tmp54 = Scalar(0.20999999999999999) * _tmp15 - Scalar(0.20999999999999999) * _tmp16;
  const Scalar _tmp55 = _tmp53 - _tmp54;
  const Scalar _tmp56 = _tmp52 + _tmp55;
  const Scalar _tmp57 = _tmp41 + p_init1;
  const Scalar _tmp58 = _tmp57 + Scalar(8.3196563700000006);
  const Scalar _tmp59 = _tmp44 + p_init0;
  const Scalar _tmp60 = _tmp59 + Scalar(1.9874742000000001);
  const Scalar _tmp61 = std::pow(Scalar(std::pow(_tmp58, Scalar(2)) + std::pow(_tmp60, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp62 = _tmp60 * _tmp61;
  const Scalar _tmp63 = _tmp53 + _tmp54;
  const Scalar _tmp64 = _tmp51 + _tmp63;
  const Scalar _tmp65 = -_tmp56 * _tmp62 + _tmp62 * _tmp64;
  const Scalar _tmp66 = _tmp58 * _tmp61;
  const Scalar _tmp67 = _tmp46 + p_init0;
  const Scalar _tmp68 = _tmp67 + Scalar(-2.71799795);
  const Scalar _tmp69 = Scalar(1.0) / (_tmp68);
  const Scalar _tmp70 = _tmp36 + p_init1;
  const Scalar _tmp71 = _tmp70 + Scalar(-4.7752063900000001);
  const Scalar _tmp72 = _tmp69 * _tmp71;
  const Scalar _tmp73 = Scalar(1.0) / (_tmp62 * _tmp72 - _tmp66);
  const Scalar _tmp74 = _tmp43 + _tmp45;
  const Scalar _tmp75 = _tmp74 + p_init0;
  const Scalar _tmp76 = _tmp75 + Scalar(-2.5202214700000001);
  const Scalar _tmp77 = _tmp22 + _tmp40;
  const Scalar _tmp78 = _tmp77 + p_init1;
  const Scalar _tmp79 = _tmp78 + Scalar(8.3888750099999996);
  const Scalar _tmp80 = std::pow(Scalar(std::pow(_tmp76, Scalar(2)) + std::pow(_tmp79, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp81 = _tmp79 * _tmp80;
  const Scalar _tmp82 = _tmp76 * _tmp80;
  const Scalar _tmp83 = _tmp72 * _tmp82 - _tmp81;
  const Scalar _tmp84 = _tmp73 * _tmp83;
  const Scalar _tmp85 = _tmp52 + _tmp63;
  const Scalar _tmp86 = _tmp64 * _tmp72;
  const Scalar _tmp87 = _tmp56 * _tmp66 - _tmp62 * _tmp86;
  const Scalar _tmp88 = _tmp81 * _tmp85 - _tmp82 * _tmp86 - _tmp84 * _tmp87;
  const Scalar _tmp89 = -_tmp48 * _tmp88 + _tmp64 * _tmp82 - _tmp65 * _tmp84 - _tmp82 * _tmp85;
  const Scalar _tmp90 = Scalar(1.0) / (_tmp89);
  const Scalar _tmp91 = _tmp88 * _tmp90;
  const Scalar _tmp92 = _tmp38 + _tmp77;
  const Scalar _tmp93 = _tmp48 * _tmp92;
  const Scalar _tmp94 = Scalar(1.0) / (_tmp47 - _tmp74 - _tmp93);
  const Scalar _tmp95 = _tmp49 * _tmp94;
  const Scalar _tmp96 = _tmp42 * (_tmp38 - _tmp50 * _tmp91 - _tmp92 * _tmp95);
  const Scalar _tmp97 = Scalar(1.0) * _tmp94;
  const Scalar _tmp98 = -Scalar(1.0) * _tmp49 * _tmp97 - Scalar(1.0) * _tmp96 + Scalar(1.0);
  const Scalar _tmp99 = _tmp33 + Scalar(-4.8333311099999996);
  const Scalar _tmp100 = _tmp21 + Scalar(1.79662371);
  const Scalar _tmp101 =
      std::pow(Scalar(std::pow(_tmp100, Scalar(2)) + std::pow(_tmp99, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp102 = _tmp101 * _tmp99;
  const Scalar _tmp103 = fh1 * (_tmp51 + _tmp55);
  const Scalar _tmp104 = -_tmp102 * _tmp103 - Scalar(3.29616) * _tmp26 - _tmp32 * fv1;
  const Scalar _tmp105 = _tmp48 * _tmp97;
  const Scalar _tmp106 = _tmp42 * (_tmp93 * _tmp97 + Scalar(1.0));
  const Scalar _tmp107 = Scalar(1.0) * _tmp105 - Scalar(1.0) * _tmp106;
  const Scalar _tmp108 =
      std::sqrt(Scalar(std::pow(_tmp68, Scalar(2)) + std::pow(_tmp71, Scalar(2))));
  const Scalar _tmp109 = Scalar(1.0) / (_tmp108);
  const Scalar _tmp110 = _tmp108 * _tmp69;
  const Scalar _tmp111 = _tmp110 * (-_tmp109 * _tmp36 * _tmp68 + _tmp109 * _tmp46 * _tmp71);
  const Scalar _tmp112 = _tmp72 * _tmp73;
  const Scalar _tmp113 = _tmp112 * _tmp87 + _tmp86;
  const Scalar _tmp114 = _tmp112 * _tmp65 - _tmp113 * _tmp48 - _tmp64;
  const Scalar _tmp115 = _tmp111 * _tmp62 + _tmp41 * _tmp62 - _tmp44 * _tmp66;
  const Scalar _tmp116 = _tmp111 * _tmp82 - _tmp115 * _tmp84 - _tmp74 * _tmp81 + _tmp77 * _tmp82;
  const Scalar _tmp117 = _tmp116 * _tmp90;
  const Scalar _tmp118 = Scalar(1.0) / (_tmp116);
  const Scalar _tmp119 = _tmp118 * _tmp89;
  const Scalar _tmp120 = _tmp119 * (-_tmp111 + _tmp112 * _tmp115 - _tmp114 * _tmp117);
  const Scalar _tmp121 = _tmp114 + _tmp120;
  const Scalar _tmp122 = _tmp92 * _tmp94;
  const Scalar _tmp123 = _tmp113 + _tmp120 * _tmp122 - _tmp121 * _tmp91;
  const Scalar _tmp124 = Scalar(1.0) * _tmp42;
  const Scalar _tmp125 = _tmp100 * _tmp101;
  const Scalar _tmp126 = _tmp125 * fh1;
  const Scalar _tmp127 = Scalar(1.0) * _tmp73;
  const Scalar _tmp128 = _tmp127 * _tmp87;
  const Scalar _tmp129 = -_tmp127 * _tmp65 + _tmp128 * _tmp48;
  const Scalar _tmp130 = _tmp119 * (-_tmp115 * _tmp127 - _tmp117 * _tmp129);
  const Scalar _tmp131 = _tmp129 + _tmp130;
  const Scalar _tmp132 = _tmp122 * _tmp130 - _tmp128 - _tmp131 * _tmp91;
  const Scalar _tmp133 = _tmp102 * fh1;
  const Scalar _tmp134 = _tmp119 * _tmp97;
  const Scalar _tmp135 = Scalar(1.0) * _tmp118;
  const Scalar _tmp136 = _tmp134 * _tmp92 - _tmp135 * _tmp88;
  const Scalar _tmp137 = fh1 * (_tmp102 * _tmp20 - _tmp125 * _tmp32);
  const Scalar _tmp138 = _tmp103 * _tmp125 + Scalar(3.29616) * _tmp17 + _tmp20 * fv1;
  const Scalar _tmp139 = _tmp42 * _tmp92;
  const Scalar _tmp140 = Scalar(1.0) * _tmp139 * _tmp97 - Scalar(1.0) * _tmp97;
  const Scalar _tmp141 =
      _tmp104 * _tmp107 + Scalar(1.0) * _tmp126 * (_tmp120 * _tmp97 - _tmp123 * _tmp124) +
      Scalar(1.0) * _tmp133 * (-_tmp124 * _tmp132 + _tmp130 * _tmp97) +
      Scalar(1.0) * _tmp137 * (-_tmp124 * _tmp136 + _tmp134) + _tmp138 * _tmp140 + _tmp35 * _tmp98;
  const Scalar _tmp142 = _tmp82 * _tmp90;
  const Scalar _tmp143 = _tmp83 * _tmp90;
  const Scalar _tmp144 = -_tmp131 * _tmp143 + Scalar(1.0);
  const Scalar _tmp145 = _tmp62 * _tmp73;
  const Scalar _tmp146 = -_tmp121 * _tmp143 - _tmp72;
  const Scalar _tmp147 = _tmp118 * _tmp127 * _tmp83;
  const Scalar _tmp148 = _tmp50 * _tmp90;
  const Scalar _tmp149 = _tmp148 * _tmp84;
  const Scalar _tmp150 = _tmp110 * (_tmp148 * _tmp82 - _tmp149 * _tmp62);
  const Scalar _tmp151 =
      -_tmp110 * _tmp126 * (_tmp121 * _tmp142 + _tmp145 * _tmp146 + Scalar(1.0)) -
      _tmp110 * _tmp133 * (_tmp131 * _tmp142 + _tmp144 * _tmp145) -
      _tmp110 * _tmp137 * (_tmp135 * _tmp82 - _tmp147 * _tmp62) - _tmp150 * _tmp35;
  const Scalar _tmp152 = Scalar(1.0) / (_tmp151);
  const Scalar _tmp153 = std::asinh(_tmp141 * _tmp152);
  const Scalar _tmp154 = Scalar(1.0) * _tmp153;
  const Scalar _tmp155 = Scalar(9.6622558468725703) * _tmp151;
  const Scalar _tmp156 =
      -_tmp153 * _tmp155 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp67), Scalar(2)) +
                     std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp70), Scalar(2))));
  const Scalar _tmp157 = Scalar(0.1034955) * _tmp152;
  const Scalar _tmp158 = _tmp156 * _tmp157;
  const Scalar _tmp159 = Scalar(9.6622558468725703) * _tmp150;
  const Scalar _tmp160 = std::pow(_tmp151, Scalar(-2));
  const Scalar _tmp161 = _tmp22 + _tmp27 + _tmp39;
  const Scalar _tmp162 = _tmp150 * _tmp160;
  const Scalar _tmp163 =
      (-_tmp141 * _tmp162 + _tmp152 * (_tmp107 * _tmp161 + _tmp140 * _tmp20 - _tmp98)) /
      std::sqrt(Scalar(std::pow(_tmp141, Scalar(2)) * _tmp160 + 1));
  const Scalar _tmp164 = _tmp148 * _tmp35;
  const Scalar _tmp165 = _tmp126 * _tmp146 * _tmp73 + _tmp133 * _tmp144 * _tmp73 -
                         _tmp137 * _tmp147 - _tmp164 * _tmp84;
  const Scalar _tmp166 = Scalar(1.0) / (_tmp165);
  const Scalar _tmp167 = _tmp138 * _tmp97;
  const Scalar _tmp168 = _tmp104 * _tmp106 + _tmp123 * _tmp126 * _tmp42 +
                         _tmp132 * _tmp133 * _tmp42 + _tmp136 * _tmp137 * _tmp42 -
                         _tmp139 * _tmp167 + _tmp35 * _tmp96;
  const Scalar _tmp169 = std::asinh(_tmp166 * _tmp168);
  const Scalar _tmp170 = Scalar(1.0) * _tmp169;
  const Scalar _tmp171 = std::pow(_tmp165, Scalar(-2));
  const Scalar _tmp172 = _tmp149 * _tmp171;
  const Scalar _tmp173 = _tmp20 * _tmp97;
  const Scalar _tmp174 =
      (_tmp166 * (_tmp106 * _tmp161 - _tmp139 * _tmp173 - _tmp96) - _tmp168 * _tmp172) /
      std::sqrt(Scalar(std::pow(_tmp168, Scalar(2)) * _tmp171 + 1));
  const Scalar _tmp175 = Scalar(9.6622558468725703) * _tmp148;
  const Scalar _tmp176 = _tmp175 * _tmp84;
  const Scalar _tmp177 = Scalar(9.6622558468725703) * _tmp165;
  const Scalar _tmp178 = Scalar(0.1034955) * _tmp166;
  const Scalar _tmp179 =
      -_tmp169 * _tmp177 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp57 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp59 - 1), Scalar(2))));
  const Scalar _tmp180 = _tmp178 * _tmp179;
  const Scalar _tmp181 = -_tmp104 * _tmp105 - _tmp120 * _tmp126 * _tmp94 -
                         _tmp130 * _tmp133 * _tmp94 - _tmp134 * _tmp137 + _tmp167 + _tmp35 * _tmp95;
  const Scalar _tmp182 =
      _tmp121 * _tmp126 * _tmp90 + _tmp131 * _tmp133 * _tmp90 + _tmp135 * _tmp137 + _tmp164;
  const Scalar _tmp183 = Scalar(1.0) / (_tmp182);
  const Scalar _tmp184 = std::asinh(_tmp181 * _tmp183);
  const Scalar _tmp185 = Scalar(9.6622558468725703) * _tmp182;
  const Scalar _tmp186 =
      -_tmp184 * _tmp185 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp75), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp78 - 1), Scalar(2))));
  const Scalar _tmp187 = Scalar(0.1034955) * _tmp183;
  const Scalar _tmp188 = _tmp186 * _tmp187;
  const Scalar _tmp189 = std::pow(_tmp182, Scalar(-2));
  const Scalar _tmp190 = _tmp148 * _tmp189;
  const Scalar _tmp191 = (_tmp181 * _tmp190 + _tmp183 * (-_tmp105 * _tmp161 + _tmp173 - _tmp95)) /
                         std::sqrt(Scalar(std::pow(_tmp181, Scalar(2)) * _tmp189 + 1));
  const Scalar _tmp192 = Scalar(1.0) * _tmp184;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -_tmp34 *
      (_tmp2 * std::sinh(Scalar(1.0) * _tmp1) +
       _tmp2 * std::sinh(
                   Scalar(0.1034955) * _tmp0 *
                   (-_tmp1 * _tmp34 -
                    Scalar(4.8333311099999996) *
                        std::sqrt(Scalar(
                            std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp33), Scalar(2)) +
                            Scalar(0.13817235445745474) *
                                std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp21 - 1),
                                         Scalar(2)))))));
  _res(1, 0) =
      -_tmp155 *
          (-Scalar(0.86565325453551001) * _tmp162 + Scalar(1.0) * _tmp163 * std::sinh(_tmp154) -
           (-Scalar(0.1034955) * _tmp156 * _tmp162 +
            _tmp157 * (-_tmp153 * _tmp159 - _tmp155 * _tmp163)) *
               std::sinh(_tmp158)) -
      _tmp159 * (Scalar(0.86565325453551001) * _tmp152 + std::cosh(_tmp154) - std::cosh(_tmp158));
  _res(2, 0) =
      -_tmp176 * (Scalar(0.87679799772039002) * _tmp166 + std::cosh(_tmp170) - std::cosh(_tmp180)) -
      _tmp177 *
          (-Scalar(0.87679799772039002) * _tmp172 + Scalar(1.0) * _tmp174 * std::sinh(_tmp170) -
           (-Scalar(0.1034955) * _tmp172 * _tmp179 +
            _tmp178 * (-_tmp169 * _tmp176 - _tmp174 * _tmp177)) *
               std::sinh(_tmp180));
  _res(3, 0) =
      _tmp175 * (Scalar(0.87653584775870996) * _tmp183 - std::cosh(_tmp188) + std::cosh(_tmp192)) -
      _tmp185 *
          (Scalar(0.87653584775870996) * _tmp190 + Scalar(1.0) * _tmp191 * std::sinh(_tmp192) -
           (Scalar(0.1034955) * _tmp186 * _tmp190 +
            _tmp187 * (_tmp175 * _tmp184 - _tmp185 * _tmp191)) *
               std::sinh(_tmp188));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
