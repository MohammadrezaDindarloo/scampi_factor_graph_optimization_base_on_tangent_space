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
 * Symbolic function: IK_residual_func_cost2_wrt_fv1_Nl17
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFv1Nl17(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot, const Scalar p_init0,
    const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x, const Scalar rot_init_y,
    const Scalar rot_init_z, const Scalar rot_init_w, const Scalar epsilon) {
  // Total ops: 595

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();

  // Intermediate terms (189)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = std::asinh(_tmp0 * fv1);
  const Scalar _tmp2 = Scalar(1.0) * _tmp0 /
                       std::sqrt(Scalar(1 + std::pow(fv1, Scalar(2)) / std::pow(fh1, Scalar(2))));
  const Scalar _tmp3 = _DeltaRot[0] * rot_init_z + _DeltaRot[1] * rot_init_w -
                       _DeltaRot[2] * rot_init_x + _DeltaRot[3] * rot_init_y;
  const Scalar _tmp4 = -2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp5 = -_DeltaRot[0] * rot_init_y + _DeltaRot[1] * rot_init_x +
                       _DeltaRot[2] * rot_init_w + _DeltaRot[3] * rot_init_z;
  const Scalar _tmp6 = 1 - 2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp7 = Scalar(0.20999999999999999) * _tmp4 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp8 = _DeltaRot[0] * rot_init_w - _DeltaRot[1] * rot_init_z +
                       _DeltaRot[2] * rot_init_y + _DeltaRot[3] * rot_init_x;
  const Scalar _tmp9 = 2 * _tmp5 * _tmp8;
  const Scalar _tmp10 = -2 * _DeltaRot[0] * rot_init_x - 2 * _DeltaRot[1] * rot_init_y -
                        2 * _DeltaRot[2] * rot_init_z + 2 * _DeltaRot[3] * rot_init_w;
  const Scalar _tmp11 = _tmp10 * _tmp3;
  const Scalar _tmp12 = _tmp11 + _tmp9;
  const Scalar _tmp13 = -Scalar(0.010999999999999999) * _tmp12;
  const Scalar _tmp14 = 2 * _tmp3;
  const Scalar _tmp15 = _tmp14 * _tmp8;
  const Scalar _tmp16 = _tmp10 * _tmp5;
  const Scalar _tmp17 = Scalar(0.20999999999999999) * _tmp15 - Scalar(0.20999999999999999) * _tmp16;
  const Scalar _tmp18 = _tmp13 + _tmp17;
  const Scalar _tmp19 = _tmp18 + _tmp7;
  const Scalar _tmp20 = _tmp19 + p_init0;
  const Scalar _tmp21 = -2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp21 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp23 = _tmp14 * _tmp5;
  const Scalar _tmp24 = _tmp10 * _tmp8;
  const Scalar _tmp25 = _tmp23 - _tmp24;
  const Scalar _tmp26 = Scalar(0.010999999999999999) * _tmp25;
  const Scalar _tmp27 = -_tmp26;
  const Scalar _tmp28 = Scalar(0.20999999999999999) * _tmp15 + Scalar(0.20999999999999999) * _tmp16;
  const Scalar _tmp29 = _tmp27 + _tmp28;
  const Scalar _tmp30 = _tmp22 + _tmp29;
  const Scalar _tmp31 = _tmp30 + p_init1;
  const Scalar _tmp32 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp33 = -_tmp7;
  const Scalar _tmp34 = _tmp18 + _tmp33;
  const Scalar _tmp35 = _tmp34 + p_init0;
  const Scalar _tmp36 = -_tmp28;
  const Scalar _tmp37 = _tmp27 + _tmp36;
  const Scalar _tmp38 = _tmp22 + _tmp37;
  const Scalar _tmp39 = _tmp38 + p_init1;
  const Scalar _tmp40 = _tmp39 + Scalar(-4.8333311099999996);
  const Scalar _tmp41 = _tmp35 + Scalar(1.79662371);
  const Scalar _tmp42 = Scalar(1.0) / (_tmp41);
  const Scalar _tmp43 = _tmp40 * _tmp42;
  const Scalar _tmp44 = -_tmp22;
  const Scalar _tmp45 = _tmp37 + _tmp44;
  const Scalar _tmp46 = _tmp45 + p_init1;
  const Scalar _tmp47 = _tmp46 + Scalar(8.3196563700000006);
  const Scalar _tmp48 = _tmp13 - _tmp17;
  const Scalar _tmp49 = _tmp33 + _tmp48;
  const Scalar _tmp50 = _tmp49 + p_init0;
  const Scalar _tmp51 = _tmp50 + Scalar(1.9874742000000001);
  const Scalar _tmp52 = std::pow(Scalar(std::pow(_tmp47, Scalar(2)) + std::pow(_tmp51, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp53 = _tmp47 * _tmp52;
  const Scalar _tmp54 = _tmp51 * _tmp52;
  const Scalar _tmp55 = _tmp43 * _tmp54 - _tmp53;
  const Scalar _tmp56 = -Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp57 = -_tmp56;
  const Scalar _tmp58 = -Scalar(0.010999999999999999) * _tmp21 -
                        Scalar(0.010999999999999999) * _tmp4 + Scalar(-0.010999999999999999);
  const Scalar _tmp59 = Scalar(0.20999999999999999) * _tmp23 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp60 = _tmp58 + _tmp59;
  const Scalar _tmp61 = _tmp57 + _tmp60;
  const Scalar _tmp62 = _tmp48 + _tmp7;
  const Scalar _tmp63 = _tmp62 + p_init0;
  const Scalar _tmp64 = _tmp63 + Scalar(-2.5202214700000001);
  const Scalar _tmp65 = _tmp29 + _tmp44;
  const Scalar _tmp66 = _tmp65 + p_init1;
  const Scalar _tmp67 = _tmp66 + Scalar(8.3888750099999996);
  const Scalar _tmp68 = std::pow(Scalar(std::pow(_tmp64, Scalar(2)) + std::pow(_tmp67, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp69 = _tmp64 * _tmp68;
  const Scalar _tmp70 = _tmp58 - _tmp59;
  const Scalar _tmp71 = _tmp56 + _tmp70;
  const Scalar _tmp72 = _tmp61 * _tmp69 - _tmp69 * _tmp71;
  const Scalar _tmp73 = _tmp67 * _tmp68;
  const Scalar _tmp74 = Scalar(1.0) / (_tmp43 * _tmp69 - _tmp73);
  const Scalar _tmp75 = _tmp55 * _tmp74;
  const Scalar _tmp76 = _tmp57 + _tmp70;
  const Scalar _tmp77 = _tmp43 * _tmp61;
  const Scalar _tmp78 = -_tmp69 * _tmp77 + _tmp71 * _tmp73;
  const Scalar _tmp79 = _tmp53 * _tmp76 - _tmp54 * _tmp77 - _tmp75 * _tmp78;
  const Scalar _tmp80 = Scalar(1.0) * _tmp38;
  const Scalar _tmp81 = -_tmp80;
  const Scalar _tmp82 = Scalar(1.0) / (_tmp65 + _tmp81);
  const Scalar _tmp83 = Scalar(1.0) * _tmp34;
  const Scalar _tmp84 = _tmp82 * (-_tmp62 + _tmp83);
  const Scalar _tmp85 = _tmp54 * _tmp61 - _tmp54 * _tmp76 - _tmp72 * _tmp75 - _tmp79 * _tmp84;
  const Scalar _tmp86 = Scalar(1.0) / (_tmp85);
  const Scalar _tmp87 =
      std::sqrt(Scalar(std::pow(_tmp40, Scalar(2)) + std::pow(_tmp41, Scalar(2))));
  const Scalar _tmp88 = Scalar(1.0) / (_tmp87);
  const Scalar _tmp89 = _tmp42 * _tmp87;
  const Scalar _tmp90 = _tmp89 * (_tmp34 * _tmp40 * _tmp88 - _tmp38 * _tmp41 * _tmp88);
  const Scalar _tmp91 = -_tmp62 * _tmp73 + _tmp65 * _tmp69 + _tmp69 * _tmp90;
  const Scalar _tmp92 = _tmp43 * _tmp74;
  const Scalar _tmp93 = _tmp77 + _tmp78 * _tmp92;
  const Scalar _tmp94 = -_tmp61 + _tmp72 * _tmp92 - _tmp84 * _tmp93;
  const Scalar _tmp95 = _tmp45 * _tmp54 - _tmp49 * _tmp53 + _tmp54 * _tmp90 - _tmp75 * _tmp91;
  const Scalar _tmp96 = _tmp86 * _tmp95;
  const Scalar _tmp97 = Scalar(1.0) / (_tmp95);
  const Scalar _tmp98 = _tmp85 * _tmp97;
  const Scalar _tmp99 = _tmp98 * (-_tmp90 + _tmp91 * _tmp92 - _tmp94 * _tmp96);
  const Scalar _tmp100 = _tmp86 * (_tmp94 + _tmp99);
  const Scalar _tmp101 = -_tmp100 * _tmp55 - _tmp43;
  const Scalar _tmp102 = _tmp69 * _tmp74;
  const Scalar _tmp103 = _tmp20 + Scalar(-2.71799795);
  const Scalar _tmp104 = _tmp31 + Scalar(-4.7752063900000001);
  const Scalar _tmp105 =
      std::pow(Scalar(std::pow(_tmp103, Scalar(2)) + std::pow(_tmp104, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp106 = _tmp103 * _tmp105;
  const Scalar _tmp107 = _tmp106 * fh1;
  const Scalar _tmp108 = Scalar(1.0) * _tmp97;
  const Scalar _tmp109 = Scalar(1.0) * _tmp74;
  const Scalar _tmp110 = _tmp109 * _tmp55 * _tmp97;
  const Scalar _tmp111 = _tmp104 * _tmp105;
  const Scalar _tmp112 = fh1 * (-_tmp106 * _tmp30 + _tmp111 * _tmp19);
  const Scalar _tmp113 = _tmp109 * _tmp78;
  const Scalar _tmp114 = -_tmp109 * _tmp72 + _tmp113 * _tmp84;
  const Scalar _tmp115 = _tmp98 * (-_tmp109 * _tmp91 - _tmp114 * _tmp96);
  const Scalar _tmp116 = _tmp86 * (_tmp114 + _tmp115);
  const Scalar _tmp117 = -_tmp116 * _tmp55 + Scalar(1.0);
  const Scalar _tmp118 = _tmp111 * fh1;
  const Scalar _tmp119 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp120 = _tmp80 * _tmp84 + _tmp83;
  const Scalar _tmp121 = 0;
  const Scalar _tmp122 = _tmp121 * _tmp75;
  const Scalar _tmp123 = _tmp89 * (_tmp121 * _tmp54 - _tmp122 * _tmp69);
  const Scalar _tmp124 = -_tmp107 * _tmp89 * (_tmp100 * _tmp54 + _tmp101 * _tmp102 + Scalar(1.0)) -
                         _tmp112 * _tmp89 * (_tmp108 * _tmp54 - _tmp110 * _tmp69) -
                         _tmp118 * _tmp89 * (_tmp102 * _tmp117 + _tmp116 * _tmp54) -
                         _tmp119 * _tmp123;
  const Scalar _tmp125 = Scalar(1.0) / (_tmp124);
  const Scalar _tmp126 = fh1 * (_tmp56 + _tmp60);
  const Scalar _tmp127 = -_tmp111 * _tmp126 - Scalar(3.29616) * _tmp25 - _tmp30 * fv1;
  const Scalar _tmp128 = _tmp45 + _tmp81;
  const Scalar _tmp129 = _tmp128 * _tmp84;
  const Scalar _tmp130 = Scalar(1.0) / (-_tmp129 - _tmp49 + _tmp83);
  const Scalar _tmp131 = Scalar(1.0) * _tmp130;
  const Scalar _tmp132 = _tmp129 * _tmp131 + Scalar(1.0);
  const Scalar _tmp133 = Scalar(1.0) * _tmp82;
  const Scalar _tmp134 = _tmp131 * _tmp84;
  const Scalar _tmp135 = -Scalar(1.0) * _tmp132 * _tmp133 + Scalar(1.0) * _tmp134;
  const Scalar _tmp136 = _tmp128 * _tmp130;
  const Scalar _tmp137 = -_tmp113 + _tmp115 * _tmp136 - _tmp116 * _tmp79;
  const Scalar _tmp138 = _tmp120 * _tmp130;
  const Scalar _tmp139 = _tmp82 * (-_tmp121 * _tmp79 - _tmp128 * _tmp138 + _tmp81);
  const Scalar _tmp140 = -Scalar(1.0) * _tmp138 - Scalar(1.0) * _tmp139 + Scalar(1.0);
  const Scalar _tmp141 = _tmp131 * _tmp98;
  const Scalar _tmp142 = -_tmp108 * _tmp79 + _tmp128 * _tmp141;
  const Scalar _tmp143 = -_tmp100 * _tmp79 + _tmp136 * _tmp99 + _tmp93;
  const Scalar _tmp144 = _tmp106 * _tmp126 + Scalar(3.29616) * _tmp12 + _tmp19 * fv1;
  const Scalar _tmp145 = _tmp128 * _tmp82;
  const Scalar _tmp146 = Scalar(1.0) * _tmp131 * _tmp145 - Scalar(1.0) * _tmp131;
  const Scalar _tmp147 = Scalar(1.0) * _tmp107 * (_tmp131 * _tmp99 - _tmp133 * _tmp143) +
                         Scalar(1.0) * _tmp112 * (-_tmp133 * _tmp142 + _tmp141) +
                         Scalar(1.0) * _tmp118 * (_tmp115 * _tmp131 - _tmp133 * _tmp137) +
                         _tmp119 * _tmp140 + _tmp127 * _tmp135 + _tmp144 * _tmp146;
  const Scalar _tmp148 = std::asinh(_tmp125 * _tmp147);
  const Scalar _tmp149 = Scalar(9.6622558468725703) * _tmp124;
  const Scalar _tmp150 =
      -_tmp148 * _tmp149 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp39), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp35 - 1), Scalar(2))));
  const Scalar _tmp151 = Scalar(0.1034955) * _tmp125;
  const Scalar _tmp152 = _tmp150 * _tmp151;
  const Scalar _tmp153 = Scalar(1.0) * _tmp148;
  const Scalar _tmp154 = Scalar(9.6622558468725703) * _tmp123;
  const Scalar _tmp155 = std::pow(_tmp124, Scalar(-2));
  const Scalar _tmp156 = _tmp123 * _tmp155;
  const Scalar _tmp157 = _tmp26 + _tmp36 + _tmp44;
  const Scalar _tmp158 =
      (_tmp125 * (_tmp135 * _tmp157 - _tmp140 + _tmp146 * _tmp19) - _tmp147 * _tmp156) /
      std::sqrt(Scalar(std::pow(_tmp147, Scalar(2)) * _tmp155 + 1));
  const Scalar _tmp159 = _tmp132 * _tmp82;
  const Scalar _tmp160 = _tmp131 * _tmp144;
  const Scalar _tmp161 = _tmp107 * _tmp143 * _tmp82 + _tmp112 * _tmp142 * _tmp82 +
                         _tmp118 * _tmp137 * _tmp82 + _tmp119 * _tmp139 + _tmp127 * _tmp159 -
                         _tmp145 * _tmp160;
  const Scalar _tmp162 = _tmp119 * _tmp121;
  const Scalar _tmp163 = _tmp101 * _tmp107 * _tmp74 - _tmp110 * _tmp112 +
                         _tmp117 * _tmp118 * _tmp74 - _tmp162 * _tmp75;
  const Scalar _tmp164 = Scalar(1.0) / (_tmp163);
  const Scalar _tmp165 = std::asinh(_tmp161 * _tmp164);
  const Scalar _tmp166 = Scalar(9.6622558468725703) * _tmp163;
  const Scalar _tmp167 =
      -_tmp165 * _tmp166 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp63), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp66 - 1), Scalar(2))));
  const Scalar _tmp168 = std::pow(_tmp163, Scalar(-2));
  const Scalar _tmp169 = _tmp122 * _tmp168;
  const Scalar _tmp170 = Scalar(9.6622558468725703) * _tmp121;
  const Scalar _tmp171 = _tmp170 * _tmp75;
  const Scalar _tmp172 = _tmp131 * _tmp19;
  const Scalar _tmp173 =
      (-_tmp161 * _tmp169 + _tmp164 * (-_tmp139 - _tmp145 * _tmp172 + _tmp157 * _tmp159)) /
      std::sqrt(Scalar(std::pow(_tmp161, Scalar(2)) * _tmp168 + 1));
  const Scalar _tmp174 = Scalar(0.1034955) * _tmp164;
  const Scalar _tmp175 = _tmp167 * _tmp174;
  const Scalar _tmp176 = Scalar(1.0) * _tmp165;
  const Scalar _tmp177 = _tmp100 * _tmp107 + _tmp108 * _tmp112 + _tmp116 * _tmp118 + _tmp162;
  const Scalar _tmp178 = Scalar(1.0) / (_tmp177);
  const Scalar _tmp179 = -_tmp107 * _tmp130 * _tmp99 - _tmp112 * _tmp141 -
                         _tmp115 * _tmp118 * _tmp130 + _tmp119 * _tmp138 - _tmp127 * _tmp134 +
                         _tmp160;
  const Scalar _tmp180 = std::asinh(_tmp178 * _tmp179);
  const Scalar _tmp181 = Scalar(1.0) * _tmp180;
  const Scalar _tmp182 = std::pow(_tmp177, Scalar(-2));
  const Scalar _tmp183 = _tmp121 * _tmp182;
  const Scalar _tmp184 = (_tmp178 * (-_tmp134 * _tmp157 - _tmp138 + _tmp172) + _tmp179 * _tmp183) /
                         std::sqrt(Scalar(std::pow(_tmp179, Scalar(2)) * _tmp182 + 1));
  const Scalar _tmp185 = Scalar(9.6622558468725703) * _tmp177;
  const Scalar _tmp186 =
      -_tmp180 * _tmp185 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp46 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp50 - 1), Scalar(2))));
  const Scalar _tmp187 = Scalar(0.1034955) * _tmp178;
  const Scalar _tmp188 = _tmp186 * _tmp187;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      _tmp32 *
      (-_tmp2 * std::cosh(Scalar(1.0) * _tmp1) +
       _tmp2 *
           std::cosh(
               Scalar(0.1034955) * _tmp0 *
               (-_tmp1 * _tmp32 -
                Scalar(4.7752063900000001) *
                    std::sqrt(Scalar(
                        Scalar(0.32397683292140877) *
                            std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp20), Scalar(2)) +
                        std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp31), Scalar(2)))))));
  _res(1, 0) = _tmp149 * (-Scalar(1.0) * _tmp158 * std::cosh(_tmp153) -
                          (-Scalar(0.1034955) * _tmp150 * _tmp156 +
                           _tmp151 * (-_tmp148 * _tmp154 - _tmp149 * _tmp158)) *
                              std::cosh(_tmp152)) +
               _tmp154 * (-std::sinh(_tmp152) - std::sinh(_tmp153));
  _res(2, 0) = _tmp166 * (-Scalar(1.0) * _tmp173 * std::cosh(_tmp176) -
                          (-Scalar(0.1034955) * _tmp167 * _tmp169 +
                           _tmp174 * (-_tmp165 * _tmp171 - _tmp166 * _tmp173)) *
                              std::cosh(_tmp175)) +
               _tmp171 * (-std::sinh(_tmp175) - std::sinh(_tmp176));
  _res(3, 0) = -_tmp170 * (-std::sinh(_tmp181) - std::sinh(_tmp188)) +
               _tmp185 * (-Scalar(1.0) * _tmp184 * std::cosh(_tmp181) -
                          (Scalar(0.1034955) * _tmp183 * _tmp186 +
                           _tmp187 * (_tmp170 * _tmp180 - _tmp184 * _tmp185)) *
                              std::cosh(_tmp188));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym