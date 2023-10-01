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
 * Symbolic function: IK_residual_func_cost1_wrt_fv1_Nl0
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost1WrtFv1Nl0(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot, const Scalar p_init0,
    const Scalar p_init1, const Scalar p_init2, const Scalar rot_init_x, const Scalar rot_init_y,
    const Scalar rot_init_z, const Scalar rot_init_w, const Scalar epsilon) {
  // Total ops: 609

  // Unused inputs
  (void)p_init2;
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();

  // Intermediate terms (194)
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
  const Scalar _tmp8 = -_tmp7;
  const Scalar _tmp9 = _DeltaRot[0] * rot_init_w - _DeltaRot[1] * rot_init_z +
                       _DeltaRot[2] * rot_init_y + _DeltaRot[3] * rot_init_x;
  const Scalar _tmp10 = 2 * _tmp9;
  const Scalar _tmp11 = _tmp10 * _tmp5;
  const Scalar _tmp12 = -2 * _DeltaRot[0] * rot_init_x - 2 * _DeltaRot[1] * rot_init_y -
                        2 * _DeltaRot[2] * rot_init_z + 2 * _DeltaRot[3] * rot_init_w;
  const Scalar _tmp13 = _tmp12 * _tmp3;
  const Scalar _tmp14 = _tmp11 + _tmp13;
  const Scalar _tmp15 = -Scalar(0.010999999999999999) * _tmp14;
  const Scalar _tmp16 = _tmp10 * _tmp3;
  const Scalar _tmp17 = _tmp12 * _tmp5;
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp16 - Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp19 = _tmp15 - _tmp18;
  const Scalar _tmp20 = _tmp19 + _tmp8;
  const Scalar _tmp21 = _tmp20 + p_init0;
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp16 + Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp23 = -_tmp22;
  const Scalar _tmp24 = -2 * std::pow(_tmp9, Scalar(2));
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp24 + Scalar(0.20999999999999999) * _tmp6;
  const Scalar _tmp26 = 2 * _tmp3 * _tmp5;
  const Scalar _tmp27 = _tmp12 * _tmp9;
  const Scalar _tmp28 = _tmp26 - _tmp27;
  const Scalar _tmp29 = Scalar(0.010999999999999999) * _tmp28;
  const Scalar _tmp30 = -_tmp29;
  const Scalar _tmp31 = -_tmp25 + _tmp30;
  const Scalar _tmp32 = _tmp23 + _tmp31;
  const Scalar _tmp33 = _tmp32 + p_init1;
  const Scalar _tmp34 = Scalar(9.6622558468725703) * fh1;
  const Scalar _tmp35 = _tmp19 + _tmp7;
  const Scalar _tmp36 = _tmp35 + p_init0;
  const Scalar _tmp37 = _tmp36 + Scalar(-2.5202214700000001);
  const Scalar _tmp38 = _tmp22 + _tmp31;
  const Scalar _tmp39 = _tmp38 + p_init1;
  const Scalar _tmp40 = _tmp39 + Scalar(8.3888750099999996);
  const Scalar _tmp41 =
      std::sqrt(Scalar(std::pow(_tmp37, Scalar(2)) + std::pow(_tmp40, Scalar(2))));
  const Scalar _tmp42 = Scalar(1.0) / (_tmp41);
  const Scalar _tmp43 = Scalar(1.0) / (_tmp37);
  const Scalar _tmp44 = _tmp41 * _tmp43;
  const Scalar _tmp45 = _tmp44 * (_tmp35 * _tmp40 * _tmp42 - _tmp37 * _tmp38 * _tmp42);
  const Scalar _tmp46 = _tmp15 + _tmp18;
  const Scalar _tmp47 = _tmp46 + _tmp7;
  const Scalar _tmp48 = _tmp47 + p_init0;
  const Scalar _tmp49 = _tmp48 + Scalar(-2.71799795);
  const Scalar _tmp50 = _tmp22 + _tmp25;
  const Scalar _tmp51 = _tmp30 + _tmp50;
  const Scalar _tmp52 = _tmp51 + p_init1;
  const Scalar _tmp53 = _tmp52 + Scalar(-4.7752063900000001);
  const Scalar _tmp54 = std::pow(Scalar(std::pow(_tmp49, Scalar(2)) + std::pow(_tmp53, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp55 = _tmp49 * _tmp54;
  const Scalar _tmp56 = _tmp53 * _tmp54;
  const Scalar _tmp57 = _tmp45 * _tmp55 - _tmp47 * _tmp56 + _tmp51 * _tmp55;
  const Scalar _tmp58 = _tmp40 * _tmp43;
  const Scalar _tmp59 = Scalar(1.0) / (_tmp55 * _tmp58 - _tmp56);
  const Scalar _tmp60 = _tmp58 * _tmp59;
  const Scalar _tmp61 = Scalar(0.20999999999999999) * _tmp26 + Scalar(0.20999999999999999) * _tmp27;
  const Scalar _tmp62 = -_tmp61;
  const Scalar _tmp63 = -Scalar(0.010999999999999999) * _tmp24 -
                        Scalar(0.010999999999999999) * _tmp4 + Scalar(-0.010999999999999999);
  const Scalar _tmp64 = Scalar(0.20999999999999999) * _tmp11 - Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp65 = _tmp63 + _tmp64;
  const Scalar _tmp66 = _tmp62 + _tmp65;
  const Scalar _tmp67 = _tmp58 * _tmp66;
  const Scalar _tmp68 = _tmp61 + _tmp65;
  const Scalar _tmp69 = -_tmp55 * _tmp67 + _tmp56 * _tmp68;
  const Scalar _tmp70 = _tmp60 * _tmp69 + _tmp67;
  const Scalar _tmp71 = Scalar(1.0) * _tmp38;
  const Scalar _tmp72 = -_tmp71;
  const Scalar _tmp73 = Scalar(1.0) / (_tmp51 + _tmp72);
  const Scalar _tmp74 = Scalar(1.0) * _tmp35;
  const Scalar _tmp75 = -_tmp47 + _tmp74;
  const Scalar _tmp76 = _tmp73 * _tmp75;
  const Scalar _tmp77 = _tmp55 * _tmp66 - _tmp55 * _tmp68;
  const Scalar _tmp78 = _tmp60 * _tmp77 - _tmp66 - _tmp70 * _tmp76;
  const Scalar _tmp79 = _tmp23 + _tmp25 + _tmp30;
  const Scalar _tmp80 = _tmp79 + p_init1;
  const Scalar _tmp81 = _tmp80 + Scalar(-4.8333311099999996);
  const Scalar _tmp82 = _tmp46 + _tmp8;
  const Scalar _tmp83 = _tmp82 + p_init0;
  const Scalar _tmp84 = _tmp83 + Scalar(1.79662371);
  const Scalar _tmp85 = std::pow(Scalar(std::pow(_tmp81, Scalar(2)) + std::pow(_tmp84, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp86 = _tmp81 * _tmp85;
  const Scalar _tmp87 = _tmp84 * _tmp85;
  const Scalar _tmp88 = _tmp58 * _tmp87 - _tmp86;
  const Scalar _tmp89 = _tmp59 * _tmp88;
  const Scalar _tmp90 = _tmp66 * _tmp87;
  const Scalar _tmp91 = _tmp63 - _tmp64;
  const Scalar _tmp92 = _tmp61 + _tmp91;
  const Scalar _tmp93 = -_tmp58 * _tmp90 - _tmp69 * _tmp89 + _tmp86 * _tmp92;
  const Scalar _tmp94 = -_tmp76 * _tmp93 - _tmp77 * _tmp89 - _tmp87 * _tmp92 + _tmp90;
  const Scalar _tmp95 = Scalar(1.0) / (_tmp94);
  const Scalar _tmp96 = _tmp45 * _tmp87 - _tmp57 * _tmp89 + _tmp79 * _tmp87 - _tmp82 * _tmp86;
  const Scalar _tmp97 = _tmp95 * _tmp96;
  const Scalar _tmp98 = Scalar(1.0) / (_tmp96);
  const Scalar _tmp99 = _tmp94 * _tmp98;
  const Scalar _tmp100 = _tmp99 * (-_tmp45 + _tmp57 * _tmp60 - _tmp78 * _tmp97);
  const Scalar _tmp101 = _tmp100 + _tmp78;
  const Scalar _tmp102 = _tmp88 * _tmp95;
  const Scalar _tmp103 = -_tmp101 * _tmp102 - _tmp58;
  const Scalar _tmp104 = _tmp55 * _tmp59;
  const Scalar _tmp105 = _tmp87 * _tmp95;
  const Scalar _tmp106 = _tmp33 + Scalar(8.3196563700000006);
  const Scalar _tmp107 = _tmp21 + Scalar(1.9874742000000001);
  const Scalar _tmp108 =
      std::pow(Scalar(std::pow(_tmp106, Scalar(2)) + std::pow(_tmp107, Scalar(2))),
               Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp109 = _tmp107 * _tmp108;
  const Scalar _tmp110 = _tmp109 * fh1;
  const Scalar _tmp111 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp112 = _tmp71 * _tmp76 + _tmp74;
  const Scalar _tmp113 = 0;
  const Scalar _tmp114 = _tmp113 * _tmp95;
  const Scalar _tmp115 = _tmp114 * _tmp89;
  const Scalar _tmp116 = _tmp44 * (_tmp114 * _tmp87 - _tmp115 * _tmp55);
  const Scalar _tmp117 = Scalar(1.0) * _tmp59;
  const Scalar _tmp118 = Scalar(1.0) * _tmp73;
  const Scalar _tmp119 = -_tmp117 * _tmp77 + _tmp118 * _tmp59 * _tmp69 * _tmp75;
  const Scalar _tmp120 = _tmp99 * (-_tmp117 * _tmp57 - _tmp119 * _tmp97);
  const Scalar _tmp121 = _tmp119 + _tmp120;
  const Scalar _tmp122 = -_tmp102 * _tmp121 + Scalar(1.0);
  const Scalar _tmp123 = _tmp106 * _tmp108;
  const Scalar _tmp124 = _tmp123 * fh1;
  const Scalar _tmp125 = Scalar(1.0) * _tmp98;
  const Scalar _tmp126 = fh1 * (-_tmp109 * _tmp32 + _tmp123 * _tmp20);
  const Scalar _tmp127 = -_tmp110 * _tmp44 * (_tmp101 * _tmp105 + _tmp103 * _tmp104 + Scalar(1.0)) -
                         _tmp111 * _tmp116 -
                         _tmp124 * _tmp44 * (_tmp104 * _tmp122 + _tmp105 * _tmp121) -
                         _tmp126 * _tmp44 * (-_tmp125 * _tmp55 * _tmp89 + _tmp125 * _tmp87);
  const Scalar _tmp128 = std::pow(_tmp127, Scalar(-2));
  const Scalar _tmp129 = _tmp116 * _tmp128;
  const Scalar _tmp130 = fh1 * (_tmp62 + _tmp91);
  const Scalar _tmp131 = _tmp109 * _tmp130 + Scalar(3.29616) * _tmp14 + _tmp20 * fv1;
  const Scalar _tmp132 = _tmp72 + _tmp79;
  const Scalar _tmp133 = _tmp132 * _tmp76;
  const Scalar _tmp134 = Scalar(1.0) / (-_tmp133 + _tmp74 - _tmp82);
  const Scalar _tmp135 = Scalar(1.0) * _tmp134;
  const Scalar _tmp136 = _tmp132 * _tmp73;
  const Scalar _tmp137 = Scalar(1.0) * _tmp135 * _tmp136 - Scalar(1.0) * _tmp135;
  const Scalar _tmp138 = _tmp135 * _tmp99;
  const Scalar _tmp139 = -_tmp125 * _tmp93 + _tmp132 * _tmp138;
  const Scalar _tmp140 = _tmp112 * _tmp134;
  const Scalar _tmp141 = _tmp93 * _tmp95;
  const Scalar _tmp142 = _tmp73 * (-_tmp113 * _tmp141 - _tmp132 * _tmp140 + _tmp72);
  const Scalar _tmp143 = -Scalar(1.0) * _tmp140 - Scalar(1.0) * _tmp142 + Scalar(1.0);
  const Scalar _tmp144 = _tmp132 * _tmp134;
  const Scalar _tmp145 = _tmp100 * _tmp144 - _tmp101 * _tmp141 + _tmp70;
  const Scalar _tmp146 = Scalar(1.0) * fh1;
  const Scalar _tmp147 = -_tmp123 * _tmp130 - Scalar(3.29616) * _tmp28 - _tmp32 * fv1;
  const Scalar _tmp148 = _tmp133 * _tmp135 + Scalar(1.0);
  const Scalar _tmp149 = _tmp135 * _tmp76;
  const Scalar _tmp150 = -Scalar(1.0) * _tmp118 * _tmp148 + Scalar(1.0) * _tmp149;
  const Scalar _tmp151 = -_tmp117 * _tmp69 + _tmp120 * _tmp144 - _tmp121 * _tmp141;
  const Scalar _tmp152 = _tmp109 * _tmp146 * (_tmp100 * _tmp135 - _tmp118 * _tmp145) +
                         _tmp111 * _tmp143 +
                         _tmp123 * _tmp146 * (-_tmp118 * _tmp151 + _tmp120 * _tmp135) +
                         Scalar(1.0) * _tmp126 * (-_tmp118 * _tmp139 + _tmp138) +
                         _tmp131 * _tmp137 + _tmp147 * _tmp150;
  const Scalar _tmp153 = Scalar(1.0) / (_tmp127);
  const Scalar _tmp154 = std::asinh(_tmp152 * _tmp153);
  const Scalar _tmp155 = Scalar(9.6622558468725703) * _tmp127;
  const Scalar _tmp156 =
      -_tmp154 * _tmp155 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp36), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp39 - 1), Scalar(2))));
  const Scalar _tmp157 = Scalar(0.1034955) * _tmp153;
  const Scalar _tmp158 = _tmp156 * _tmp157;
  const Scalar _tmp159 = _tmp29 + _tmp50;
  const Scalar _tmp160 =
      (-_tmp129 * _tmp152 + _tmp153 * (_tmp137 * _tmp20 - _tmp143 + _tmp150 * _tmp159)) /
      std::sqrt(Scalar(_tmp128 * std::pow(_tmp152, Scalar(2)) + 1));
  const Scalar _tmp161 = Scalar(9.6622558468725703) * _tmp116;
  const Scalar _tmp162 = Scalar(1.0) * _tmp154;
  const Scalar _tmp163 = _tmp125 * _tmp126;
  const Scalar _tmp164 = _tmp111 * _tmp114;
  const Scalar _tmp165 =
      _tmp103 * _tmp110 * _tmp59 + _tmp122 * _tmp124 * _tmp59 - _tmp163 * _tmp89 - _tmp164 * _tmp89;
  const Scalar _tmp166 = std::pow(_tmp165, Scalar(-2));
  const Scalar _tmp167 = _tmp115 * _tmp166;
  const Scalar _tmp168 = _tmp148 * _tmp73;
  const Scalar _tmp169 = _tmp131 * _tmp135;
  const Scalar _tmp170 = _tmp110 * _tmp145 * _tmp73 + _tmp111 * _tmp142 +
                         _tmp124 * _tmp151 * _tmp73 + _tmp126 * _tmp139 * _tmp73 -
                         _tmp136 * _tmp169 + _tmp147 * _tmp168;
  const Scalar _tmp171 = Scalar(1.0) / (_tmp165);
  const Scalar _tmp172 = std::asinh(_tmp170 * _tmp171);
  const Scalar _tmp173 = Scalar(9.6622558468725703) * _tmp165;
  const Scalar _tmp174 =
      -_tmp172 * _tmp173 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp48), Scalar(2)) +
                     std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp52), Scalar(2))));
  const Scalar _tmp175 = Scalar(0.1034955) * _tmp171;
  const Scalar _tmp176 = _tmp174 * _tmp175;
  const Scalar _tmp177 = Scalar(9.6622558468725703) * _tmp114;
  const Scalar _tmp178 = _tmp177 * _tmp89;
  const Scalar _tmp179 = _tmp135 * _tmp20;
  const Scalar _tmp180 =
      (-_tmp167 * _tmp170 + _tmp171 * (-_tmp136 * _tmp179 - _tmp142 + _tmp159 * _tmp168)) /
      std::sqrt(Scalar(_tmp166 * std::pow(_tmp170, Scalar(2)) + 1));
  const Scalar _tmp181 = Scalar(1.0) * _tmp172;
  const Scalar _tmp182 =
      _tmp101 * _tmp110 * _tmp95 + _tmp121 * _tmp124 * _tmp95 + _tmp163 + _tmp164;
  const Scalar _tmp183 = Scalar(1.0) / (_tmp182);
  const Scalar _tmp184 = -_tmp100 * _tmp110 * _tmp134 + _tmp111 * _tmp140 -
                         _tmp120 * _tmp124 * _tmp134 - _tmp126 * _tmp138 - _tmp147 * _tmp149 +
                         _tmp169;
  const Scalar _tmp185 = std::asinh(_tmp183 * _tmp184);
  const Scalar _tmp186 = Scalar(9.6622558468725703) * _tmp182;
  const Scalar _tmp187 =
      -_tmp185 * _tmp186 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp80), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp83 - 1), Scalar(2))));
  const Scalar _tmp188 = Scalar(0.1034955) * _tmp183;
  const Scalar _tmp189 = _tmp187 * _tmp188;
  const Scalar _tmp190 = Scalar(1.0) * _tmp185;
  const Scalar _tmp191 = std::pow(_tmp182, Scalar(-2));
  const Scalar _tmp192 = _tmp114 * _tmp191;
  const Scalar _tmp193 = (_tmp183 * (-_tmp140 - _tmp149 * _tmp159 + _tmp179) + _tmp184 * _tmp192) /
                         std::sqrt(Scalar(std::pow(_tmp184, Scalar(2)) * _tmp191 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) =
      -_tmp34 *
      (_tmp2 * std::sinh(Scalar(1.0) * _tmp1) +
       _tmp2 *
           std::sinh(
               Scalar(0.1034955) * _tmp0 *
               (-_tmp1 * _tmp34 -
                Scalar(8.3196563700000006) *
                    std::sqrt(Scalar(
                        Scalar(0.057067943376852184) *
                            std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp21 - 1), Scalar(2)) +
                        std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp33 - 1), Scalar(2)))))));
  _res(1, 0) =
      -_tmp155 *
          (-Scalar(0.87653584775870996) * _tmp129 + Scalar(1.0) * _tmp160 * std::sinh(_tmp162) -
           (-Scalar(0.1034955) * _tmp129 * _tmp156 +
            _tmp157 * (-_tmp154 * _tmp161 - _tmp155 * _tmp160)) *
               std::sinh(_tmp158)) -
      _tmp161 * (Scalar(0.87653584775870996) * _tmp153 - std::cosh(_tmp158) + std::cosh(_tmp162));
  _res(2, 0) =
      -_tmp173 *
          (-Scalar(0.86565325453551001) * _tmp167 + Scalar(1.0) * _tmp180 * std::sinh(_tmp181) -
           (-Scalar(0.1034955) * _tmp167 * _tmp174 +
            _tmp175 * (-_tmp172 * _tmp178 - _tmp173 * _tmp180)) *
               std::sinh(_tmp176)) -
      _tmp178 * (Scalar(0.86565325453551001) * _tmp171 - std::cosh(_tmp176) + std::cosh(_tmp181));
  _res(3, 0) =
      _tmp177 * (Scalar(0.86625939559540499) * _tmp183 - std::cosh(_tmp189) + std::cosh(_tmp190)) -
      _tmp186 *
          (Scalar(0.86625939559540499) * _tmp192 + Scalar(1.0) * _tmp193 * std::sinh(_tmp190) -
           (Scalar(0.1034955) * _tmp187 * _tmp192 +
            _tmp188 * (_tmp177 * _tmp185 - _tmp186 * _tmp193)) *
               std::sinh(_tmp189));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
